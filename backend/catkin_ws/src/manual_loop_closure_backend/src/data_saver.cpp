#include "data_saver.h"
#include "runtime_config.h"

#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <cstdio>
#include <ctime>
#include <iomanip>
#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/Geoid.hpp>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/inference/Symbol.h>

#include "ms_mapping/ms_mapping.h"
#include "common/runtime_logger.h"
#include "common/custom_voxel_filter.hpp"
#include "tools/lidar_compress_module.hpp"

#include <boost/make_shared.hpp>
#include <future>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <XmlRpcValue.h>
#include <fstream>
#include <memory>
#include <iterator>
#include <limits>
#include <mutex>
#include <set>
#include <unordered_set>
#include <vector>
#include <regex>
#include <sstream>
#include <system_error>
#include <optional>
#include <cstdlib>
#include <cctype>
#include <stdexcept>

namespace {
constexpr float kEpsilon = 1e-6f;
constexpr int kG2oAnchorId = std::numeric_limits<int>::max() - 1;
constexpr const char *kCalibrationParamRosKey = "/ms_mapping/runtime_calibration_param_path";
constexpr const char *kAnsiReset = "\033[0m";
constexpr const char *kAnsiTumPathColor = "\033[1;32m";
constexpr const char *kAnsiG2oPathColor = "\033[1;36m";

inline std::string ColorizePath(const std::string &path, const char *color_code)
{
    return std::string(color_code) + path + kAnsiReset;
}

template <typename Scalar>
inline void BodyToLidar(const Eigen::Quaterniond &q_body_sensor,
                        const Eigen::Vector3d &t_body_sensor,
                        Scalar &x, Scalar &y, Scalar &z)
{
    Eigen::Vector3d translation(static_cast<double>(x), static_cast<double>(y), static_cast<double>(z));
    translation -= t_body_sensor;
    translation = q_body_sensor.conjugate() * translation;
    x = static_cast<Scalar>(translation[0]);
    y = static_cast<Scalar>(translation[1]);
    z = static_cast<Scalar>(translation[2]);
}

void EmitYamlScalar(std::ostream &out, const XmlRpc::XmlRpcValue &value)
{
    using XmlRpc::XmlRpcValue;
    switch (value.getType())
    {
    case XmlRpcValue::TypeBoolean:
        out << (static_cast<bool>(value) ? "true" : "false");
        break;
    case XmlRpcValue::TypeInt:
        out << static_cast<int>(value);
        break;
    case XmlRpcValue::TypeDouble:
        out << static_cast<double>(value);
        break;
    case XmlRpcValue::TypeString:
        out << "\"" << static_cast<std::string>(value) << "\"";
        break;
    case XmlRpcValue::TypeDateTime:
    case XmlRpcValue::TypeBase64:
        out << '"' << static_cast<std::string>(value) << '"';
        break;
    case XmlRpcValue::TypeInvalid:
    default:
        out << "null";
        break;
    }
}

void EmitYamlValue(std::ostream &out, XmlRpc::XmlRpcValue &value, int indent);

void EmitYamlKey(std::ostream &out, const std::string &key, XmlRpc::XmlRpcValue &value, int indent)
{
    using XmlRpc::XmlRpcValue;
    std::string padding(indent, ' ');
    if (value.getType() == XmlRpcValue::TypeStruct || value.getType() == XmlRpcValue::TypeArray)
    {
        out << padding << key << ":";
        EmitYamlValue(out, value, indent + 2);
    }
    else
    {
        out << padding << key << ": ";
        EmitYamlScalar(out, value);
        out << '\n';
    }
}

void EmitYamlValue(std::ostream &out, XmlRpc::XmlRpcValue &value, int indent)
{
    using XmlRpc::XmlRpcValue;
    switch (value.getType())
    {
    case XmlRpcValue::TypeStruct:
    {
        out << '\n';
        for (auto it = value.begin(); it != value.end(); ++it)
        {
            EmitYamlKey(out, it->first, it->second, indent);
        }
        break;
    }
    case XmlRpcValue::TypeArray:
    {
        out << '\n';
        for (int i = 0; i < value.size(); ++i)
        {
            XmlRpc::XmlRpcValue &item = value[i];
            std::string padding(indent, ' ');
            out << padding << "-";
            if (item.getType() == XmlRpcValue::TypeStruct || item.getType() == XmlRpcValue::TypeArray)
            {
                EmitYamlValue(out, item, indent + 2);
            }
            else
            {
                out << ' ';
                EmitYamlScalar(out, item);
                out << '\n';
            }
        }
        break;
    }
    default:
        out << ' ';
        EmitYamlScalar(out, value);
        out << '\n';
        break;
    }
}

bool XmlRpcValueToString(const XmlRpc::XmlRpcValue &value, std::string &out)
{
    using XmlRpc::XmlRpcValue;
    switch (value.getType())
    {
    case XmlRpcValue::TypeBoolean:
        out = static_cast<bool>(value) ? "true" : "false";
        return true;
    case XmlRpcValue::TypeInt:
        out = std::to_string(static_cast<int>(value));
        return true;
    case XmlRpcValue::TypeDouble:
    {
        std::ostringstream oss;
        oss << std::setprecision(15) << static_cast<double>(value);
        out = oss.str();
        return true;
    }
    case XmlRpcValue::TypeString:
        out = static_cast<std::string>(value);
        return true;
    case XmlRpcValue::TypeDateTime:
    case XmlRpcValue::TypeBase64:
        out = static_cast<std::string>(value);
        return true;
    default:
        break;
    }
    return false;
}

bool ExtractNumericField(const std::string &content, const std::string &key, double &value)
{
    try
    {
        const std::regex pattern(key + R"([^0-9+\-]*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?))",
                                 std::regex::icase);
        std::smatch match;
        if (std::regex_search(content, match, pattern) && match.size() > 1)
        {
            value = std::stod(match[1].str());
            return true;
        }
    }
    catch (const std::exception &e)
    {
        LOG_WARN_STREAM("Failed to extract numeric field '" << key << "': " << e.what());
    }
    return false;
}

struct GnssNoiseSerialization
{
    Eigen::VectorXd sigmas;
    std::string robust_type{"NONE"};
    double robust_param{0.0};
};

std::string IdentifyRobustKernel(const noiseModel::mEstimator::Base::shared_ptr &kernel, double &parameter)
{
    parameter = 0.0;
    if (!kernel)
        return "NONE";

    // 使用未限定的 dynamic_pointer_cast，通过 ADL 在 std/boost 间自动选择，实现 GTSAM 版本兼容。
    if (auto cauchy = dynamic_pointer_cast<noiseModel::mEstimator::Cauchy>(kernel))
    {
        parameter = cauchy->modelParameter();
        return "CAUCHY";
    }
    if (auto huber = dynamic_pointer_cast<noiseModel::mEstimator::Huber>(kernel))
    {
        parameter = huber->modelParameter();
        return "HUBER";
    }
    if (auto tukey = dynamic_pointer_cast<noiseModel::mEstimator::Tukey>(kernel))
    {
        parameter = tukey->modelParameter();
        return "TUKEY";
    }
    parameter = 0.0;
    return "UNKNOWN";
}

bool ExtractNoiseSerialization(const gtsam::SharedNoiseModel &model, GnssNoiseSerialization &serialization)
{
    serialization.sigmas.resize(0);
    serialization.robust_type = "NONE";
    serialization.robust_param = 0.0;
    if (!model)
        return false;

    SharedNoiseModel base = model;
    if (auto robust = dynamic_pointer_cast<noiseModel::Robust>(model))
    {
        base = robust->noise();
        serialization.robust_type = IdentifyRobustKernel(robust->robust(), serialization.robust_param);
    }

    if (auto diag = dynamic_pointer_cast<noiseModel::Diagonal>(base))
    {
        serialization.sigmas = diag->sigmas();
        return true;
    }

    if (auto gaussian = dynamic_pointer_cast<noiseModel::Gaussian>(base))
    {
        const gtsam::Matrix cov = gaussian->covariance();
        serialization.sigmas = cov.diagonal().cwiseSqrt();
        return true;
    }

    return false;
}

SharedNoiseModel ReconstructNoiseModel(const GnssNoiseSerialization &serialization)
{
    if (serialization.sigmas.size() == 0)
        return SharedNoiseModel();

    auto base = noiseModel::Diagonal::Sigmas(serialization.sigmas);

    if (serialization.robust_type == "CAUCHY")
    {
        auto kernel = noiseModel::mEstimator::Cauchy::Create(serialization.robust_param);
        return noiseModel::Robust::Create(kernel, base);
    }
    if (serialization.robust_type == "HUBER")
    {
        auto kernel = noiseModel::mEstimator::Huber::Create(serialization.robust_param);
        return noiseModel::Robust::Create(kernel, base);
    }
    if (serialization.robust_type == "TUKEY")
    {
        auto kernel = noiseModel::mEstimator::Tukey::Create(serialization.robust_param);
        return noiseModel::Robust::Create(kernel, base);
    }

    // UNKNOWN or NONE fall back to base noise.
    return base;
}

bool ExtractInformationMatrix(const SharedNoiseModel &noise, Eigen::Matrix<double, 6, 6> &information)
{
    SharedNoiseModel base = noise;
    if (!base)
        return false;

    if (auto robust = dynamic_pointer_cast<noiseModel::Robust>(base))
    {
        base = robust->noise();
    }

    if (auto diagonal = dynamic_pointer_cast<noiseModel::Diagonal>(base))
    {
        information.setZero();
        const auto &precisions = diagonal->precisions();
        for (int i = 0; i < std::min<int>(precisions.size(), 6); ++i)
        {
            information(i, i) = precisions(i);
        }
        return true;
    }

    if (auto gaussian = dynamic_pointer_cast<noiseModel::Gaussian>(base))
    {
        information = gaussian->information();
        return true;
    }

    return false;
}

// g2o stores the EDGE_SE3:QUAT information in the order [x y z | rx ry rz] (translation first);
// GTSAM's Pose3 tangent space is [rx ry rz | x y z] (rotation first). gtsam::writeG2o/readG2o apply
// this permutation; the custom reader and the anchor writer below must apply it as well.
// The permutation is its own inverse.
Eigen::Matrix<double, 6, 6> SwapPose3InformationOrder(const Eigen::Matrix<double, 6, 6> &information)
{
    Eigen::Matrix<double, 6, 6> swapped;
    swapped.block<3, 3>(0, 0) = information.block<3, 3>(3, 3);
    swapped.block<3, 3>(0, 3) = information.block<3, 3>(3, 0);
    swapped.block<3, 3>(3, 0) = information.block<3, 3>(0, 3);
    swapped.block<3, 3>(3, 3) = information.block<3, 3>(0, 0);
    return swapped;
}

void WriteUpperTriangularInformation(std::ostream &out, const Eigen::Matrix<double, 6, 6> &information)
{
    for (int i = 0; i < 6; ++i)
    {
        for (int j = i; j < 6; ++j)
        {
            out << ' ' << information(i, j);
        }
    }
}

void AppendAnchorPriorToG2o(const std::string &filepath,
                            const gtsam::NonlinearFactorGraph &primary,
                            const gtsam::NonlinearFactorGraph &secondary)
{
    const PriorFactor<Pose3> *selected_prior = nullptr;

    auto locate_prior = [&](const gtsam::NonlinearFactorGraph &graph) {
        for (const auto &factor_ptr : graph)
        {
            if (!factor_ptr)
                continue;
            if (const auto *prior = dynamic_cast<const PriorFactor<Pose3> *>(factor_ptr.get()))
            {
                Symbol key(prior->keys()[0]);
                if (key.chr() == 'X' && key.index() == 0)
                {
                    selected_prior = prior;
                    return;
                }
            }
        }
    };

    locate_prior(primary);
    if (!selected_prior)
        locate_prior(secondary);

    if (!selected_prior)
        return;

    Eigen::Matrix<double, 6, 6> information;
    if (!ExtractInformationMatrix(selected_prior->noiseModel(), information))
        return;

    std::ofstream out(filepath, std::ios::app);
    if (!out.is_open())
    {
        std::cerr << "[DataSaver] Failed to append anchor prior to " << filepath << std::endl;
        return;
    }

    out << std::scientific;
    out << std::setprecision(15);

    out << "# Anchor vertex for pose prior" << '\n';
    out << "VERTEX_SE3:QUAT " << kG2oAnchorId << " 0 0 0 0 0 0 1" << '\n';

    Symbol key(selected_prior->keys()[0]);
    Pose3 pose = selected_prior->prior();
    const Point3 t = pose.translation();
    const auto q = pose.rotation().toQuaternion();

    out << "EDGE_SE3:QUAT " << kG2oAnchorId << ' ' << key.index() << ' '
        << t.x() << ' ' << t.y() << ' ' << t.z() << ' '
        << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w();
    // The anchor prior is an EDGE_SE3:QUAT line, so its information is written in g2o order.
    WriteUpperTriangularInformation(out, SwapPose3InformationOrder(information));
    out << '\n';
}

void AppendGnssFactorsToG2o(const std::string &filepath, const gtsam::NonlinearFactorGraph &factors)
{
    std::ofstream out(filepath, std::ios::app);
    if (!out.is_open())
    {
        std::cerr << "[DataSaver] Failed to append GNSS priors to " << filepath << std::endl;
        return;
    }

    out << std::scientific;
    out << std::setprecision(15);

    bool header_written = false;

    const auto write_header = [&](void)
    {
        if (!header_written)
        {
            out << "# GNSS prior factors serialized by MS-Mapping" << '\n';
            header_written = true;
        }
    };

    for (const auto &factor_ptr : factors)
    {
        if (!factor_ptr)
            continue;

        GnssNoiseSerialization noise_info;

        if (const auto *gps = dynamic_cast<const gtsam::GPSFactor *>(factor_ptr.get()))
        {
            if (!ExtractNoiseSerialization(gps->noiseModel(), noise_info))
                continue;

            write_header();
            const Key key = gps->keys()[0];
            const Point3 &measurement = gps->measurementIn();
            const auto sigma_count = static_cast<int>(noise_info.sigmas.size());

            out << "# GNSS_PRIOR XYZ " << Symbol(key).index() << ' '
                << measurement.x() << ' ' << measurement.y() << ' ' << measurement.z() << ' '
                << sigma_count;
            for (int idx = 0; idx < sigma_count; ++idx)
            {
                out << ' ' << noise_info.sigmas(idx);
            }
            out << ' ' << noise_info.robust_type << ' ' << noise_info.robust_param << '\n';
            continue;
        }

        if (const auto *pose_prior = dynamic_cast<const gtsam::PriorFactor<Pose3> *>(factor_ptr.get()))
        {
            if (!ExtractNoiseSerialization(pose_prior->noiseModel(), noise_info))
                continue;

            write_header();
            const Key key = pose_prior->keys()[0];
            const Pose3 &pose = pose_prior->prior();
            const Point3 t = pose.translation();
            const auto quat = pose.rotation().toQuaternion();
            const auto sigma_count = static_cast<int>(noise_info.sigmas.size());

            out << "# GNSS_PRIOR POSE " << Symbol(key).index() << ' '
                << t.x() << ' ' << t.y() << ' ' << t.z() << ' '
                << quat.x() << ' ' << quat.y() << ' ' << quat.z() << ' ' << quat.w() << ' '
                << sigma_count;
            for (int idx = 0; idx < sigma_count; ++idx)
            {
                out << ' ' << noise_info.sigmas(idx);
            }
            out << ' ' << noise_info.robust_type << ' ' << noise_info.robust_param << '\n';
            continue;
        }

        if (const auto *xy_prior = dynamic_cast<const XYPriorFactor *>(factor_ptr.get()))
        {
            if (!ExtractNoiseSerialization(xy_prior->noiseModel(), noise_info))
                continue;

            write_header();
            const Key key = xy_prior->key();
            const Point2 &measurement = xy_prior->measured();
            const auto sigma_count = static_cast<int>(noise_info.sigmas.size());

            out << "# GNSS_PRIOR XY " << Symbol(key).index() << ' '
                << measurement.x() << ' ' << measurement.y() << ' ' << sigma_count;
            for (int idx = 0; idx < sigma_count; ++idx)
            {
                out << ' ' << noise_info.sigmas(idx);
            }
            out << ' ' << noise_info.robust_type << ' ' << noise_info.robust_param << '\n';
        }
    }

    if (header_written)
    {
        out.flush();
    }
}

#if __has_include(<filesystem>)
namespace fs = std::filesystem;
#else
#error "std::filesystem is required"
#endif
}

pcl::PointCloud<PointT>::Ptr DataSaver::AcquirePointIScratch(bool clear)
{
    if (!scratch_pointi_)
    {
        scratch_pointi_.reset(new pcl::PointCloud<PointT>());
    }
    if (clear)
    {
        scratch_pointi_->clear();
    }
    return scratch_pointi_;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr DataSaver::AcquireTrajectoryScratch(bool clear)
{
    if (!scratch_traj_xyz_)
    {
        scratch_traj_xyz_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    }
    if (clear)
    {
        scratch_traj_xyz_->clear();
    }
    return scratch_traj_xyz_;
}

void DataSaver::UpdateTransformCache()
{
    Eigen::Matrix3d R = q_body_sensor.toRotationMatrix();
    Eigen::Matrix3d Rt = R.transpose();
    body_to_lidar_matrix_d_.setIdentity();
    body_to_lidar_matrix_d_.block<3, 3>(0, 0) = Rt;
    body_to_lidar_matrix_d_.block<3, 1>(0, 3) = -Rt * t_body_sensor;
    body_to_lidar_matrix_f_ = body_to_lidar_matrix_d_.cast<float>();
    body_to_lidar_cached_ = true;
}

void DataSaver::EnsureTransformCache() const
{
    if (!body_to_lidar_cached_)
    {
        const_cast<DataSaver *>(this)->UpdateTransformCache();
    }
}

const Eigen::Matrix4f &DataSaver::BodyToLidarMatrixF() const
{
    EnsureTransformCache();
    return body_to_lidar_matrix_f_;
}

DataSaver::DataSaver()
{
    q_body_sensor = Eigen::Quaterniond::Identity();
    t_body_sensor = Eigen::Vector3d::Zero();
    UpdateTransformCache();
}

DataSaver::~DataSaver() {}

DataSaver::DataSaver(string _base_dir, string _sequence_name)
{
    q_body_sensor = Eigen::Quaterniond::Identity();
    t_body_sensor = Eigen::Vector3d::Zero();

    this->base_dir = _base_dir;
    this->sequence_name = _sequence_name;

    if (_base_dir.back() != '/')
    {
        _base_dir.append("/");
    }
    save_directory = _base_dir + sequence_name + '/';
    LOG_INFO_STREAM("[DataSaver] Save directory: " << save_directory);

    auto unused = system((std::string("exec rm -r ") + save_directory).c_str());
    unused = system((std::string("mkdir -p ") + save_directory).c_str());

    InitializeLocalizationOutputs();
}

void DataSaver::InitializeLocalizationOutputs()
{
    if (save_directory.empty())
    {
        return;
    }

    namespace fs = std::filesystem;

    pcdmap_root_path_.clear();
    localization_map_path_.clear();
    logs_path_.clear();

    pcdmap_root_path_ = save_directory + "pcdmap";
    if (!pcdmap_root_path_.empty() && pcdmap_root_path_.back() == '/')
    {
        pcdmap_root_path_.pop_back();
    }

    std::error_code ec;
    fs::remove_all(pcdmap_root_path_, ec);
    ec.clear();
    fs::create_directories(pcdmap_root_path_, ec);
    if (ec)
    {
        LOG_WARN_STREAM("[DataSaver] Failed to create pcdmap directory: " << pcdmap_root_path_
                        << ", " << ec.message());
    }

    localization_map_path_ = pcdmap_root_path_ + "/maps";
    fs::create_directories(localization_map_path_, ec);
    if (ec)
    {
        LOG_WARN_STREAM("[DataSaver] Failed to create maps directory: " << localization_map_path_
                        << ", " << ec.message());
    }

    // Create logs directory
    logs_directory_name_ = EnsureLogDirectoryStamp();
    if (logs_directory_name_.empty())
    {
        logs_directory_name_ = "logs";
    }

    logs_path_ = save_directory + logs_directory_name_;
    if (!logs_path_.empty() && logs_path_.back() == '/')
    {
        logs_path_.pop_back();
    }
    fs::create_directories(logs_path_, ec);
    if (ec)
    {
        LOG_WARN_STREAM("[DataSaver] Failed to create logs directory: " << logs_path_
                        << ", " << ec.message());
    }

    // Create sensor data目录（默认直接放置各传感器bin文件，位于根目录，非归档内容）
    sensor_data_path_ = save_directory + "sensor_data";
    if (!sensor_data_path_.empty() && sensor_data_path_.back() == '/')
    {
        sensor_data_path_.pop_back();
    }
    fs::create_directories(sensor_data_path_, ec);
    if (ec)
    {
        LOG_WARN_STREAM("[DataSaver] Failed to create sensor_data directory: "
                        << sensor_data_path_ << ", " << ec.message());
    }
    else
    {
        LOG_INFO_STREAM("[DataSaver] sensor_data directory ready at " << sensor_data_path_
                        << " (请直接放置各传感器的 .bin 文件)");
    }

    localization_grid_size_ = (localization_map_grid_size > 0.0) ? localization_map_grid_size : 20.0;
    localization_downsample_resolution_ = (grid_map_downsample_size > 0.0) ? grid_map_downsample_size : 0.2;
    intensity_image_resolution_ = (intensity_image_resolution > 0.0)
                                      ? intensity_image_resolution
                                      : localization_downsample_resolution_;
    export_grid_map_ = save_grid_map;
    export_intensity_image_ = save_intensity_image;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr DataSaver::DownSample(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud) const
{
    if (!cloud || cloud->empty())
    {
        return pcl::PointCloud<pcl::PointXYZI>::Ptr();
    }

    auto filtered = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    float leaf = static_cast<float>(localization_downsample_resolution_);
    constexpr float kDefaultGridLeaf = 0.2f;
    if (leaf <= 0.0f)
    {
        leaf = kDefaultGridLeaf;
        LOG_WARN_STREAM("[DataSaver] grid_map_downsample_size<=0, fallback to "
                        << leaf << "m voxel before grid partition.");
    }

    LOG_INFO_STREAM("[DataSaver] Downsampling grid map with resolution: " << leaf
                    << "m, input cloud size: " << cloud->size());

    custom_filter::VoxelFilterOptimized<pcl::PointXYZI>(cloud, filtered, leaf);
    {
        const auto stats = custom_filter::GetLastVoxelFilterStats();
        LOG_INFO_STREAM("[DataSaver] Voxel filter (grid map) leaf=" << stats.voxel_size
                        << "m, input=" << stats.input_points
                        << ", output=" << stats.output_points
                        << ", skipped=" << stats.skipped_points);
    }

    filtered->width = filtered->size();
    filtered->height = 1;
    filtered->is_dense = false;

    LOG_INFO_STREAM("[DataSaver] Downsampled grid map output size: " << filtered->size());

    return filtered;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr DataSaver::DownSampleWithLeaf(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
    float leaf_size_m) const
{
    if (leaf_size_m <= 0.0f)
    {
        return DownSample(cloud);
    }

    if (!cloud || cloud->empty())
    {
        return pcl::PointCloud<pcl::PointXYZI>::Ptr();
    }

    auto filtered = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    const float leaf = leaf_size_m;
    LOG_INFO_STREAM("[DataSaver] Downsampling grid map (forced) with resolution: "
                    << leaf << "m, input cloud size: " << cloud->size());

    custom_filter::VoxelFilterOptimized<pcl::PointXYZI>(cloud, filtered, leaf);
    {
        const auto stats = custom_filter::GetLastVoxelFilterStats();
        LOG_INFO_STREAM("[DataSaver] Voxel filter (forced) leaf=" << stats.voxel_size
                        << "m, input=" << stats.input_points
                        << ", output=" << stats.output_points
                        << ", skipped=" << stats.skipped_points);
    }

    if (filtered->size() >= cloud->size())
    {
        LOG_WARN_STREAM("[DataSaver] Voxel filter did not reduce point count (" << cloud->size()
                        << " -> " << filtered->size() << "); applying manual voxel collapse.");

        struct VoxelKey
        {
            int64_t x, y, z;
            bool operator==(const VoxelKey &other) const
            {
                return x == other.x && y == other.y && z == other.z;
            }
        };
        struct VoxelKeyHash
        {
            std::size_t operator()(const VoxelKey &k) const noexcept
            {
                const std::size_t hx = std::hash<int64_t>{}(k.x);
                const std::size_t hy = std::hash<int64_t>{}(k.y);
                const std::size_t hz = std::hash<int64_t>{}(k.z);
                return hx ^ (hy + 0x9e3779b97f4a7c15ULL + (hx << 6) + (hx >> 2)) ^ (hz << 1);
            }
        };

        std::unordered_set<VoxelKey, VoxelKeyHash> seen;
        seen.reserve(cloud->size());
        filtered->clear();
        filtered->reserve(cloud->size());

        const double inv_leaf = 1.0 / static_cast<double>(leaf);
        for (const auto &p : cloud->points)
        {
            VoxelKey key{
                static_cast<int64_t>(std::floor(p.x * inv_leaf)),
                static_cast<int64_t>(std::floor(p.y * inv_leaf)),
                static_cast<int64_t>(std::floor(p.z * inv_leaf))};
            if (seen.insert(key).second)
            {
                filtered->push_back(p);
            }
        }

        LOG_INFO_STREAM("[DataSaver] Manual voxel collapse result: " << filtered->size()
                        << " points (" << cloud->size() << " -> " << filtered->size() << ")");
    }

    filtered->width = filtered->size();
    filtered->height = 1;
    filtered->is_dense = false;

    LOG_INFO_STREAM("[DataSaver] Downsampled grid map output size: " << filtered->size());

    return filtered;
}

std::string DataSaver::PcdmapFile(const std::string &filename) const
{
    if (pcdmap_root_path_.empty())
    {
        return save_directory + filename;
    }
    std::string path = pcdmap_root_path_;
    if (!path.empty() && path.back() != '/')
    {
        path.push_back('/');
    }
    path += filename;
    return path;
}

pcl::PointCloud<PointT>::Ptr DataSaver::FilterMapOutliers(const pcl::PointCloud<PointT>::Ptr &cloud) const
{
    if (!cloud || cloud->empty())
    {
        return cloud;
    }

    if (!mapOutlierFilterEnabled)
    {
        return cloud;
    }

    const bool sor_enabled = (mapSorMeanK > 0) && (mapSorStdMul > 0.0);
    const bool ror_enabled = (mapRorRadius > 0.0) && (mapRorMinNeighbors > 0);

    if (!sor_enabled && !ror_enabled)
    {
        return cloud;
    }

    auto apply_sor = [&](const pcl::PointCloud<PointT>::Ptr &input) {
        if (!sor_enabled || !input || input->empty())
        {
            return input;
        }
        auto filtered = boost::make_shared<pcl::PointCloud<PointT>>();
        pcl::StatisticalOutlierRemoval<PointT> sor;
        sor.setMeanK(mapSorMeanK);
        sor.setStddevMulThresh(mapSorStdMul);
        sor.setInputCloud(input);
        sor.filter(*filtered);
        return filtered;
    };

    auto apply_ror = [&](const pcl::PointCloud<PointT>::Ptr &input) {
        if (!ror_enabled || !input || input->empty())
        {
            return input;
        }
        auto filtered = boost::make_shared<pcl::PointCloud<PointT>>();
        pcl::RadiusOutlierRemoval<PointT> ror;
        ror.setRadiusSearch(mapRorRadius);
        ror.setMinNeighborsInRadius(mapRorMinNeighbors);
        ror.setInputCloud(input);
        ror.filter(*filtered);
        return filtered;
    };

    pcl::PointCloud<PointT>::Ptr current = cloud;
    if (mapOutlierFilterMode == "sor")
    {
        current = apply_sor(current);
        if (current == cloud && ror_enabled)
        {
            current = apply_ror(current);
        }
    }
    else if (mapOutlierFilterMode == "ror")
    {
        current = apply_ror(current);
        if (current == cloud && sor_enabled)
        {
            current = apply_sor(current);
        }
    }
    else
    {
        // fallback to SOR first
        current = apply_sor(current);
        if (current == cloud)
        {
            current = apply_ror(current);
        }
    }

    if (current)
    {
        current->width = current->size();
        current->height = 1;
        current->is_dense = false;
    }

    return current;
}

bool DataSaver::PrepareGridDirectory() const
{
    if (localization_map_path_.empty())
    {
        std::cerr << "[ERROR] localization_map_path_ is empty, cannot prepare grid directory." << std::endl;
        return false;
    }

    std::error_code ec;
    const fs::path grid_dir(localization_map_path_);

    if (fs::exists(grid_dir, ec))
    {
        if (!fs::is_directory(grid_dir, ec))
        {
            std::cerr << "[ERROR] grid path exists but is not a directory: "
                      << grid_dir << " error: " << ec.message() << std::endl;
            return false;
        }

        for (const auto &entry : fs::directory_iterator(grid_dir, ec))
        {
            std::error_code remove_ec;
            fs::remove_all(entry.path(), remove_ec);
            if (remove_ec)
            {
                std::cerr << "[WARNING] Failed to clean old grid file " << entry.path()
                          << ": " << remove_ec.message() << std::endl;
            }
        }
        return true;
    }

    if (!fs::create_directories(grid_dir, ec) && !fs::exists(grid_dir, ec))
    {
        std::cerr << "[ERROR] Failed to create grid directory: " << grid_dir
                  << " error: " << ec.message() << std::endl;
        return false;
    }
    return true;
}

void DataSaver::SaveLocalizationMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr &map_cloud)
{
    if (!map_cloud || map_cloud->empty())
    {
        std::cerr << "Input point cloud is empty!" << std::endl;
        return;
    }

    if (localization_map_path_.empty())
    {
        std::cerr << "[ERROR] localization_map_path_ is not set." << std::endl;
        return;
    }

    if (localization_grid_size_ <= kEpsilon)
    {
        std::cerr << "[ERROR] grid_size_ must be positive, current value: " << localization_grid_size_ << std::endl;
        return;
    }

    // Pre-scan to detect extreme extents that would overflow PCL's VoxelGrid indexing.
    Eigen::Vector4f min_pt_vec(std::numeric_limits<float>::max(),
                               std::numeric_limits<float>::max(),
                               std::numeric_limits<float>::max(),
                               0.0f);
    Eigen::Vector4f max_pt_vec(std::numeric_limits<float>::lowest(),
                               std::numeric_limits<float>::lowest(),
                               std::numeric_limits<float>::lowest(),
                               0.0f);
    std::size_t finite_count = 0;
    std::size_t invalid_count = 0;
    for (const auto &p : map_cloud->points)
    {
        if (!pcl::isFinite(p))
        {
            ++invalid_count;
            continue;
        }
        ++finite_count;
        min_pt_vec.x() = std::min(min_pt_vec.x(), p.x);
        min_pt_vec.y() = std::min(min_pt_vec.y(), p.y);
        min_pt_vec.z() = std::min(min_pt_vec.z(), p.z);
        max_pt_vec.x() = std::max(max_pt_vec.x(), p.x);
        max_pt_vec.y() = std::max(max_pt_vec.y(), p.y);
        max_pt_vec.z() = std::max(max_pt_vec.z(), p.z);
    }

    if (finite_count == 0)
    {
        LOG_ERROR_STREAM("[DataSaver] No finite points available for grid map export. Invalid points: "
                         << invalid_count);
        return;
    }

    const double range_x = static_cast<double>(max_pt_vec.x()) - min_pt_vec.x();
    const double range_y = static_cast<double>(max_pt_vec.y()) - min_pt_vec.y();
    const double range_z = static_cast<double>(max_pt_vec.z()) - min_pt_vec.z();
    const double max_range = std::max({std::abs(range_x), std::abs(range_y), std::abs(range_z)});
    constexpr float kGridVoxelLeaf = 0.2f;  // 强制与 generate_gridmap.py 一致
    float voxel_leaf = kGridVoxelLeaf;
    const double index_limit = static_cast<double>(std::numeric_limits<int>::max()) - 1.0;
    const double required_leaf = max_range / index_limit;
    if (required_leaf > static_cast<double>(voxel_leaf))
    {
        voxel_leaf = static_cast<float>(required_leaf * 1.05);
        LOG_WARN_STREAM("[DataSaver] Grid map span (" << range_x << "m, " << range_y << "m, " << range_z
                        << "m) exceeds safe range for 0.2m voxels; increasing voxel leaf to "
                        << voxel_leaf << "m to avoid VoxelGrid overflow. Invalid points skipped: "
                        << invalid_count);
    }
    else if (invalid_count > 0)
    {
        LOG_WARN_STREAM("[DataSaver] Grid map contains " << invalid_count
                        << " invalid points; extent (m): x=" << range_x
                        << ", y=" << range_y << ", z=" << range_z);
    }

    if (!PrepareGridDirectory())
    {
        return;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr down_sampled_cloud = DownSampleWithLeaf(map_cloud, voxel_leaf);
    if (!down_sampled_cloud || down_sampled_cloud->empty())
    {
        LOG_ERROR_STREAM("[DataSaver] 0.2m voxel downsample for grid map returned empty, abort grid export. Input size: "
                         << (map_cloud ? map_cloud->size() : 0));
        return;
    }
    LOG_INFO_STREAM("[DataSaver] Grid map downsample leaf=" << voxel_leaf
                    << "m, " << map_cloud->size() << " -> " << down_sampled_cloud->size() << " points");

    pcl::PointXYZI min_pt, max_pt;
    pcl::getMinMax3D(*down_sampled_cloud, min_pt, max_pt);

    min_x_ = min_pt.x;
    max_x_ = max_pt.x;
    min_y_ = min_pt.y;
    max_y_ = max_pt.y;

    const double grid_size = localization_grid_size_;
    const int min_x_b = static_cast<int>(grid_size * std::floor(min_x_ / grid_size));
    const int max_x_b = static_cast<int>(grid_size * (std::floor(max_x_ / grid_size) + 1));
    const int min_y_b = static_cast<int>(grid_size * std::floor(min_y_ / grid_size));
    const int max_y_b = static_cast<int>(grid_size * (std::floor(max_y_ / grid_size) + 1));
    const int div_x = (max_x_b - min_x_b) / static_cast<int>(grid_size);
    const int div_y = (max_y_b - min_y_b) / static_cast<int>(grid_size);

    if (div_x <= 0 || div_y <= 0)
    {
        std::cerr << "[ERROR] Invalid grid division size: div_x=" << div_x
                  << ", div_y=" << div_y << std::endl;
        return;
    }

    const int grid_num = div_x * div_y;

    std::vector<GridInfo> grids;
    grids.reserve(grid_num);

    const size_t source_size = down_sampled_cloud->size();

    for (int y = 0; y < div_y; y++)
    {
        for (int x = 0; x < div_x; x++)
        {
            const int id = div_x * y + x;
            GridInfo grid;
            grid.grid_id = id;
            grid.grid_id_x = x;
            grid.grid_id_y = y;
            grid.lower_bound_x = static_cast<float>(min_x_b + grid_size * x);
            grid.lower_bound_y = static_cast<float>(min_y_b + grid_size * y);
            grid.upper_bound_x = static_cast<float>(min_x_b + grid_size * (x + 1));
            grid.upper_bound_y = static_cast<float>(min_y_b + grid_size * (y + 1));

            std::stringstream ss;
            ss << static_cast<int>(grid_size) << "_" << grid.lower_bound_x << "_" << grid.lower_bound_y;
            grid.name = ss.str() + ".pcd";
            grid.filename = localization_map_path_ + "/" + grid.name;

            if (grid_num > 0)
            {
                grid.cloud.reserve(source_size / static_cast<size_t>(grid_num));
            }
            grids.push_back(std::move(grid));
        }
    }

    const auto &source_points = down_sampled_cloud->points;
    for (const auto &p : source_points)
    {
        const int idx = static_cast<int>(std::floor((p.x - static_cast<float>(min_x_b)) / grid_size));
        const int idy = static_cast<int>(std::floor((p.y - static_cast<float>(min_y_b)) / grid_size));
        if (idx < 0 || idx >= div_x || idy < 0 || idy >= div_y)
        {
            continue;
        }
        const int id = idy * div_x + idx;
        if (id >= 0 && id < grid_num)
        {
            grids[id].cloud.push_back(p);
        }
    }

    std::size_t allocated_points = 0;
    for (const auto &grid : grids)
    {
        allocated_points += grid.cloud.size();
    }
    LOG_INFO_STREAM("[DataSaver] Grid map point stats: source(after voxel)=" << source_size
                    << ", assigned=" << allocated_points);

    for (const auto &grid : grids)
    {
        if (!grid.cloud.empty())
        {
            if (!WritePointCloudBinary(grid.filename, grid.cloud))
            {
                std::cerr << "Error saving grid " << grid.filename << std::endl;
            }
        }
    }

    WriteCSV(grids);
}

bool DataSaver::SavePointCloudAsImage(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                                      const std::string &filename,
                                      float resolution,
                                      const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &trajectory)
{
    if (!cloud || cloud->empty())
    {
        std::cerr << "[ERROR] Point cloud is empty, skip image generation." << std::endl;
        return false;
    }

    if (resolution <= 0)
    {
        std::cerr << "[ERROR] Resolution must be positive, current value: " << resolution << std::endl;
        return false;
    }

    if (cloud->size() < 10)
    {
        std::cerr << "[WARNING] Only " << cloud->size() << " points in cloud; intensity image may be unreliable." << std::endl;
    }

    float min_intensity = std::numeric_limits<float>::max();
    float max_intensity = std::numeric_limits<float>::lowest();
    int intensity_points = 0;
    for (const auto &point : cloud->points)
    {
        if (!std::isfinite(point.intensity))
        {
            continue;
        }
        min_intensity = std::min(min_intensity, point.intensity);
        max_intensity = std::max(max_intensity, point.intensity);
        intensity_points++;
    }

    if (intensity_points == 0)
    {
        std::cerr << "[ERROR] No finite intensity values found; abort image export." << std::endl;
        return false;
    }

    if (std::abs(max_intensity - min_intensity) < kEpsilon)
    {
        std::cout << "[WARNING] Intensity range is zero; use constant fill value." << std::endl;
        max_intensity = min_intensity + 1.0f;
    }
    const float intensity_range = max_intensity - min_intensity;

    const fs::path image_dir = fs::path(localization_map_path_).parent_path();
    std::error_code dir_ec;
    if (!image_dir.empty() && !fs::exists(image_dir, dir_ec))
    {
        if (!fs::create_directories(image_dir, dir_ec))
        {
            std::cerr << "[ERROR] Failed to create intensity image directory " << image_dir
                      << ": " << dir_ec.message() << std::endl;
            return false;
        }
    }

    if (!image_dir.empty() && !fs::is_directory(image_dir, dir_ec))
    {
        std::cerr << "[ERROR] Intensity image output path is not a directory: " << image_dir
                  << ": " << dir_ec.message() << std::endl;
        return false;
    }

    enum class Axis
    {
        X,
        Y,
        Z
    };

    auto coordinate_of = [](const pcl::PointXYZI &point, Axis axis) -> float {
        switch (axis)
        {
        case Axis::X:
            return point.x;
        case Axis::Y:
            return point.y;
        case Axis::Z:
            return point.z;
        default:
            return std::numeric_limits<float>::quiet_NaN();
        }
    };

    auto coordinate_of_traj = [](const pcl::PointXYZ &point, Axis axis) -> float {
        switch (axis)
        {
        case Axis::X:
            return point.x;
        case Axis::Y:
            return point.y;
        case Axis::Z:
            return point.z;
        default:
            return std::numeric_limits<float>::quiet_NaN();
        }
    };

    const float base_resolution = resolution;
    const int MAX_SIZE = 10000;
    const int MIN_SIZE = 10;

    auto clamp_index = [](int value, int lower, int upper) {
        if (value < lower)
        {
            return lower;
        }
        if (value > upper)
        {
            return upper;
        }
        return value;
    };

    auto clamp_unit = [](float value) {
        if (value < 0.0f)
        {
            return 0.0f;
        }
        if (value > 1.0f)
        {
            return 1.0f;
        }
        return value;
    };

    auto generate_projection = [&](Axis horizontal,
                                   Axis vertical,
                                   const std::string &suffix,
                                   const std::string &horizontal_label,
                                   const std::string &vertical_label,
                                   const std::string &orientation_tag) -> bool {
        float min_h = std::numeric_limits<float>::max();
        float max_h = std::numeric_limits<float>::lowest();
        float min_v = std::numeric_limits<float>::max();
        float max_v = std::numeric_limits<float>::lowest();
        int valid_points = 0;

        for (const auto &point : cloud->points)
        {
            const float h = coordinate_of(point, horizontal);
            const float v = coordinate_of(point, vertical);
            if (!std::isfinite(h) || !std::isfinite(v) || !std::isfinite(point.intensity))
            {
                continue;
            }
            min_h = std::min(min_h, h);
            max_h = std::max(max_h, h);
            min_v = std::min(min_v, v);
            max_v = std::max(max_v, v);
            valid_points++;
        }

        if (valid_points == 0)
        {
            std::cerr << "[ERROR] No finite " << orientation_tag
                      << " data found; skip intensity image export." << std::endl;
            return false;
        }

        const float h_range = max_h - min_h;
        const float v_range = max_v - min_v;
        if (h_range < kEpsilon || v_range < kEpsilon)
        {
            std::cerr << "[ERROR] Point cloud span too small for " << orientation_tag << " projection. Range: "
                      << horizontal_label << "=" << h_range << " m, "
                      << vertical_label << "=" << v_range << " m" << std::endl;
            return false;
        }

        float local_resolution = base_resolution;
        int width = static_cast<int>(std::ceil(h_range / local_resolution)) + 1;
        int height = static_cast<int>(std::ceil(v_range / local_resolution)) + 1;

        std::cout << "[INFO] Initial intensity image size (" << orientation_tag << "): "
                  << width << " x " << height << " pixels" << std::endl;

        if (width > MAX_SIZE || height > MAX_SIZE)
        {
            float scale = std::max(static_cast<float>(width) / MAX_SIZE,
                                   static_cast<float>(height) / MAX_SIZE);
            local_resolution *= scale;
            width = static_cast<int>(std::ceil(h_range / local_resolution)) + 1;
            height = static_cast<int>(std::ceil(v_range / local_resolution)) + 1;

            std::cout << "[WARNING] Image too large (" << orientation_tag << "); scale resolution to "
                      << local_resolution << " m/pixel" << std::endl;
            std::cout << "[INFO] Resized image dimensions (" << orientation_tag << "): "
                      << width << " x " << height << " pixels" << std::endl;
        }

        if (width < MIN_SIZE || height < MIN_SIZE)
        {
            std::cerr << "[ERROR] Resulting " << orientation_tag << " image too small (" << width << " x " << height
                      << "); verify point cloud coverage or adjust resolution." << std::endl;
            return false;
        }

        cv::Mat intensity_sum = cv::Mat::zeros(height, width, CV_32F);
        cv::Mat count_mat = cv::Mat::zeros(height, width, CV_32F);

        int projected_points = 0;
        for (const auto &point : cloud->points)
        {
            const float h = coordinate_of(point, horizontal);
            const float v = coordinate_of(point, vertical);
            if (!std::isfinite(h) || !std::isfinite(v) || !std::isfinite(point.intensity))
            {
                continue;
            }

            int px = static_cast<int>(std::floor((h - min_h) / local_resolution));
            int py = static_cast<int>(std::floor((max_v - v) / local_resolution));

            px = clamp_index(px, 0, std::max(0, width - 1));
            py = clamp_index(py, 0, std::max(0, height - 1));

            float normalized_intensity = 0.5f;
            if (intensity_range > kEpsilon)
            {
                normalized_intensity = (point.intensity - min_intensity) / intensity_range;
                normalized_intensity = clamp_unit(normalized_intensity);
            }

            intensity_sum.at<float>(py, px) += normalized_intensity;
            count_mat.at<float>(py, px) += 1.0f;
            projected_points++;
        }

        cv::Mat map_image = cv::Mat::zeros(height, width, CV_8UC1);
        int non_empty_pixels = 0;
        for (int y = 0; y < height; ++y)
        {
            for (int x = 0; x < width; ++x)
            {
                float count = count_mat.at<float>(y, x);
                if (count > 0.0f)
                {
                    float avg_intensity = intensity_sum.at<float>(y, x) / count;
                    map_image.at<uint8_t>(y, x) = static_cast<uint8_t>(
                        std::min(255.0f, avg_intensity * 255.0f));
                    non_empty_pixels++;
                }
            }
        }

        if (non_empty_pixels < 10)
        {
            std::cerr << "[ERROR] " << orientation_tag << " intensity image contains <10 valid pixels; skipping export." << std::endl;
            return false;
        }

        cv::Mat smooth_image;
        if (non_empty_pixels > 10)
        {
            cv::GaussianBlur(map_image, smooth_image, cv::Size(3, 3), 1.0);
        }
        else
        {
            smooth_image = map_image.clone();
        }

        cv::Mat enhanced_image;
        if (non_empty_pixels > (width * height * 0.01))
        {
            cv::equalizeHist(smooth_image, enhanced_image);
        }
        else
        {
            double min_val = 0.0;
            double max_val = 0.0;
            cv::minMaxLoc(smooth_image, &min_val, &max_val);
            if (max_val > min_val)
            {
                smooth_image.convertTo(enhanced_image, CV_8UC1,
                                       255.0 / (max_val - min_val),
                                       -255.0 * min_val / (max_val - min_val));
            }
            else
            {
                enhanced_image = smooth_image.clone();
            }
        }

        auto overlay_trajectory = [&](cv::Mat &image) {
            if (!trajectory || trajectory->empty())
            {
                return;
            }
            const int thickness = 2;
            cv::Point prev_point(-1, -1);
            bool has_prev = false;
            for (const auto &pt : trajectory->points)
            {
                const float h = coordinate_of_traj(pt, horizontal);
                const float v = coordinate_of_traj(pt, vertical);
                if (!std::isfinite(h) || !std::isfinite(v))
                {
                    has_prev = false;
                    continue;
                }
                int px = static_cast<int>(std::floor((h - min_h) / local_resolution));
                int py = static_cast<int>(std::floor((max_v - v) / local_resolution));

                px = clamp_index(px, 0, std::max(0, width - 1));
                py = clamp_index(py, 0, std::max(0, height - 1));

                cv::Point curr_point(px, py);
                if (has_prev)
                {
                    cv::line(image, prev_point, curr_point, cv::Scalar(255), thickness, cv::LINE_AA);
                }
                prev_point = curr_point;
                has_prev = true;
            }
        };

        overlay_trajectory(enhanced_image);

        const fs::path image_path = image_dir / (filename + suffix + ".png");
        if (!cv::imwrite(image_path.string(), enhanced_image))
        {
            std::cerr << "[ERROR] Failed to write intensity image (" << orientation_tag << "): "
                      << image_path << std::endl;
            return false;
        }

        const fs::path meta_path = image_dir / (filename + suffix + "_metadata.txt");
        std::ofstream meta_file(meta_path.string());
        if (!meta_file.is_open())
        {
            std::cerr << "[ERROR] Unable to create metadata file (" << orientation_tag << "): "
                      << meta_path << std::endl;
            return false;
        }

        meta_file << "# 2D Map Metadata" << std::endl;
        meta_file << "orientation: " << orientation_tag << std::endl;
        meta_file << "resolution: " << local_resolution << std::endl;
        meta_file << "width: " << width << std::endl;
        meta_file << "height: " << height << std::endl;
        meta_file << "min_" << horizontal_label << ": " << min_h << std::endl;
        meta_file << "max_" << horizontal_label << ": " << max_h << std::endl;
        meta_file << "min_" << vertical_label << ": " << min_v << std::endl;
        meta_file << "max_" << vertical_label << ": " << max_v << std::endl;
        meta_file << "min_intensity: " << min_intensity << std::endl;
        meta_file << "max_intensity: " << max_intensity << std::endl;
        meta_file << "valid_points: " << valid_points << std::endl;
        meta_file << "projected_points: " << projected_points << std::endl;
        meta_file << "non_empty_pixels: " << non_empty_pixels << std::endl;
        meta_file << "trajectory_overlay: " << (trajectory && !trajectory->empty() ? "true" : "false") << std::endl;
        meta_file.close();

        std::cout << "[SUCCESS] Intensity image saved (" << orientation_tag << "): " << image_path << std::endl;
        std::cout << "[INFO] Final image size (" << orientation_tag << "): "
                  << width << " x " << height << " pixels" << std::endl;
        std::cout << "[INFO] Final resolution (" << orientation_tag << "): "
                  << local_resolution << " m/pixel" << std::endl;
        std::cout << "[INFO] Coverage (" << orientation_tag << "): "
                  << horizontal_label << "[" << min_h << ", " << max_h << "], "
                  << vertical_label << "[" << min_v << ", " << max_v << "]" << std::endl;

        return true;
    };

    bool xy_success = generate_projection(Axis::X, Axis::Y, "", "x", "y", "XY");
    bool xz_success = generate_projection(Axis::X, Axis::Z, "_xz", "x", "z", "XZ");
    bool yz_success = generate_projection(Axis::Y, Axis::Z, "_yz", "y", "z", "YZ");

    return xy_success && xz_success && yz_success;
}

bool DataSaver::WriteCSV(const std::vector<GridInfo> &grids)
{
    if (localization_map_path_.empty())
    {
        std::cerr << "[ERROR] localization_map_path_ is not set, cannot write CSV." << std::endl;
        return false;
    }

    std::string whole_file_name = localization_map_path_ + "/pcd_info.csv";
    std::ofstream ofs(whole_file_name.c_str());
    if (!ofs.is_open())
    {
        std::cerr << "[ERROR] Failed to open grid CSV file: " << whole_file_name << std::endl;
        return false;
    }

    const int grid_num = static_cast<int>(grids.size());
    for (int i = 0; i < grid_num; i++)
    {
        if (!grids[i].cloud.points.empty())
        {
            ofs << grids[i].name << ","
                << grids[i].lower_bound_x << ","
                << grids[i].lower_bound_y << ","
                << 0.0 << ","
                << grids[i].upper_bound_x << ","
                << grids[i].upper_bound_y << ","
                << 0.0 << std::endl;
        }
    }
    return true;
}

void DataSaver::saveOptimizedVerticesTUM(gtsam::Values _estimates)
{
    std::fstream stream(save_directory + "optimized_odom_tum.txt",
                        std::fstream::out);
    stream.precision(15);
    for (int i = 0; i < _estimates.size(); i++)
    {
        auto &pose = _estimates.at(X(i)).cast<gtsam::Pose3>();
        //        auto &pose = _estimates.at<gtsam::Pose3>(X(i));
        gtsam::Point3 p = pose.translation();
        gtsam::Quaternion q = pose.rotation().toQuaternion();
        stream << keyframeTimes.at(i) << " " << p.x() << " " << p.y() << " "
               << p.z() << " " << q.x() << " " << q.y() << " " << q.z() << " "
               << q.w() << std::endl;
    }
}

void DataSaver::saveOptimizedVerticesTUM(std::vector<Vector7> _estimates)
{
    std::fstream stream(save_directory + "optimized_odom_tum.txt",
                        std::fstream::out);
    stream.precision(15);
    // x y z qx qy qz qw
    for (int i = 0; i < _estimates.size(); i++)
    {
        auto &pose = _estimates.at(i);
        stream << keyframeTimes.at(i) << " " << pose(0) << " " << pose(1) << " "
               << pose(2) << " " << pose(3) << " " << pose(4) << " " << pose(5)
               << " " << pose(6) << std::endl;
    }
}

void DataSaver::saveOptimizedVerticesTUM(std::vector<Vector7> _estimates,
                                         std::string file_name)
{
    // Determine the correct directory based on filename
    std::string output_path;
    if (file_name == "optimized_poses_tum.txt" || file_name == "optimized_odom_tum.txt" ||
        file_name == "pose_graph_3d_result.txt" || file_name == "data_time.txt")
    {
        output_path = logs_path_ + "/" + file_name;
    }
    else
    {
        output_path = save_directory + file_name;
    }

    std::fstream stream(output_path, std::fstream::out);
    stream.precision(15);
    // x y z qx qy qz qw
    for (int i = 0; i < _estimates.size(); i++)
    {
        auto &pose = _estimates.at(i);
        stream << keyframeTimes.at(i) << " " << pose(0) << " " << pose(1) << " "
               << pose(2) << " " << pose(3) << " " << pose(4) << " " << pose(5)
               << " " << pose(6) << std::endl;
    }

    stream.close();
    const std::string traj_filename = saveResultBodyFrame ? "trajectory_imu.pcd" : "trajectory_lidar.pcd";
    saveTrajectoryPointCloud(_estimates, PcdmapFile(traj_filename));

    if (file_name == "optimized_poses_tum.txt")
    {
        ExportImuPosesCsvToMapping(std::filesystem::path(logs_path_) / file_name);
    }
}

void DataSaver::saveOdomCov(std::vector<Eigen::Matrix<double, 6, 6>> &cov_vec,
                            std::string file_name)
{

    std::fstream pcovfile(save_directory + file_name, std::fstream::out);
    pcovfile.precision(15);
    for (int i = 0; i < keyframeTimes.size(); i++)
    {
        pcovfile << keyframeTimes.at(i);
        Eigen::Matrix<double, 6, 6> poseCov = cov_vec.at(i);
        // write upper triangular part of the covariance matrix
        // in the order of translation followed by rotation
        for (int row = 0; row < 6; row++)
        {
            for (int col = row; col < 6; col++)
            {
                // swap the order of translation and rotation
                int swapped_row = (row < 3) ? (row + 3) : (row - 3);
                int swapped_col = (col < 3) ? (col + 3) : (col - 3);
                pcovfile << " " << poseCov(swapped_row, swapped_col);
            }
        }
        pcovfile << std::endl;
    }
    pcovfile.close();
}

void DataSaver::setDir(string _base_dir, string _sequence_name)
{
    this->base_dir = _base_dir;
    this->sequence_name = _sequence_name;

    if (_base_dir.back() != '/')
    {
        _base_dir.append("/");
    }
    save_directory = _base_dir + sequence_name + '/';

    auto unused = system((std::string("exec rm -r ") + save_directory).c_str());
    unused = system((std::string("mkdir -p ") + save_directory).c_str());

    //  LOG(INFO) << "SET DIR:" << save_directory;

    InitializeLocalizationOutputs();
}

void DataSaver::ResetLocalizationOutputs()
{
    InitializeLocalizationOutputs();
}

void DataSaver::UpdateVersionManifest()
{
    if (pcdmap_root_path_.empty())
    {
        LOG_WARN_STREAM("[DataSaver] UpdateVersionManifest skipped: pcdmap path is empty.");
        return;
    }

    namespace fs = std::filesystem;
    std::error_code ec;
    fs::create_directories(pcdmap_root_path_, ec);
    if (ec)
    {
        LOG_WARN_STREAM("[DataSaver] Failed to ensure pcdmap directory before updating version.json: "
                        << pcdmap_root_path_ << ", " << ec.message());
        return;
    }

    try
    {
        fs::path version_json_path = fs::path(pcdmap_root_path_) / "version.json";
        GenerateVersionJson(version_json_path.string());
    }
    catch (const std::exception &e)
    {
        LOG_ERROR_STREAM("[DataSaver] Failed to refresh version.json: " << e.what());
    }
}

void DataSaver::setConfigDir(string _config_dir)
{
    if (_config_dir.back() != '/')
    {
        _config_dir.append("/");
    }
    this->config_directory = _config_dir;
}

void DataSaver::setMapDir(string _config_dir)
{
    if (_config_dir.back() != '/')
    {
        _config_dir.append("/");
    }
    this->map_directory = _config_dir;
    legacy_trajectory_.clear();
    legacy_mode_enabled_ = false;
}

bool DataSaver::ShouldSaveKeyframePCD() const
{
    return save_key_frame && (keyframe_save_mode_ == 0 || keyframe_save_mode_ == 2);
}

bool DataSaver::ShouldSaveKeyframeBin() const
{
    return save_key_frame && (keyframe_save_mode_ == 1 || keyframe_save_mode_ == 2);
}

void DataSaver::setKeyframe(bool _save_key_frame, int keyframe_mode)
{
    this->save_key_frame = _save_key_frame;
    keyframe_save_mode_ = std::max(0, std::min(keyframe_mode, 2));

    keyFrmaePath.clear();

    if (!save_key_frame)
    {
        LOG_INFO_STREAM("[DataSaver] Keyframe saving disabled.");
        return;
    }

    namespace fs = std::filesystem;
    if (ShouldSaveKeyframePCD())
    {
        keyFrmaePath = save_directory + "key_point_frame/";

        std::error_code ec;
        fs::create_directories(fs::path(keyFrmaePath), ec);
        if (ec)
        {
            LOG_WARN_STREAM("[DataSaver] Failed to create keyframe directory " << keyFrmaePath << ": " << ec.message());
        }
        else
        {
            LOG_INFO_STREAM("[DataSaver] Keyframe PCD directory prepared at " << keyFrmaePath);
        }

    }
    else
    {
        LOG_INFO_STREAM("[DataSaver] Keyframe PCD export disabled by configuration (mode=" << keyframe_save_mode_ << ").");
    }

    if (ShouldSaveKeyframeBin())
    {
        LOG_INFO_STREAM("[DataSaver] Keyframe BIN export enabled (mode=" << keyframe_save_mode_ << ").");
    }
    else
    {
        LOG_INFO_STREAM("[DataSaver] Keyframe BIN export disabled by configuration (mode=" << keyframe_save_mode_ << ").");
    }
}

void DataSaver::ImportExistingKeyframes(const std::string &source_dir)
{
    if (!save_key_frame || !ShouldSaveKeyframePCD())
    {
        return;
    }

    namespace fs = std::filesystem;
    fs::path src_dir(source_dir);
    if (!fs::exists(src_dir) || !fs::is_directory(src_dir))
    {
        LOG_WARN_STREAM("[DataSaver] Source keyframe directory unavailable: " << source_dir);
        return;
    }

    if (keyFrmaePath.empty())
    {
        LOG_WARN_STREAM("[DataSaver] Destination keyframe path is empty, skip importing.");
        return;
    }

    fs::path dst_dir(keyFrmaePath);
    std::error_code ec;
    fs::create_directories(dst_dir, ec);
    if (ec)
    {
        LOG_WARN_STREAM("[DataSaver] Failed to create destination directory " << dst_dir << ": " << ec.message());
        return;
    }

    std::size_t copied_files = 0;
    for (const auto &entry : fs::directory_iterator(src_dir))
    {
        if (!entry.is_regular_file())
        {
            continue;
        }

        const fs::path target_path = dst_dir / entry.path().filename();
        std::error_code copy_ec;
        fs::copy_file(entry.path(), target_path, fs::copy_options::overwrite_existing, copy_ec);
        if (copy_ec)
        {
            LOG_WARN_STREAM_THROTTLE(5.0, "[DataSaver] Failed to copy keyframe file "
                                               << entry.path() << ": " << copy_ec.message());
            continue;
        }
        ++copied_files;
    }

    LOG_INFO_STREAM("[DataSaver] Imported " << copied_files
                    << " keyframe files from " << source_dir
                    << " to " << keyFrmaePath);
}

bool DataSaver::InitializeKeyframeBinWriter()
{
    keyframe_bin_active_ = false;
    keyframe_bin_frame_count_ = 0;
    keyframe_bin_point_count_ = 0;
    keyframe_bin_output_path_.clear();
    keyframe_bin_stream_.close();

    if (!ShouldSaveKeyframeBin())
    {
        return false;
    }

    namespace fs = std::filesystem;
    fs::path mapping_root = fs::path(save_directory) / "mapping";
    fs::path keyframes_root = mapping_root / "keyframes";
    std::error_code ec;
    fs::create_directories(keyframes_root, ec);
    if (ec)
    {
        LOG_WARN_STREAM("[DataSaver] Failed to create mapping keyframe directory "
                        << keyframes_root.generic_string() << ": " << ec.message());
        return false;
    }

    keyframe_bin_output_path_ = keyframes_root / "keyframe_top_front.bin";
    keyframe_bin_stream_.open(keyframe_bin_output_path_, std::ios::binary | std::ios::trunc);
    if (!keyframe_bin_stream_.is_open())
    {
        LOG_WARN_STREAM("[DataSaver] Unable to open keyframe bin for writing: "
                        << keyframe_bin_output_path_.generic_string());
        return false;
    }

    keyframe_bin_active_ = true;
    LOG_INFO_STREAM("[DataSaver] Keyframe bin writer prepared at "
                    << keyframe_bin_output_path_.generic_string());
    return true;
}

bool DataSaver::AppendKeyframeBin(const pcl::PointCloud<PointT> &cloud,
                                  double timestamp,
                                  const Eigen::Matrix4d &pose)
{
    if (!keyframe_bin_active_ || !keyframe_bin_stream_.is_open())
    {
        return false;
    }
    if (cloud.empty())
    {
        ++keyframe_bin_frame_count_;
        return true;
    }

    FrameWithOutRT frame;
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> cloud_ptr =
        std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    cloud_ptr->reserve(cloud.size());
    for (const auto &pt : cloud.points)
    {
        pcl::PointXYZI p;
        p.x = pt.x;
        p.y = pt.y;
        p.z = pt.z;
        p.intensity = pt.intensity;
        cloud_ptr->push_back(p);
    }

    CompressModule compressor;
    Eigen::Matrix4d pose_copy = pose;
    compressor.EncodeWithOutRT(frame, cloud_ptr, timestamp, pose_copy);

    if (!CompressModule::SaveToBin(frame, keyframe_bin_stream_))
    {
        LOG_WARN_STREAM("[DataSaver] Failed while writing keyframe bin frame: "
                        << keyframe_bin_output_path_.generic_string());
        keyframe_bin_stream_.close();
        keyframe_bin_active_ = false;
        return false;
    }

    keyframe_bin_point_count_ += static_cast<uint64_t>(cloud.size());
    ++keyframe_bin_frame_count_;
    return true;
}

void DataSaver::FinalizeKeyframeBinWriter()
{
    if (!keyframe_bin_active_)
    {
        keyframe_bin_stream_.close();
        return;
    }

    if (keyframe_bin_stream_.is_open())
    {
        keyframe_bin_stream_.close();
    }
    LOG_INFO_STREAM("[DataSaver] Keyframe bin saved to "
                    << keyframe_bin_output_path_.generic_string()
                    << " (" << keyframe_bin_frame_count_ << " frames, "
                    << keyframe_bin_point_count_ << " points)");
    keyframe_bin_active_ = false;
}

bool DataSaver::WriteImuPosesCsv(const std::filesystem::path &tum_source,
                                 const std::filesystem::path &csv_destination)
{
    namespace fs = std::filesystem;
    if (tum_source.empty())
    {
        return false;
    }

    std::ifstream stream(tum_source.string());
    if (!stream.is_open())
    {
        LOG_WARN_STREAM("[DataSaver] Failed to open trajectory for imu_poses.csv: "
                        << tum_source.generic_string());
        return false;
    }

    std::error_code ec;
    fs::create_directories(csv_destination.parent_path(), ec);
    if (ec)
    {
        LOG_WARN_STREAM("[DataSaver] Unable to create directory for imu_poses.csv: "
                        << csv_destination.parent_path().generic_string() << ", " << ec.message());
        return false;
    }

    std::ofstream output(csv_destination.string(), std::ios::trunc);
    if (!output.is_open())
    {
        LOG_WARN_STREAM("[DataSaver] Failed to open imu_poses.csv for writing: "
                        << csv_destination.generic_string());
        return false;
    }

    output << "timestamp,x,y,z,qw,qx,qy,qz,roll,pitch,yaw\n";

    std::string line;
    bool wrote_any = false;
    while (std::getline(stream, line))
    {
        if (line.empty())
        {
            continue;
        }

        std::istringstream line_stream(line);
        double timestamp = 0.0;
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double qx = 0.0;
        double qy = 0.0;
        double qz = 0.0;
        double qw = 1.0;
        if (!(line_stream >> timestamp >> x >> y >> z >> qx >> qy >> qz >> qw))
        {
            continue;
        }

        Eigen::Quaterniond quat(qw, qx, qy, qz);
        if (quat.norm() < 1e-12)
        {
            continue;
        }
        quat.normalize();

        Eigen::Matrix3d rotation = quat.toRotationMatrix();
        double roll = std::atan2(rotation(2, 1), rotation(2, 2));
        double pitch = std::asin(std::clamp(-rotation(2, 0), -1.0, 1.0));
        double yaw = std::atan2(rotation(1, 0), rotation(0, 0));

        output << std::setprecision(15) << timestamp << ','
               << x << ','
               << y << ','
               << z << ','
               << std::setprecision(12) << quat.w() << ','
               << quat.x() << ','
               << quat.y() << ','
               << quat.z() << ','
               << std::setprecision(9) << roll << ','
               << pitch << ','
               << yaw << '\n';
        wrote_any = true;
    }

    if (!wrote_any)
    {
        LOG_WARN_STREAM("[DataSaver] imu_poses.csv is empty after processing "
                        << tum_source.generic_string());
    }
    return wrote_any;
}

void DataSaver::ExportImuPosesCsvToMapping(const std::filesystem::path &tum_source)
{
    namespace fs = std::filesystem;
    if (tum_source.empty())
    {
        return;
    }

    fs::path base(save_directory);
    fs::path mapping_dir = base / "mapping";
    std::error_code ec;
    fs::create_directories(mapping_dir, ec);
    if (ec)
    {
        LOG_WARN_STREAM("[DataSaver] Failed to create mapping directory for imu_poses.csv: "
                        << mapping_dir.generic_string() << ", " << ec.message());
        return;
    }

    fs::path destination = mapping_dir / "imu_poses.csv";
    if (WriteImuPosesCsv(tum_source, destination))
    {
        LOG_INFO_STREAM("[DataSaver] imu_poses.csv exported to " << destination.generic_string());
    }
}

void DataSaver::setExtrinc(bool _saveResultBodyFrame,
                           Eigen::Vector3d _t_body_sensor,
                           Eigen::Quaterniond _q_body_sensor)
{
    this->saveResultBodyFrame = _saveResultBodyFrame;
    this->t_body_sensor = _t_body_sensor;
    this->q_body_sensor = _q_body_sensor;
    UpdateTransformCache();
}

void DataSaver::saveOptimizedVerticesKITTI(gtsam::Values _estimates)
{
    std::fstream stream(save_directory + "optimized_odom_kitti.txt",
                        std::fstream::out);
    stream.precision(15);
    //    for (const auto &key_value: _estimates) {
    //        auto p = dynamic_cast<const GenericValue<Pose3>
    //        *>(&key_value.value); if (!p) continue;
    //
    //        const Pose3 &pose = p->value();
    //
    //        Point3 t = pose.translation();
    //        Rot3 R = pose.rotation();
    //        auto col1 = R.column(1); // Point3
    //        auto col2 = R.column(2); // Point3
    //        auto col3 = R.column(3); // Point3
    //
    //        stream << col1.x() << " " << col2.x() << " " << col3.x() << " " <<
    //        t.x() << " "
    //               << col1.y() << " " << col2.y() << " " << col3.y() << " " <<
    //               t.y() << " "
    //               << col1.z() << " " << col2.z() << " " << col3.z() << " " <<
    //               t.z() << std::endl;
    //    }

    for (int i = 0; i < _estimates.size(); i++)
    {
        auto &pose = _estimates.at(X(i)).cast<gtsam::Pose3>();
        //        gtsam::Point3 p = pose.translation();
        //        gtsam::Quaternion q = pose.rotation().toQuaternion();
        //        stream << keyframeTimes.at(i) << " " << p.x() << " " << p.y()
        //               << " " << p.z() << " "
        //               << q.x() << " " << q.y() << " "
        //               << q.z() << " " << q.w() << std::endl;

        Point3 t = pose.translation();
        Rot3 R = pose.rotation();
        auto col1 = R.column(1);
        auto col2 = R.column(2);
        auto col3 = R.column(3);

        stream << col1.x() << " " << col2.x() << " " << col3.x() << " " << t.x()
               << " " << col1.y() << " " << col2.y() << " " << col3.y() << " "
               << t.y() << " " << col1.z() << " " << col2.z() << " " << col3.z()
               << " " << t.z() << std::endl;
    }
}

void DataSaver::saveOptimizedVerticesKITTI(std::vector<Vector7> _estimates)
{
    std::fstream stream(save_directory + "optimized_odom_kitti.txt",
                        std::fstream::out);
    stream.precision(15);
    for (int i = 0; i < _estimates.size(); i++)
    {
        auto &pose = _estimates.at(i);

        Rot3 R(pose[6], pose[3], pose[4], pose[5]);
        auto col1 = R.column(1);
        auto col2 = R.column(2);
        auto col3 = R.column(3);

        stream << col1.x() << " " << col2.x() << " " << col3.x() << " " << pose[0]
               << " " << col1.y() << " " << col2.y() << " " << col3.y() << " "
               << pose[1] << " " << col1.z() << " " << col2.z() << " " << col3.z()
               << " " << pose[2] << std::endl;
    }
}

void DataSaver::saveOdometryVerticesKITTI(std::string _filename)
{
    //  std::fstream stream(_filename.c_str(), std::fstream::out);
    //  stream.precision(15);
    //  for (const auto &_pose6d: keyframePoses) {
    //    gtsam::Pose3 pose = Pose6DtoGTSAMPose3(_pose6d);
    //    Point3 t = pose.translation();
    //    Rot3 R = pose.rotation();
    //    auto col1 = R.column(1); // Point3
    //    auto col2 = R.column(2); // Point3
    //    auto col3 = R.column(3); // Point3
    //
    //    stream << col1.x() << " " << col2.x() << " " << col3.x() << " " << t.x()
    //    << " "
    //           << col1.y() << " " << col2.y() << " " << col3.y() << " " << t.y()
    //           << " "
    //           << col1.z() << " " << col2.z() << " " << col3.z() << " " << t.z()
    //           << std::endl;
    //  }
}

void DataSaver::saveOriginGPS(Eigen::Vector3d gps_point)
{
    std::fstream originStream(save_directory + "origin.txt", std::fstream::out);
    originStream.precision(15);
    originStream << gps_point[0] << " " << gps_point[1] << " " << gps_point[2]
                 << std::endl;
    originStream.close();

    saveOriginPositionCfg(gps_point, sequence_name);
}

void DataSaver::saveOriginPositionCfg(const Eigen::Vector3d &gps_point,
                                      const std::string &address,
                                      const std::string &filename)
{
    const std::string file_path = PcdmapFile(filename);
    std::ofstream origin_file(file_path, std::ios::out | std::ios::trunc);
    if (!origin_file.is_open())
    {
        std::cerr << "Failed to open origin cfg file: " << file_path << std::endl;
        return;
    }

    const double lat = gps_point[0];
    const double lon = gps_point[1];
    const double height = gps_point[2];

    std::time_t now = std::time(nullptr);
    char time_buffer[128] = {0};
    if (std::strftime(time_buffer, sizeof(time_buffer), "%a %b %e %H:%M:%S %Y", std::localtime(&now)) == 0)
    {
        std::snprintf(time_buffer, sizeof(time_buffer), "unknown time");
    }

    // 从ROS参数获取项目元数据以构建更完整的address
    std::string project_name = "";
    std::string area_name = "";
    ros::param::get("/ms_mapping/project_name", project_name);
    ros::param::get("/ms_mapping/area_name", area_name);
    
    std::string final_address = address;
    if (!project_name.empty() && !area_name.empty())
    {
        final_address = project_name + "_" + area_name;
    }
    else if (!address.empty())
    {
        final_address = address;
    }

    origin_file << "{\n";
    origin_file << "   \"LLH\" : {\n";
    origin_file << std::fixed;
    origin_file << "      \"Height\" : " << std::setprecision(15) << height << ",\n";
    origin_file << "      \"Lat\" : " << std::setprecision(15) << lat << ",\n";
    origin_file << "      \"Lon\" : " << std::setprecision(15) << lon << "\n";
    origin_file << "   },\n";
    origin_file << "   \"address\" : \"" << final_address << "\",\n";
    origin_file << "   \"createTime\" : \"" << time_buffer << "\\n\"\n";
    origin_file << "}\n";

    origin_file.close();
    std::cout << "Saved origin cfg: " << file_path << std::endl;

}

void DataSaver::saveTrajectoryKML(const std::vector<Eigen::Vector3d> &enu_positions,
                                  const Eigen::Vector3d &origin_lla,
                                  const std::string &filename,
                                  const std::string &color_hex,
                                  const std::string &output_directory_override)
{
    if (enu_positions.empty())
    {
        std::cout << "Trajectory empty; skip KML export." << std::endl;
        return;
    }

    const std::string base_dir = output_directory_override.empty() ? logs_path_ : output_directory_override;
    const std::string file_path = base_dir + "/" + filename;
    std::error_code dir_ec;
    std::filesystem::create_directories(std::filesystem::path(base_dir), dir_ec);
    std::ofstream kml_file(file_path, std::ios::out | std::ios::trunc);
    if (!kml_file.is_open())
    {
        std::cerr << "Failed to open trajectory KML file: " << file_path << std::endl;
        return;
    }

    GeographicLib::LocalCartesian geo_converter(origin_lla[0], origin_lla[1], origin_lla[2]);

    kml_file << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    kml_file << "<kml xmlns=\"http://www.opengis.net/kml/2.2\" xmlns:gx=\"http://www.google.com/kml/ext/2.2\">\n";
    kml_file << "  <Document>\n";
    kml_file << "    <name>" << sequence_name << " Trajectory</name>\n";
    kml_file << "    <Style id=\"trajectoryStyle\">\n";
    kml_file << "      <LineStyle><color>" << color_hex << "</color><width>4</width></LineStyle>\n";
    kml_file << "    </Style>\n";

    kml_file << "    <Placemark>\n";
    kml_file << "      <name>Trajectory</name>\n";
    kml_file << "      <styleUrl>#trajectoryStyle</styleUrl>\n";
    kml_file << "      <LineString>\n";
    kml_file << "        <tessellate>1</tessellate>\n";
    // 使用绝对高度，避免与地形重复叠加导致“抬高”现象
    kml_file << "        <altitudeMode>absolute</altitudeMode>\n";
    kml_file << "        <coordinates>\n";

    kml_file << std::fixed;
    const double altitude_offset = 0.0; // 不做额外抬升，保持与GNSS一致

    static std::once_flag geoid_once;
    static std::unique_ptr<GeographicLib::Geoid> geoid_ptr;
    static bool geoid_available = false;
    static std::string geoid_error;

    std::call_once(geoid_once, []() {
        try
        {
            geoid_ptr = std::make_unique<GeographicLib::Geoid>("egm96-5");
            geoid_available = true;
        }
        catch (const std::exception &e)
        {
            geoid_error = e.what();
            geoid_available = false;
        }
    });
    static bool geoid_warned = false;

    for (const auto &enu : enu_positions)
    {
        double lat = 0.0, lon = 0.0, h = 0.0;
        geo_converter.Reverse(enu.x(), enu.y(), enu.z(), lat, lon, h);

        double altitude_msl = h;
        if (geoid_available && geoid_ptr)
        {
            altitude_msl = h - geoid_ptr->operator()(lat, lon);
        }
        else if (!geoid_warned)
        {
            geoid_warned = true;
            LOG_WARN_STREAM("[DataSaver] Geoid model egm96-5 unavailable; KML altitude remains ellipsoidal. Error: " << geoid_error);
        }

        kml_file << "          " << std::setprecision(9) << lon << "," << lat << "," << std::setprecision(3) << (altitude_msl + altitude_offset) << "\n";
    }

    kml_file << "        </coordinates>\n";
    kml_file << "      </LineString>\n";
    kml_file << "    </Placemark>\n";

    // 额外输出一条“贴地”轨迹，避免被卫星影像/地形遮挡（不抬升高度，直接依地表渲染）
    kml_file << "    <Placemark>\n";
    kml_file << "      <name>Trajectory (Ground Draped)</name>\n";
    kml_file << "      <styleUrl>#trajectoryStyle</styleUrl>\n";
    kml_file << "      <LineString>\n";
    kml_file << "        <tessellate>1</tessellate>\n";
    kml_file << "        <altitudeMode>clampToGround</altitudeMode>\n";
    kml_file << "        <coordinates>\n";
    for (const auto &enu : enu_positions)
    {
        double lat = 0.0, lon = 0.0, h_tmp = 0.0;
        geo_converter.Reverse(enu.x(), enu.y(), enu.z(), lat, lon, h_tmp);
        // clampToGround 模式下可省略高度，仅输出 lon,lat 即可
        kml_file << "          " << std::setprecision(9) << lon << "," << lat << "\n";
    }
    kml_file << "        </coordinates>\n";
    kml_file << "      </LineString>\n";
    kml_file << "    </Placemark>\n";

    kml_file << "    <Placemark>\n";
    kml_file << "      <name>Map Origin</name>\n";
    kml_file << "      <Point>\n";
    kml_file << "        <altitudeMode>absolute</altitudeMode>\n";
    double origin_altitude_msl = origin_lla[2];
    if (geoid_available && geoid_ptr)
    {
        origin_altitude_msl = origin_lla[2] - geoid_ptr->operator()(origin_lla[0], origin_lla[1]);
    }
    else if (!geoid_warned)
    {
        geoid_warned = true;
        LOG_WARN_STREAM("[DataSaver] Geoid model egm96-5 unavailable; origin altitude remains ellipsoidal. Error: " << geoid_error);
    }

    kml_file << "        <coordinates>" << std::setprecision(9) << origin_lla[1] << "," << origin_lla[0]
             << "," << std::setprecision(3) << (origin_altitude_msl + altitude_offset) << "</coordinates>\n";
    kml_file << "      </Point>\n";
    kml_file << "    </Placemark>\n";

    kml_file << "  </Document>\n";
    kml_file << "</kml>\n";
    kml_file.close();
    std::cout << "Saved trajectory KML: " << file_path << std::endl;
}

void DataSaver::saveTrajectoryPointCloud(const std::vector<Vector7> &estimates,
                                         const std::string &file_path)
{
    if (estimates.empty())
    {
        return;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr trajectory(new pcl::PointCloud<pcl::PointXYZI>());
    trajectory->points.reserve(estimates.size());

    for (size_t i = 0; i < estimates.size(); ++i)
    {
        pcl::PointXYZI pt;
        pt.x = static_cast<float>(estimates[i](0));
        pt.y = static_cast<float>(estimates[i](1));
        pt.z = static_cast<float>(estimates[i](2));
        double intensity_value = (i < keyframeTimes.size()) ? keyframeTimes[i] : static_cast<double>(i);
        pt.intensity = static_cast<float>(intensity_value);
        trajectory->points.push_back(pt);
    }

    trajectory->width = trajectory->points.size();
    trajectory->height = 1;
    trajectory->is_dense = true;

    if (trajectory->points.empty())
    {
        return;
    }

    try
    {
        pcl::io::savePCDFileBinary(file_path, *trajectory);
        std::cout << "Saved trajectory point cloud: " << file_path << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Failed to save trajectory point cloud: " << e.what() << std::endl;
    }
}

void DataSaver::saveTimes(vector<double> _keyframeTimes)
{
    if (_keyframeTimes.empty())
    {
        //    LOG(ERROR) << "EMPTY KEYFRAME TIMES!";
        return;
    }
    this->keyframeTimes = _keyframeTimes;

    std::fstream pgTimeSaveStream(logs_path_ + "/times.txt",
                                  std::fstream::out);
    pgTimeSaveStream.precision(15);

    // save timestamp
    for (auto const timestamp : keyframeTimes)
    {
        pgTimeSaveStream << timestamp << std::endl; // path
    }

    pgTimeSaveStream.close();
}

void DataSaver::saveOdometryVerticesTUM(
    std::vector<nav_msgs::Odometry> keyframePosesOdom)
{
    const std::string tum_path = logs_path_ + "/odom_tum.txt";
    std::ofstream tum_stream(tum_path, std::ofstream::out);
    if (!tum_stream.is_open())
    {
        LOG_ERROR_STREAM("[DataSaver] Failed to open odom_tum.txt for writing at " << tum_path);
        return;
    }

    tum_stream.precision(15);

    for (int i = 0; i < keyframePosesOdom.size(); i++)
    {
        nav_msgs::Odometry odometry = keyframePosesOdom.at(i);
        double time = odometry.header.stamp.toSec();
        // check the size of keyframeTimes
        tum_stream << time << " " << odometry.pose.pose.position.x << " "
                   << odometry.pose.pose.position.y << " "
                   << odometry.pose.pose.position.z << " "
                   << odometry.pose.pose.orientation.x << " "
                   << odometry.pose.pose.orientation.y << " "
                   << odometry.pose.pose.orientation.z << " "
                   << odometry.pose.pose.orientation.w << std::endl;
    }
}

void DataSaver::SetPriorGlobalMap(const pcl::PointCloud<PointT>::Ptr &map)
{
    prior_global_map_cache_ = map;
}


void DataSaver::saveEdgeErrors(const std::string &filename, gtsam::ISAM2 *isam, gtsam::Values &estimate)
{
    std::ofstream errorFile(save_directory + filename, std::fstream::out);
    if (!errorFile.is_open())
    {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }
    std::cout << "save graph error file: " << filename << std::endl;

    // First, save all node poses
    for (const auto &key_value : estimate)
    {
        gtsam::Key key = key_value.key;
        //        if (gtsam::Symbol(key).chr() == 'X') {  // Assuming 'x' is used for pose variables
        size_t nodeIndex = gtsam::Symbol(key).index();
        gtsam::Pose3 pose = estimate.at<gtsam::Pose3>(key);
        gtsam::Point3 translation = pose.translation();
        gtsam::Quaternion rotation = pose.rotation().toQuaternion();
        errorFile << nodeIndex << " "
                  << translation.x() << " " << translation.y() << " " << translation.z() << " "
                  << rotation.x() << " " << rotation.y() << " " << rotation.z() << " " << rotation.w()
                  << std::endl;
        //        }
    }

    // 保存每条边的误差
    for (const auto &factor : isam->getFactorsUnsafe())
    {
        if (auto f = dynamic_cast<gtsam::NoiseModelFactor *>(factor.get()))
        {
            gtsam::Vector error = f->whitenedError(estimate);
            double weight = f->weight(estimate);
            //            errorFile << "Factor between: ";
            //            for (const auto& key : f->keys()) {
            //                errorFile << DefaultKeyFormatter(key) << " ";
            //            }
            //            errorFile << "Error: " << error.transpose()
            //                      << " Weight: " << weight << std::endl;
            for (const auto &key : f->keys())
            {
                // 提取节点序号
                size_t nodeIndex = Symbol(key).index();
                errorFile << nodeIndex << " ";
            }
            errorFile << error.transpose() << " ";

            // 检查是否为BetweenFactor<Pose3>类型
            if (auto betweenFactor = dynamic_cast<BetweenFactor<Pose3> *>(f))
            {
                Pose3 measured = betweenFactor->measured();
                gtsam::Vector6 errorVector = error;
                //                errorFile << "Error (rotation then translation): "
                //                          << errorVector.head<3>().transpose() << " "  // 旋转误差
                //                          << errorVector.tail<3>().transpose() << " "; // 平移误差
                //                errorFile << "Measured: "
                //                          << measured.rotation().rpy().transpose() << " "  // 旋转测量
                //                          << measured.translation().transpose() << " ";    // 平移测量
                //                errorFile << measured.transpose() << " ";    // 平移测量
            }
            else
            {
                // prior factor
                // errorFile << "Error: " << error.transpose() << " ";
            }
            errorFile << weight << std::endl;
        }
    }
    errorFile.close();
}

void DataSaver::saveGraphGtsam(gtsam::NonlinearFactorGraph gtSAMgraph,
                               gtsam::ISAM2 *isam,
                               gtsam::Values isamCurrentEstimate)
{
    WriteGraphG2o(logs_path_ + "/pose_graph.g2o", gtSAMgraph, isamCurrentEstimate);
    std::cout << "WRITE G2O FILE: " << logs_path_ + "/pose_graph.g2o"
              << std::endl;
    std::cout << "Variable size: " << isamCurrentEstimate.size() << std::endl;
    std::cout << "Nonlinear factor size: " << gtSAMgraph.size()
              << std::endl;

    // write pose cov
    if (this->keyframeTimes.empty())
    {
        std::cout << "Empty keyframeTimes: " << save_directory << ", Pls save keyframeTimes first!"
                  << std::endl;
        return;
    }

    std::fstream pcovfile(logs_path_ + "/pose_cov.txt", std::fstream::out);
    pcovfile.precision(15);
    //     save timestamp
    for (int i = 0; i < keyframeTimes.size(); i++)
    {
        if (!isamCurrentEstimate.exists(X(i)))
        {
            // 异步保存时可能抓到未优化完成的新节点，跳过缺失的节点以避免异常
            continue;
        }
        pcovfile << keyframeTimes.at(i);
        Matrix6 poseCov = isam->marginalCovariance(X(i));
        // write upper triangular part of the covariance matrix
        // in the order of translation followed by rotation
        for (int row = 0; row < 6; row++)
        {
            for (int col = row; col < 6; col++)
            {
                // swap the order of translation and rotation
                int swapped_row = (row < 3) ? (row + 3) : (row - 3);
                int swapped_col = (col < 3) ? (col + 3) : (col - 3);
                pcovfile << " " << poseCov(swapped_row, swapped_col);
            }
        }
        pcovfile << std::endl;
    }
    pcovfile.close();
}

void DataSaver::saveGraph(std::vector<nav_msgs::Odometry> keyframePosesOdom)
{
    std::fstream g2o_outfile(save_directory + "odom.g2o", std::fstream::out);
    g2o_outfile.precision(15);
    // g2o_outfile << std::fixed << std::setprecision(9);

    for (int i = 0; i < keyframePosesOdom.size(); i++)
    {
        nav_msgs::Odometry odometry = keyframePosesOdom.at(i);
        double time = odometry.header.stamp.toSec();

        g2o_outfile << "VERTEX_SE3:QUAT " << std::to_string(i) << " ";
        g2o_outfile << odometry.pose.pose.position.x << " ";
        g2o_outfile << odometry.pose.pose.position.y << " ";
        g2o_outfile << odometry.pose.pose.position.z << " ";
        g2o_outfile << odometry.pose.pose.orientation.x << " ";
        g2o_outfile << odometry.pose.pose.orientation.y << " ";
        g2o_outfile << odometry.pose.pose.orientation.z << " ";
        g2o_outfile << odometry.pose.pose.orientation.w << std::endl;
    }
    //  LOG(INFO) << "WRITE G2O VERTICES: " << keyframePosesOdom.size();
    g2o_outfile.close();
}

void DataSaver::saveLogBag(std::vector<Vector12> logVec)
{
    // save log files
    //    rosbag::Bag result_bag;
    //    result_bag.open(save_directory + sequence_name + "_log.bag",
    //    rosbag::bagmode::Write);
    //
    //    if (logVec.size()){
    //        ROS_ERROR("SAVE RESULT BAG FAILED, EMPTY!");
    //        return;
    //    }
    //
    //    for (int i = 0; i < logVec.size(); ++i) {
    //
    //    }
}

void DataSaver::saveResultBag(
    std::vector<nav_msgs::Odometry> allOdometryVec,
    std::vector<nav_msgs::Odometry> updatedOdometryVec,
    std::vector<sensor_msgs::PointCloud2> allResVec)
{
    rosbag::Bag result_bag;
    result_bag.open(save_directory + sequence_name + "_result.bag",
                    rosbag::bagmode::Write);

    std::cout << "save bias and velocity " << allOdometryVec.size() << ", "
              << updatedOdometryVec.size() << std::endl;
    for (int i = 0; i < allOdometryVec.size(); i++)
    {
        nav_msgs::Odometry _laserOdometry = allOdometryVec.at(i);
        result_bag.write("lio_odometry", _laserOdometry.header.stamp,
                         _laserOdometry);

        nav_msgs::Odometry updateOdometry = updatedOdometryVec.at(i);
        result_bag.write("pgo_odometry", _laserOdometry.header.stamp,
                         updateOdometry);

        //        sensor_msgs::PointCloud2 _laserCloudFullRes = allResVec.at(i);
        //        result_bag.write("cloud_deskewed",
        //        _laserCloudFullRes.header.stamp, _laserCloudFullRes);
    }
    std::cout << "save bias and velocity " << allOdometryVec.size() << std::endl;

    //    for (int i = 0; i < updatedOdometryVec.size(); i++) {
    //        nav_msgs::Odometry _laserOdometry = updatedOdometryVec.at(i);
    //        result_bag.write("pgo_odometry", _laserOdometry.header.stamp,
    //        _laserOdometry);
    //    }

    //    for (int i = 0; i < allResVec.size(); i++) {
    //        sensor_msgs::PointCloud2 _laserCloudFullRes = allResVec.at(i);
    //        result_bag.write("cloud_deskewed", _laserCloudFullRes.header.stamp,
    //        _laserCloudFullRes);
    //    }
    result_bag.close();
}

void DataSaver::saveResultBag(std::vector<nav_msgs::Odometry> allOdometryVec,
                              std::vector<sensor_msgs::PointCloud2> allResVec)
{
    rosbag::Bag result_bag;
    result_bag.open(save_directory + sequence_name + "_result.bag",
                    rosbag::bagmode::Write);

    std::cout << "odom and cloud size: " << allOdometryVec.size() << "--"
              << allResVec.size() << std::endl;

    if (allOdometryVec.size() == allOdometryVec.size())
    {
        LOG_ERROR_STREAM("SAVE RESULT BAG FAILED");
        return;
    }

    //  LOG(INFO) << "ODOM AND PCD SIZE:" << allOdometryVec.size() << ", " <<
    //  allResVec.size();
    for (int i = 0; i < allOdometryVec.size(); i++)
    {
        nav_msgs::Odometry _laserOdometry = allOdometryVec.at(i);
        result_bag.write("pgo_odometry", _laserOdometry.header.stamp,
                         _laserOdometry);
    }

    for (int i = 0; i < allResVec.size(); i++)
    {
        sensor_msgs::PointCloud2 _laserCloudFullRes = allResVec.at(i);
        result_bag.write("cloud_deskewed", _laserCloudFullRes.header.stamp,
                         _laserCloudFullRes);
    }
    result_bag.close();
}

void DataSaver::saveResultBag(
    std::vector<nav_msgs::Odometry> allOdometryVec,
    std::vector<pcl::PointCloud<PointT>::Ptr> allResVec)
{
    rosbag::Bag result_bag;
    result_bag.open(save_directory + sequence_name + "_result.bag",
                    rosbag::bagmode::Write);

    //  LOG(INFO) << "ODOM AND PCD SIZE:" << allOdometryVec.size() << ", " <<
    //  allResVec.size();

    for (int i = 0; i < allResVec.size(); i++)
    {
        nav_msgs::Odometry _laserOdometry = allOdometryVec.at(i);
        result_bag.write("pgo_odometry", _laserOdometry.header.stamp,
                         _laserOdometry);

        sensor_msgs::PointCloud2 pointCloud2Msg;
        pcl::PointCloud<PointT>::Ptr _laserCloudFullRes = allResVec.at(i);
        pcl::toROSMsg(*_laserCloudFullRes, pointCloud2Msg);
        pointCloud2Msg.header = _laserOdometry.header;
        pointCloud2Msg.header.frame_id = "camera_init";
        result_bag.write("cloud_deskewed", _laserOdometry.header.stamp,
                         pointCloud2Msg);
    }
    result_bag.close();
    //  LOG(INFO) << "WRITE ROSBAG: " << save_directory + "_result.bag" << ",
    //  SIZE: " << result_bag.getSize();
}

void DataSaver::saveLoopandImagePair(
    std::map<int, int> loopIndexCheckedMap,
    std::vector<std::vector<int>> all_camera_corre_match_pair)
{
    std::ofstream loop_outfile;
    loop_outfile.open(save_directory + "lidar_loop.txt", std::ios::out);
    loop_outfile.precision(15);
    //  LOG(INFO) << "WRITE Lidar LOOP FILE: " << save_directory +
    //  "lidar_loop.txt";

    int j = 0;
    for (auto it = loopIndexCheckedMap.begin(); it != loopIndexCheckedMap.end();
         ++it)
    {
        int curr_node_idx = it->first;
        int prev_node_idx = it->second;

        //    geometry_msgs::Point p;
        //    p.x = keyframePosesUpdated[curr_node_idx].x;
        //    p.y = keyframePosesUpdated[curr_node_idx].y;
        //    p.z = keyframePosesUpdated[curr_node_idx].z;
        //
        //    p.x = keyframePosesUpdated[prev_node_idx].x;
        //    p.y = keyframePosesUpdated[prev_node_idx].y;
        //    p.z = keyframePosesUpdated[prev_node_idx].z;
        //
        //    // we can write some edges to g2o file
        //    //    g2o_out<<"EDGE_SE3:QUAT "<<curr_node_idx<<" "<<prev_node_idx<<"
        //    "
        //    //        <<p.x() <<" "<<p.y() <<" "<<p.z() <<" "
        //    //        <<q.x()<<" "<<q.y()<<" "<<q.z()<<" "<<q.w()<<" ";
        //
        //    if (saveLoopdata) {
        //      std::string common_name = std::to_string(curr_node_idx) + "_" +
        //      std::to_string(prev_node_idx);
        //
        //      std::string pcd_name_0 = common_name + "_0.pcd";
        //      std::string pcd_name_1 = common_name + "_1.pcd";
        //      pcl::io::savePCDFileBinary(pgScansDirectory + pcd_name_0,
        //      *keyframeLaserRawClouds[curr_node_idx]);
        //      pcl::io::savePCDFileBinary(pgScansDirectory + pcd_name_1,
        //      *keyframeLaserRawClouds[prev_node_idx]);
        //
        ////      cv::imwrite(pgImageDirectory + common_name + "_0_0.png",
        /// keyMeasures.at(curr_node_idx).camera0.front()); /
        /// cv::imwrite(pgImageDirectory + common_name + "_0_1.png",
        /// keyMeasures.at(curr_node_idx).camera1.front()); /
        /// cv::imwrite(pgImageDirectory + common_name + "_0_2.png",
        /// keyMeasures.at(curr_node_idx).camera2.front()); /
        /// cv::imwrite(pgImageDirectory + common_name + "_0_3.png",
        /// keyMeasures.at(curr_node_idx).camera3.front()); /
        /// cv::imwrite(pgImageDirectory + common_name + "_0_4.png",
        /// keyMeasures.at(curr_node_idx).camera4.front()); /      //
        /// cv::imshow("imgCallback", image_mat);
        ////
        ////      cv::imwrite(pgImageDirectory + common_name + "_1_0.png",
        /// keyMeasures.at(prev_node_idx).camera0.front()); /
        /// cv::imwrite(pgImageDirectory + common_name + "_1_1.png",
        /// keyMeasures.at(prev_node_idx).camera1.front()); /
        /// cv::imwrite(pgImageDirectory + common_name + "_1_2.png",
        /// keyMeasures.at(prev_node_idx).camera2.front()); /
        /// cv::imwrite(pgImageDirectory + common_name + "_1_3.png",
        /// keyMeasures.at(prev_node_idx).camera3.front()); /
        /// cv::imwrite(pgImageDirectory + common_name + "_1_4.png",
        /// keyMeasures.at(prev_node_idx).camera4.front());
        //      cv::imwrite(pgImageDirectory + common_name + "_1_4.png",
        //      resultMat_vec.at(j));
        //    }
        j++;
        loop_outfile.precision(15);
        loop_outfile << std::to_string(curr_node_idx) << " "
                     << std::to_string(prev_node_idx);
        loop_outfile << std::endl;
    }
    loop_outfile.close();
    //  LOG(INFO) << "SAVE LOOP FILE: " << loopIndexCheckedMap.size();

    // save camera pairs if their correspondences are sufficient
    std::ofstream camera_pair_outfile;
    camera_pair_outfile.open(save_directory + "camera_pair_indices.txt",
                             std::ios::out);
    //  LOG(INFO) << "WRITE CAMERA PAIR FILE: " << save_directory +
    //  "camera_pair_indices.txt"; LOG(INFO) << "Matching camera size: " <<
    //  all_camera_corre_match_pair.size();
    for (const auto &camera_pair : all_camera_corre_match_pair)
    {
        int lidar_idx_1 = camera_pair[0];
        int lidar_idx_2 = camera_pair[1];
        int cam_idx_1 = camera_pair[2];
        int cam_idx_2 = camera_pair[3];
        int num_corr = camera_pair[4];
        camera_pair_outfile << lidar_idx_1 << " " << lidar_idx_2 << " " << cam_idx_1
                            << " " << cam_idx_2 << " " << num_corr << std::endl;
    }
    camera_pair_outfile.close();
}

void DataSaver::savePointCloudMap(
    const std::vector<nav_msgs::Odometry> &allOdometryVec,
    const std::vector<pcl::PointCloud<PointT>::Ptr> &allResVec)
{
    LOG_INFO_STREAM("[DataSaver] Saving point cloud map, frames: " << allOdometryVec.size()
                    << ", clouds: " << allResVec.size());
    if (allOdometryVec.size() != allResVec.size())
    {
        LOG_WARN_STREAM("[DataSaver] Point cloud count does not match odometry count");
    }

    auto laserCloudTrans = AcquirePointIScratch();
    pcl::PointCloud<PointT>::Ptr globalmap(new pcl::PointCloud<PointT>());
    auto traj_cloud = AcquireTrajectoryScratch();
    traj_cloud->width = allOdometryVec.size();
    traj_cloud->height = 1;
    traj_cloud->is_dense = false;
    std::size_t new_traj_points = 0;

    const bool transform_to_lidar_frame = !saveResultBodyFrame;
    const auto &body_to_lidar = BodyToLidarMatrixF();
    const bool save_pcd = ShouldSaveKeyframePCD();
    const bool save_bin = ShouldSaveKeyframeBin();
    bool bin_writer_ready = save_bin && InitializeKeyframeBinWriter();
    if (save_bin && !bin_writer_ready)
    {
        LOG_WARN_STREAM("[DataSaver] Keyframe bin export requested but initialization failed.");
    }

    for (size_t i = 0; i < allOdometryVec.size(); ++i)
    {
        const auto &odom = allOdometryVec[i];
        const auto &laserCloudRaw = allResVec[i];
        if (!laserCloudRaw || laserCloudRaw->empty())
        {
            continue;
        }

        Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
        transform.rotate(Eigen::Quaterniond(
            odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y, odom.pose.pose.orientation.z));
        transform.pretranslate(Eigen::Vector3d(odom.pose.pose.position.x,
                                               odom.pose.pose.position.y,
                                               odom.pose.pose.position.z));

        if (save_key_frame && (save_pcd || bin_writer_ready))
        {
            if (i % 200 == 0 && (save_pcd || save_bin))
            {
                LOG_INFO_STREAM("[DataSaver] Keyframe save progress: " << i << "/" << allOdometryVec.size());
            }

            if (laserCloudRaw && !laserCloudRaw->empty())
            {
                pcl::PointCloud<PointT>::Ptr keyframe_cloud;
                if (transform_to_lidar_frame)
                {
                    keyframe_cloud = boost::make_shared<pcl::PointCloud<PointT>>();
                    pcl::transformPointCloud(*laserCloudRaw, *keyframe_cloud, body_to_lidar);
                }
                else
                {
                    keyframe_cloud = laserCloudRaw;
                }

                if (save_pcd)
                {
                    const std::string key_path = keyFrmaePath + std::to_string(i) + ".pcd";
                    if (!WritePointCloudBinary(key_path, *keyframe_cloud))
                    {
                        std::cerr << "[ERROR] Failed to save key frame cloud: " << key_path << std::endl;
                    }
                }
                if (bin_writer_ready)
                {
                    double timestamp = (i < keyframeTimes.size())
                                           ? keyframeTimes[i]
                                           : odom.header.stamp.toSec();
                    Eigen::Matrix4d pose = transform.matrix();
                    pcl::PointCloud<PointT>::Ptr bin_cloud = laserCloudRaw;

                    // [FIX] 确保bin文件中点云和位姿坐标系一致
                    // 之前的bug: 当transform_to_lidar_frame=true时，位姿转换到lidar frame，
                    // 但点云还是body frame，导致bin文件中数据坐标系不匹配
                    if (transform_to_lidar_frame)
                    {
                        // 如果要保存lidar frame，将点云和位姿都转换到lidar frame
                        pose = BodyToLidarMatrix() * pose;
                        bin_cloud = boost::make_shared<pcl::PointCloud<PointT>>();
                        pcl::transformPointCloud(*laserCloudRaw, *bin_cloud, body_to_lidar);
                    }
                    // 否则保存body frame，点云和位姿都保持body frame

                    if (!AppendKeyframeBin(*bin_cloud, timestamp, pose))
                    {
                        bin_writer_ready = false;
                    }
                }
            }
            else
            {
                LOG_DEBUG_STREAM("[DataSaver] Empty keyframe, skip writing");
            }
        }

        laserCloudTrans->clear();
        pcl::transformPointCloud(*laserCloudRaw, *laserCloudTrans,
                                 transform.matrix());

        if (transform_to_lidar_frame)
        {
            pcl::transformPointCloud(*laserCloudTrans, *laserCloudTrans, body_to_lidar);
        }
        *globalmap += *laserCloudTrans;

        pcl::PointXYZ pt;
        Eigen::Vector4f position(odom.pose.pose.position.x,
                                 odom.pose.pose.position.y,
                                 odom.pose.pose.position.z,
                                 1.0f);
        if (transform_to_lidar_frame)
        {
            position = body_to_lidar * position;
        }
        pt.x = position.x();
        pt.y = position.y();
        pt.z = position.z();
        traj_cloud->points.push_back(pt);
        ++new_traj_points;
    }

    // 合并先验轨迹点（M2F + 旧地图存在轨迹 PCD）
    if (useMultiMode && baseline == 1 && !map_directory.empty())
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr legacy_traj(new pcl::PointCloud<pcl::PointXYZ>());
        if (LoadLegacyTrajectoryPoints(legacy_traj) && legacy_traj && !legacy_traj->empty())
        {
            if (transform_to_lidar_frame)
            {
                pcl::transformPointCloud(*legacy_traj, *legacy_traj, body_to_lidar);
            }
            traj_cloud->insert(traj_cloud->end(), legacy_traj->begin(), legacy_traj->end());
            LOG_INFO_STREAM("[DataSaver] Merged trajectory points: new=" << new_traj_points
                            << ", prior=" << legacy_traj->size()
                            << ", merged_total=" << traj_cloud->size());
        }
        else
        {
            LOG_WARN_STREAM_THROTTLE(5.0, "[DataSaver] No legacy trajectory merged (missing or empty).");
        }
    }

    FinalizeKeyframeBinWriter();

    LOG_INFO_STREAM("[DataSaver] Trajectory point count: " << traj_cloud->size());
    if (!traj_cloud->empty())
    {
        const std::string traj_filename = saveResultBodyFrame ? "trajectory_imu.pcd" : "trajectory_lidar.pcd";
        const std::string traj_path = PcdmapFile(traj_filename);

        // 输出带强度的轨迹，新点/旧点用不同 intensity 区分
        pcl::PointCloud<pcl::PointXYZI> traj_with_intensity;
        traj_with_intensity.reserve(traj_cloud->size());
        for (std::size_t i = 0; i < traj_cloud->size(); ++i)
        {
            pcl::PointXYZI pt;
            pt.x = traj_cloud->points[i].x;
            pt.y = traj_cloud->points[i].y;
            pt.z = traj_cloud->points[i].z;
            pt.intensity = (i < new_traj_points) ? 100.0f : 60.0f; // 新session=2.0，旧session=1.0
            traj_with_intensity.push_back(pt);
        }

        if (!WritePointCloudBinary(traj_path, traj_with_intensity))
        {
            std::cerr << "[ERROR] Failed to save trajectory point cloud: " << traj_path << std::endl;
        }
        // No legacy copy; pcdmap is the canonical location.
    }

    const std::size_t pre_filter_size = globalmap->size();
    // M2F 多 session 保存时，将先验地图与当前地图合并后再导出。
    LOG_INFO_STREAM("[DataSaver] Map merge check: useMultiMode=" << useMultiMode
                    << ", baseline=" << baseline << ", map_directory=" << (map_directory.empty() ? "(empty)" : map_directory));
    if (useMultiMode && baseline == 1 && !map_directory.empty())
    {
        LOG_INFO_STREAM("[DataSaver] M2F mode detected, attempting to merge with prior map...");
        pcl::PointCloud<PointT>::Ptr prior_map(new pcl::PointCloud<PointT>());

        // 优先从磁盘加载旧地图的原始高分辨率 PCD，用于最终合并
        if (LoadLegacyGlobalMap(prior_map, /*apply_downsample=*/false))
        {
            LOG_INFO_STREAM("[DataSaver] ✓ Loaded prior map from disk for M2F export (original resolution): points="
                            << prior_map->size());
        }
        else if (prior_global_map_cache_ && !prior_global_map_cache_->empty())
        {
            // 回退：如果磁盘加载失败，则使用运行中缓存的降采样地图
            prior_map = prior_global_map_cache_;
            LOG_WARN_STREAM("[DataSaver] Falling back to cached prior map for M2F export: points=" << prior_map->size());
        }
        else
        {
            LOG_WARN_STREAM("[DataSaver] ✗ Prior map unavailable for M2F export; exporting new segment only.");
            LOG_WARN_STREAM("[DataSaver]   Check that map_directory contains a valid global_map_dense_imu.pcd or similar file.");
            prior_map.reset();
        }

        if (prior_map && !prior_map->empty())
        {
            const std::size_t prior_size = prior_map->size();
            LOG_INFO_STREAM("[DataSaver] Merging maps: new_points=" << globalmap->size() << " + prior_points=" << prior_size);
            *globalmap += *prior_map;
            LOG_INFO_STREAM("[DataSaver] ✓ Map merge complete: total_points=" << globalmap->size());
        }
        else
        {
            LOG_WARN_STREAM("[DataSaver] ✗ Prior map is null or empty, skipping merge. Only new session will be saved!");
        }
    }
    else
    {
        if (!useMultiMode)
            LOG_INFO_STREAM("[DataSaver] Single-session mode (useMultiMode=false), skipping prior map merge.");
        else if (baseline != 1)
            LOG_INFO_STREAM("[DataSaver] F2F mode (baseline=" << baseline << "), skipping prior map merge.");
        else
            LOG_INFO_STREAM("[DataSaver] map_directory is empty, skipping prior map merge.");
    }

    const std::size_t merged_size = globalmap->size();
    globalmap = FilterMapOutliers(globalmap);
    const std::size_t post_filter_size = globalmap ? globalmap->size() : 0;
    if (post_filter_size < pre_filter_size)
    {
        LOG_INFO_STREAM("[DataSaver] Map outlier filtering removed " << (pre_filter_size - post_filter_size)
                        << " points (" << pre_filter_size << " -> " << post_filter_size << ")");
    }
    else if (post_filter_size < merged_size)
    {
        LOG_INFO_STREAM("[DataSaver] Map outlier filtering removed "
                        << (merged_size - post_filter_size) << " points after prior merge.");
    }
    LOG_INFO_STREAM("[DataSaver] Global map point count: " << post_filter_size);

    if (globalmap && !globalmap->empty())
    {
        const std::string map_filename = saveResultBodyFrame ? "global_map_dense_imu.pcd" : "global_map_dense_lidar.pcd";
        const std::string map_path = PcdmapFile(map_filename);
        std::vector<std::future<void>> tasks;

        tasks.emplace_back(std::async(std::launch::async, [this, map_path, map_filename, globalmap]() {
            if (!WritePointCloudBinary(map_path, *globalmap))
            {
                LOG_ERROR_STREAM("[DataSaver] Failed to write global map: " << map_path);
            }
            // Map stored solely under pcdmap.
        }));

        if (export_grid_map_)
        {
            auto grid_cloud = globalmap;
            tasks.emplace_back(std::async(std::launch::async, [this, grid_cloud]() {
                try
                {
                    SaveLocalizationMap(grid_cloud);
                }
                catch (const std::exception &e)
                {
                    LOG_ERROR_STREAM("[DataSaver] Failed to save grid map: " << e.what());
                }
            }));
        }

        if (export_intensity_image_)
        {
            auto image_cloud = globalmap;
            pcl::PointCloud<pcl::PointXYZ>::ConstPtr trajectory_cloud = traj_cloud;
            const double effective_resolution =
                (intensity_image_resolution_ > 0.0) ? intensity_image_resolution_ : map_saved_size;
            tasks.emplace_back(std::async(std::launch::async, [this, image_cloud, trajectory_cloud, effective_resolution]() mutable {
                if (!SavePointCloudAsImage(image_cloud, sequence_name + "_intensity",
                                           static_cast<float>(effective_resolution),
                                           trajectory_cloud))
                {
                    LOG_WARN_STREAM("[DataSaver] Failed to generate intensity image");
                }
            }));
        }

        for (auto &task : tasks)
        {
            task.get();
        }
    }

    // Generate version.json in pcdmap root directory
    if (!pcdmap_root_path_.empty())
    {
        namespace fs = std::filesystem;
        fs::path version_json_path = fs::path(pcdmap_root_path_) / "version.json";
        GenerateVersionJson(version_json_path.string());
    }

    LOG_INFO_STREAM("[DataSaver] Map save complete, points: " << globalmap->size());
}

void DataSaver::savePointCloudMap(
    const std::vector<nav_msgs::Odometry> &allOdometryVec,
    const std::vector<pcl::PointCloud<PointT>::Ptr> &allResVec,
    int start_index,
    bool full_export)
{
    LOG_INFO_STREAM("[DataSaver] Saving map segment, start index: " << start_index
                    << ", total frames: " << allOdometryVec.size());
    (void)full_export;  // image命名与single session保持一致，避免未使用参数告警
    if (allOdometryVec.size() != allResVec.size())
    {
        LOG_WARN_STREAM("[DataSaver] Point cloud count does not match odometry count");
    }

    auto laserCloudTrans = AcquirePointIScratch();
    pcl::PointCloud<PointT>::Ptr globalmap(new pcl::PointCloud<PointT>());
    auto traj_cloud = AcquireTrajectoryScratch();
    traj_cloud->clear();
    traj_cloud->is_dense = false;
    std::size_t new_traj_points = 0;
    const bool transform_to_lidar_frame = !saveResultBodyFrame;
    const auto &body_to_lidar = BodyToLidarMatrixF();
    const bool save_pcd = ShouldSaveKeyframePCD();
    const bool save_bin = ShouldSaveKeyframeBin();
    bool bin_writer_ready = save_bin && InitializeKeyframeBinWriter();
    if (save_bin && !bin_writer_ready)
    {
        LOG_WARN_STREAM("[DataSaver] Keyframe bin export requested but initialization failed.");
    }

    // begin from the requested index
    const int total_frames = static_cast<int>(allOdometryVec.size());
    const int begin_index = std::max(0, start_index);
    for (int i = begin_index; i < total_frames; i++)
    {
        const auto &odom = allOdometryVec[i];
        const auto &laserCloudRaw = allResVec[i];
        if (!laserCloudRaw || laserCloudRaw->empty())
        {
            continue;
        }

        Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
        transform.rotate(Eigen::Quaterniond(
            odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y, odom.pose.pose.orientation.z));
        transform.pretranslate(Eigen::Vector3d(odom.pose.pose.position.x,
                                               odom.pose.pose.position.y,
                                               odom.pose.pose.position.z));

        if (save_key_frame && (save_pcd || bin_writer_ready))
        {
            if (laserCloudRaw && !laserCloudRaw->empty())
            {
                pcl::PointCloud<PointT>::Ptr keyframe_cloud;
                if (transform_to_lidar_frame)
                {
                    keyframe_cloud = boost::make_shared<pcl::PointCloud<PointT>>();
                    pcl::transformPointCloud(*laserCloudRaw, *keyframe_cloud, body_to_lidar);
                }
                else
                {
                    keyframe_cloud = laserCloudRaw;
                }

                if (save_pcd)
                {
                    const std::string key_path = keyFrmaePath + std::to_string(i) + ".pcd";
                    if (!WritePointCloudBinary(key_path, *keyframe_cloud))
                    {
                        std::cerr << "[ERROR] Failed to save key frame cloud: " << key_path << std::endl;
                    }
                }
                if (bin_writer_ready)
                {
                    double timestamp = (i < keyframeTimes.size())
                                           ? keyframeTimes[i]
                                           : odom.header.stamp.toSec();
                    Eigen::Matrix4d pose = transform.matrix();
                    pcl::PointCloud<PointT>::Ptr bin_cloud = laserCloudRaw;

                    // [FIX] 确保bin文件中点云和位姿坐标系一致
                    // 之前的bug: 当transform_to_lidar_frame=true时，位姿转换到lidar frame，
                    // 但点云还是body frame，导致bin文件中数据坐标系不匹配
                    if (transform_to_lidar_frame)
                    {
                        // 如果要保存lidar frame，将点云和位姿都转换到lidar frame
                        pose = BodyToLidarMatrix() * pose;
                        bin_cloud = boost::make_shared<pcl::PointCloud<PointT>>();
                        pcl::transformPointCloud(*laserCloudRaw, *bin_cloud, body_to_lidar);
                    }
                    // 否则保存body frame，点云和位姿都保持body frame

                    if (!AppendKeyframeBin(*bin_cloud, timestamp, pose))
                    {
                        bin_writer_ready = false;
                    }
                }
            }
        }

        laserCloudTrans->clear();
        pcl::transformPointCloud(*laserCloudRaw, *laserCloudTrans, transform.matrix());
        if (transform_to_lidar_frame)
        {
            pcl::transformPointCloud(*laserCloudTrans, *laserCloudTrans, body_to_lidar);
        }
        *globalmap += *laserCloudTrans;

        pcl::PointXYZ pt;
        Eigen::Vector4f position(odom.pose.pose.position.x,
                                 odom.pose.pose.position.y,
                                 odom.pose.pose.position.z,
                                 1.0f);
        if (transform_to_lidar_frame)
        {
            position = body_to_lidar * position;
        }
        pt.x = position.x();
        pt.y = position.y();
        pt.z = position.z();
        traj_cloud->points.push_back(pt);
        ++new_traj_points;

        if (i % 1000 == 0)
        {
            LOG_INFO_STREAM("[DataSaver] Incremental map progress: " << i << "/" << total_frames);
        }
    }

    // 合并先验轨迹点（M2F + 旧地图存在轨迹 PCD）
    if (useMultiMode && baseline == 1 && !map_directory.empty())
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr legacy_traj(new pcl::PointCloud<pcl::PointXYZ>());
        if (LoadLegacyTrajectoryPoints(legacy_traj) && legacy_traj && !legacy_traj->empty())
        {
            if (transform_to_lidar_frame)
            {
                pcl::transformPointCloud(*legacy_traj, *legacy_traj, body_to_lidar);
            }
            traj_cloud->insert(traj_cloud->end(), legacy_traj->begin(), legacy_traj->end());
            LOG_INFO_STREAM("[DataSaver] Merged trajectory points: new=" << new_traj_points
                            << ", prior=" << legacy_traj->size()
                            << ", merged_total=" << traj_cloud->size());
        }
        else
        {
            LOG_WARN_STREAM_THROTTLE(5.0, "[DataSaver] No legacy trajectory merged (missing or empty).");
        }
    }

    FinalizeKeyframeBinWriter();

    LOG_INFO_STREAM("[DataSaver] Trajectory point count: " << traj_cloud->size());
    if (!traj_cloud->empty())
    {
        const std::string traj_filename = saveResultBodyFrame ? "trajectory_imu.pcd" : "trajectory_lidar.pcd";
        const std::string traj_path = PcdmapFile(traj_filename);

        pcl::PointCloud<pcl::PointXYZI> traj_with_intensity;
        traj_with_intensity.reserve(traj_cloud->size());
        for (std::size_t i = 0; i < traj_cloud->size(); ++i)
        {
            pcl::PointXYZI pt;
            pt.x = traj_cloud->points[i].x;
            pt.y = traj_cloud->points[i].y;
            pt.z = traj_cloud->points[i].z;
            pt.intensity = (i < new_traj_points) ? 2.0f : 1.0f;
            traj_with_intensity.push_back(pt);
        }

        if (!WritePointCloudBinary(traj_path, traj_with_intensity))
        {
            LOG_ERROR_STREAM("[DataSaver] Failed to save trajectory point cloud: " << traj_path);
        }
    }

    const std::size_t pre_filter_size = globalmap->size();

    // M2F 多 session 保存时，将先验地图与当前地图合并后再导出。
    LOG_INFO_STREAM("[DataSaver] Map merge check: useMultiMode=" << useMultiMode
                    << ", baseline=" << baseline << ", map_directory=" << (map_directory.empty() ? "(empty)" : map_directory));
    if (useMultiMode && baseline == 1 && !map_directory.empty())
    {
        LOG_INFO_STREAM("[DataSaver] M2F mode detected, attempting to merge with prior map...");
        pcl::PointCloud<PointT>::Ptr prior_map(new pcl::PointCloud<PointT>());

        // 优先从磁盘加载旧地图的原始高分辨率 PCD，用于最终合并
        if (LoadLegacyGlobalMap(prior_map, /*apply_downsample=*/false))
        {
            LOG_INFO_STREAM("[DataSaver] ✓ Loaded prior map from disk for M2F export (original resolution): points="
                            << prior_map->size());
        }
        else if (prior_global_map_cache_ && !prior_global_map_cache_->empty())
        {
            // 回退：如果磁盘加载失败，则使用运行中缓存的降采样地图
            prior_map = prior_global_map_cache_;
            LOG_WARN_STREAM("[DataSaver] Falling back to cached prior map for M2F export: points=" << prior_map->size());
        }
        else
        {
            LOG_WARN_STREAM("[DataSaver] ✗ Prior map unavailable for M2F export; exporting new segment only.");
            LOG_WARN_STREAM("[DataSaver]   Check that map_directory contains a valid global_map_dense_imu.pcd or similar file.");
            prior_map.reset();
        }

        if (prior_map && !prior_map->empty())
        {
            const std::size_t prior_size = prior_map->size();
            LOG_INFO_STREAM("[DataSaver] Merging maps: new_points=" << globalmap->size() << " + prior_points=" << prior_size);
            *globalmap += *prior_map;
            LOG_INFO_STREAM("[DataSaver] ✓ Map merge complete: total_points=" << globalmap->size());
        }
        else
        {
            LOG_WARN_STREAM("[DataSaver] ✗ Prior map is null or empty, skipping merge. Only new session will be saved!");
        }
    }
    else
    {
        if (!useMultiMode)
            LOG_INFO_STREAM("[DataSaver] Single-session mode (useMultiMode=false), skipping prior map merge.");
        else if (baseline != 1)
            LOG_INFO_STREAM("[DataSaver] F2F mode (baseline=" << baseline << "), skipping prior map merge.");
        else
            LOG_INFO_STREAM("[DataSaver] map_directory is empty, skipping prior map merge.");
    }

    const std::size_t merged_size = globalmap->size();
    globalmap = FilterMapOutliers(globalmap);
    const std::size_t post_filter_size = globalmap ? globalmap->size() : 0;
    if (post_filter_size < pre_filter_size)
    {
        LOG_INFO_STREAM("[DataSaver] Incremental map outlier filtering removed "
                        << (pre_filter_size - post_filter_size) << " points (" << pre_filter_size << " -> "
                        << post_filter_size << ")");
    }
    else if (post_filter_size < merged_size)
    {
        LOG_INFO_STREAM("[DataSaver] Map outlier filtering removed "
                        << (merged_size - post_filter_size) << " points after prior merge.");
    }

    if (globalmap && !globalmap->empty())
    {
        LOG_INFO_STREAM("[DataSaver] Final map point count: " << post_filter_size);
        const std::string map_filename = saveResultBodyFrame ? "global_map_dense_imu.pcd" : "global_map_dense_lidar.pcd";
        const std::string map_path = PcdmapFile(map_filename);
        std::vector<std::future<void>> tasks;
        tasks.emplace_back(std::async(std::launch::async, [this, map_path, globalmap]() {
            if (!WritePointCloudBinary(map_path, *globalmap))
            {
                LOG_ERROR_STREAM("[DataSaver] Failed to write global map: " << map_path);
            }
        }));

        if (export_grid_map_)
        {
            auto grid_cloud = globalmap;
            tasks.emplace_back(std::async(std::launch::async, [this, grid_cloud]() {
                try
                {
                    SaveLocalizationMap(grid_cloud);
                }
                catch (const std::exception &e)
                {
                    LOG_ERROR_STREAM("[DataSaver] Failed to save grid map: " << e.what());
                }
            }));
        }

        if (export_intensity_image_)
        {
            auto image_cloud = globalmap;
            const double effective_resolution =
                (intensity_image_resolution_ > 0.0) ? intensity_image_resolution_ : map_saved_size;
            const std::string image_basename = sequence_name;
            tasks.emplace_back(std::async(std::launch::async, [this, image_cloud, effective_resolution, image_basename]() mutable {
                if (!SavePointCloudAsImage(image_cloud, image_basename,
                                           static_cast<float>(effective_resolution),
                                           nullptr))
                {
                    LOG_WARN_STREAM("[DataSaver] Failed to generate intensity image");
                }
            }));
        }

        for (auto &task : tasks)
        {
            task.get();
        }
    }

    // Generate version.json in pcdmap root directory
    if (!pcdmap_root_path_.empty())
    {
        namespace fs = std::filesystem;
        fs::path version_json_path = fs::path(pcdmap_root_path_) / "version.json";
        GenerateVersionJson(version_json_path.string());
    }

    LOG_INFO_STREAM("[DataSaver] Incremental map save complete, points: " << post_filter_size);
}

void DataSaver::savePointCloudMap(
    const std::vector<Eigen::Isometry3d> &allOdometryVec,
    const std::vector<pcl::PointCloud<PointT>::Ptr> &allResVec)
{
    LOG_INFO_STREAM("[DataSaver] Saving ICP map, frames: " << allOdometryVec.size()
                    << ", clouds: " << allResVec.size());
    if (allOdometryVec.size() != allResVec.size())
    {
        LOG_WARN_STREAM("[DataSaver] ICP point cloud count does not match odometry count");
    }

    auto laserCloudTrans = AcquirePointIScratch();
    pcl::PointCloud<PointT>::Ptr globalmap(new pcl::PointCloud<PointT>());

    auto traj_cloud = AcquireTrajectoryScratch();
    traj_cloud->width = allOdometryVec.size();
    traj_cloud->height = 1;
    traj_cloud->is_dense = false;

    const bool transform_to_lidar_frame = !saveResultBodyFrame;
    const auto &body_to_lidar = BodyToLidarMatrixF();

    for (size_t i = 0; i < allOdometryVec.size(); ++i)
    {
        Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
        transform = allOdometryVec[i];
        const auto &laserCloudRaw = allResVec[i];
        if (!laserCloudRaw || laserCloudRaw->empty())
        {
            continue;
        }

        if (i % 200 == 0)
        {
            LOG_INFO_STREAM("[DataSaver] ICP point cloud progress: " << i << "/" << allOdometryVec.size());
        }

        laserCloudTrans->clear();
        pcl::transformPointCloud(*laserCloudRaw, *laserCloudTrans,
                                 transform.matrix());

        if (transform_to_lidar_frame)
        {
            pcl::transformPointCloud(*laserCloudTrans, *laserCloudTrans, body_to_lidar);
        }

        *globalmap += *laserCloudTrans;

        pcl::PointXYZ pt;
        Eigen::Vector4f translation(transform.translation().x(),
                                     transform.translation().y(),
                                     transform.translation().z(),
                                     1.0f);
        if (transform_to_lidar_frame)
        {
            translation = body_to_lidar * translation;
        }
        pt.x = translation.x();
        pt.y = translation.y();
        pt.z = translation.z();
        traj_cloud->points.push_back(pt);

        /*if (save_key_frame) {
          std::cout << " process point cloud: " << i << "/" << allOdometryVec.size()
                    << std::endl;
          if (!laserCloudRaw->empty()) {
            pcl::io::savePCDFileASCII(keyFrmaePath + std::to_string(i) + ".pcd",
                                      *laserCloudRaw);
          } else
            std::cout << "empty key frame " << i << std::endl;
        }*/
    }

    LOG_INFO_STREAM("[DataSaver] ICP trajectory point count: " << traj_cloud->size());
    if (!traj_cloud->empty())
    {
        const std::string traj_filename = saveResultBodyFrame ? "traj_icp_imu.pcd" : "traj_icp.pcd";
        const std::string traj_path = PcdmapFile(traj_filename);
        if (!WritePointCloudBinary(traj_path, *traj_cloud))
        {
            std::cerr << "[ERROR] Failed to save ICP trajectory: " << traj_path << std::endl;
        }
    }

    const std::size_t pre_filter_size = globalmap->size();
    globalmap = FilterMapOutliers(globalmap);
    const std::size_t post_filter_size = globalmap ? globalmap->size() : 0;
    if (post_filter_size < pre_filter_size)
    {
        LOG_INFO_STREAM("[DataSaver] ICP map outlier filtering removed " << (pre_filter_size - post_filter_size)
                        << " points (" << pre_filter_size << " -> " << post_filter_size << ")");
    }
    LOG_INFO_STREAM("[DataSaver] ICP global map point count: " << post_filter_size);

    // save point cloud in lidar frame
    // if you want to save it in body frame(imu)
    // i will update it later
    if (globalmap && !globalmap->empty())
    {
        const std::string map_filename = saveResultBodyFrame ? "map_icp_imu.pcd" : "map_icp.pcd";
        const std::string map_path = PcdmapFile(map_filename);
        if (!WritePointCloudBinary(map_path, *globalmap))
        {
            std::cerr << "[ERROR] Failed to save ICP global map: " << map_path << std::endl;
        }
        // Map stored solely under pcdmap.
    }
    LOG_INFO_STREAM("[DataSaver] ICP map save complete, points: " << post_filter_size);
}

void DataSaver::savePointCloudMapLIO(
    const std::vector<nav_msgs::Odometry> &allOdometryVec,
    const std::vector<pcl::PointCloud<PointT>::Ptr> &allResVec)
{
    LOG_INFO_STREAM("[DataSaver] Saving LIO map, frames: " << allOdometryVec.size()
                    << ", clouds: " << allResVec.size());
    if (allOdometryVec.size() != allResVec.size())
    {
        LOG_WARN_STREAM("[DataSaver] LIO point cloud count does not match odometry count");
    }
    auto laserCloudTrans = AcquirePointIScratch();
    pcl::PointCloud<PointT>::Ptr globalmap(new pcl::PointCloud<PointT>());
    auto traj_cloud = AcquireTrajectoryScratch();
    traj_cloud->width = allOdometryVec.size();
    traj_cloud->height = 1;
    traj_cloud->is_dense = false;

    const bool transform_to_lidar_frame = !saveResultBodyFrame;
    const auto &body_to_lidar = BodyToLidarMatrixF();

    for (size_t i = 0; i < allOdometryVec.size(); ++i)
    {
        const auto &odom = allOdometryVec[i];
        const auto &laserCloudRaw = allResVec[i];
        if (!laserCloudRaw || laserCloudRaw->empty())
        {
            continue;
        }

        Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
        transform.rotate(Eigen::Quaterniond(
            odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y, odom.pose.pose.orientation.z));
        transform.pretranslate(Eigen::Vector3d(odom.pose.pose.position.x,
                                               odom.pose.pose.position.y,
                                               odom.pose.pose.position.z));

        laserCloudTrans->clear();
        pcl::transformPointCloud(*laserCloudRaw, *laserCloudTrans,
                                 transform.matrix());

        if (transform_to_lidar_frame)
        {
            pcl::transformPointCloud(*laserCloudTrans, *laserCloudTrans, body_to_lidar);
        }

        pcl::PointXYZ pt;
        Eigen::Vector4f translation(odom.pose.pose.position.x,
                                     odom.pose.pose.position.y,
                                     odom.pose.pose.position.z,
                                     1.0f);
        if (transform_to_lidar_frame)
        {
            translation = body_to_lidar * translation;
        }
        pt.x = translation.x();
        pt.y = translation.y();
        pt.z = translation.z();
        traj_cloud->points.push_back(pt);
        /* if (save_key_frame) {
           std::cout << " process lio point cloud: " << i << "/"
                     << allOdometryVec.size() << std::endl;
           if (!laserCloudRaw->empty())
             pcl::io::savePCDFileASCII(
                 keyFrmaePath + std::to_string(i) + "_lidar_lio.pcd",
                 *laserCloudRaw);
         }*/
        *globalmap += *laserCloudTrans;
    }

    LOG_INFO_STREAM("[DataSaver] LIO trajectory point count: " << traj_cloud->size());
    if (!traj_cloud->empty())
    {
        const std::string traj_filename = saveResultBodyFrame ? "traj_pcd_imu_lio.pcd" : "traj_pcd_lidar_lio.pcd";
        const std::string traj_path = PcdmapFile(traj_filename);
        if (!WritePointCloudBinary(traj_path, *traj_cloud))
        {
            std::cerr << "[ERROR] Failed to save LIO trajectory: " << traj_path << std::endl;
        }
    }

    const std::size_t pre_filter_size = globalmap->size();
    globalmap = FilterMapOutliers(globalmap);
    const std::size_t post_filter_size = globalmap ? globalmap->size() : 0;
    if (post_filter_size < pre_filter_size)
    {
        LOG_INFO_STREAM("[DataSaver] LIO map outlier filtering removed " << (pre_filter_size - post_filter_size)
                        << " points (" << pre_filter_size << " -> " << post_filter_size << ")");
    }
    LOG_INFO_STREAM("[DataSaver] LIO global map point count: " << post_filter_size);

    // save point cloud in lidar frame
    // if you want to save it in body frame(imu)
    // i will update it later
    if (globalmap && !globalmap->empty())
    {
        const std::string map_filename = saveResultBodyFrame ? "map_lio_imu.pcd" : "map_lio.pcd";
        const std::string map_path = PcdmapFile(map_filename);
        if (!WritePointCloudBinary(map_path, *globalmap))
        {
            std::cerr << "[ERROR] Failed to save LIO global map: " << map_path << std::endl;
        }
    }
    LOG_INFO_STREAM("[DataSaver] LIO map save complete, points: " << post_filter_size);
}

void DataSaver::SaveSessionSubsetOutputs(const std::string &tag,
                                         const std::vector<nav_msgs::Odometry> &odoms,
                                         const std::vector<pcl::PointCloud<PointT>::Ptr> &clouds,
                                         const std::vector<Vector7> &poses,
                                         const std::vector<double> &times,
                                         const std::vector<Eigen::Vector3d> &trajectory_enu,
                                         const std::optional<Eigen::Vector3d> &origin_lla,
                                         const std::string &kml_color_hex)
{
    namespace fs = std::filesystem;
    // 轨迹和 TUM 导出只依赖位姿和时间，不再强制要求 odoms/clouds 数量完全一致，
    // 以便在 legacy 场景（只有轨迹、无 keyframe 点云）下仍能正确生成 old_session_trajectory_imu.pcd。
    const std::size_t pose_count = std::min(poses.size(), times.size());
    if (pose_count == 0)
    {
        LOG_WARN_STREAM("[DataSaver] SaveSessionSubsetOutputs(" << tag << ") skipped: empty inputs");
        return;
    }

    fs::path split_dir = fs::path(logs_path_.empty() ? (save_directory + "logs") : logs_path_) / "session_split";
    std::error_code dir_ec;
    fs::create_directories(split_dir, dir_ec);
    if (dir_ec)
    {
        LOG_WARN_STREAM("[DataSaver] SaveSessionSubsetOutputs failed to create directory: "
                        << split_dir << ", " << dir_ec.message());
        return;
    }

    const bool transform_to_lidar_frame = !saveResultBodyFrame;
    const auto &body_to_lidar = BodyToLidarMatrixF();

    // 1) TUM 轨迹
    fs::path tum_path = split_dir / (tag + "_optimized_poses_tum.txt");
    {
        std::ofstream tum(tum_path, std::ios::out | std::ios::trunc);
        tum.precision(15);
        for (std::size_t i = 0; i < pose_count; ++i)
        {
            const auto &pose = poses.at(i);
            tum << times.at(i) << " " << pose(0) << " " << pose(1) << " " << pose(2) << " "
                << pose(3) << " " << pose(4) << " " << pose(5) << " " << pose(6) << "\n";
        }
    }

    // 2) 轨迹 PCD
    pcl::PointCloud<PointT>::Ptr traj_cloud(new pcl::PointCloud<PointT>());
    traj_cloud->reserve(pose_count);
    for (std::size_t i = 0; i < pose_count; ++i)
    {
        PointT pt;
        pt.x = static_cast<float>(poses.at(i)(0));
        pt.y = static_cast<float>(poses.at(i)(1));
        pt.z = static_cast<float>(poses.at(i)(2));
        pt.intensity = 1.0f;
        traj_cloud->push_back(pt);
    }
    const std::string traj_name = saveResultBodyFrame ? (tag + "_trajectory_imu.pcd")
                                                      : (tag + "_trajectory_lidar.pcd");
    fs::path traj_path = split_dir / traj_name;
    WritePointCloudBinary(traj_path.string(), *traj_cloud);

    // 3) 地图 PCD（仅聚合，不保存分段 keyframe）
    pcl::PointCloud<PointT>::Ptr map_cloud(new pcl::PointCloud<PointT>());
    auto scratch = AcquirePointIScratch();
    const std::size_t map_count = std::min(pose_count, clouds.size());
    for (std::size_t i = 0; i < map_count; ++i)
    {
        const auto &cloud = clouds.at(i);
        if (!cloud || cloud->empty())
            continue;

        // 使用优化后的图优化位姿（poses）构造位姿变换，确保与 *_trajectory_imu.pcd 一致
        const auto &pose_vec = poses.at(i);
        Eigen::Vector3d t(pose_vec(0), pose_vec(1), pose_vec(2));
        Eigen::Quaterniond q(pose_vec(6), pose_vec(3), pose_vec(4), pose_vec(5));
        q.normalize();

        Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
        tf.rotate(q);
        tf.pretranslate(t);

        Eigen::Matrix4f tf_mat = tf.matrix().cast<float>();

        // cloud 为 body/IMU 帧点云；当 saveResultBodyFrame=false 时，poses 已转换为 LiDAR 位姿，
        // 需要将 body 点云先变换到 LiDAR 再投到 map。
        if (!saveResultBodyFrame)
        {
            const auto &body_to_lidar = BodyToLidarMatrixF();  // T_body_lidar
            tf_mat = tf_mat * body_to_lidar;                  // T_map_lidar * T_body_lidar
        }

        scratch->clear();
        pcl::transformPointCloud(*cloud, *scratch, tf_mat);
        *map_cloud += *scratch;
    }

    map_cloud = FilterMapOutliers(map_cloud);
    fs::path map_path = split_dir / (tag + "_map.pcd");
    if (map_cloud && !map_cloud->empty())
    {
        WritePointCloudBinary(map_path.string(), *map_cloud);
    }
    else
    {
        LOG_WARN_STREAM("[DataSaver] Subset map is empty for tag=" << tag);
    }

    // 4) KML（可选，传入不同颜色）
    if (origin_lla && !trajectory_enu.empty())
    {
        saveTrajectoryKML(trajectory_enu, *origin_lla, tag + ".kml", kml_color_hex, split_dir.string());
    }

    LOG_INFO_STREAM("[DataSaver] Subset outputs saved to " << split_dir << " tag=" << tag
                    << " tum=" << tum_path.filename()
                    << " traj_pcd=" << traj_path.filename()
                    << " map_pcd=" << map_path.filename());
}

void DataSaver::writeDeskedFrame(pcl::PointCloud<PointT>::Ptr pc, int index)
{
    if (!ShouldSaveKeyframePCD())
    {
        return;
    }
    std::string path = keyFrmaePath + std::to_string(index) + ".pcd";
    LOG_DEBUG_STREAM("[DataSaver] 写入去畸变点云: " << path);
    if (!pc || pc->empty())
    {
        return;
    }
    if (!WritePointCloudBinary(path, *pc))
    {
        std::cerr << "[ERROR] Failed to save desked frame: " << path << std::endl;
    }
}

bool DataSaver::WriteGraphG2o(const std::string &filepath,
                              const gtsam::NonlinearFactorGraph &graph,
                              const gtsam::Values &estimates) const
{
    if (filepath.empty())
    {
        LOG_WARN_STREAM("[DataSaver] WriteGraphG2o skipped: filepath is empty.");
        return false;
    }

    try
    {
        gtsam::writeG2o(graph, estimates, filepath);
        AppendGnssFactorsToG2o(filepath, graph);
        AppendAnchorPriorToG2o(filepath, graph, graph);
        return true;
    }
    catch (const std::exception &e)
    {
        LOG_ERROR_STREAM("[DataSaver] Failed to write g2o file " << filepath << ": " << e.what());
        return false;
    }
}

void DataSaver::ArchiveEssentialOutputs()
{
    namespace fs = std::filesystem;

    if (save_directory.empty())
    {
        return;
    }

    fs::path base(save_directory);
    std::error_code ec;
    if (!fs::exists(base, ec))
    {
        LOG_WARN_STREAM("[DataSaver] Archive skipped: save directory missing -> " << base);
        return;
    }

    auto trim_copy = [](const std::string &value) -> std::string {
        if (value.empty())
        {
            return std::string();
        }
        std::size_t begin = 0;
        std::size_t end = value.size();
        while (begin < end && std::isspace(static_cast<unsigned char>(value[begin])))
        {
            ++begin;
        }
        while (end > begin && std::isspace(static_cast<unsigned char>(value[end - 1])))
        {
            --end;
        }
        return value.substr(begin, end - begin);
    };

    fs::path calibration_source_path;
    bool calibration_source_available = false;
    std::string calibration_param_source;
    if (ros::param::get(kCalibrationParamRosKey, calibration_param_source))
    {
        calibration_param_source = trim_copy(calibration_param_source);
        if (!calibration_param_source.empty())
        {
            fs::path candidate = calibration_param_source;
            std::error_code source_ec;
            if (fs::exists(candidate, source_ec) && !source_ec)
            {
                calibration_source_path = candidate;
                calibration_source_available = true;
            }
            else
            {
                LOG_WARN_STREAM("[DataSaver] calibration_param.yaml source unavailable: "
                                << calibration_param_source << (source_ec ? (", " + source_ec.message()) : ""));
            }
        }
    }

    auto copy_calibration_file = [&](const fs::path &destination) -> bool {
        if (!calibration_source_available || destination.empty())
        {
            return false;
        }
        std::error_code eq_ec;
        if (fs::equivalent(calibration_source_path, destination, eq_ec) && !eq_ec)
        {
            return true;
        }
        std::error_code dir_ec;
        fs::create_directories(destination.parent_path(), dir_ec);
        if (dir_ec)
        {
            LOG_WARN_STREAM("[DataSaver] Failed to prepare calibration destination "
                            << destination.generic_string() << ": " << dir_ec.message());
            return false;
        }
        std::error_code copy_ec;
        fs::copy_file(calibration_source_path, destination,
                      fs::copy_options::overwrite_existing, copy_ec);
        if (copy_ec)
        {
            LOG_WARN_STREAM("[DataSaver] Failed to copy calibration_param.yaml -> "
                            << destination.generic_string() << ": " << copy_ec.message());
            return false;
        }
        return true;
    };

    if (calibration_source_available)
    {
        copy_calibration_file(base / "calibration_param.yaml");
    }

    // Provide alias files expected by downstream tooling.
    if (!sequence_name.empty() && !useMultiMode)
    {
        fs::path intensity_png = base / (sequence_name + "_intensity.png");
        fs::path plain_png = base / (sequence_name + ".png");
        if (fs::exists(intensity_png) && !fs::exists(plain_png))
        {
            std::error_code copy_ec;
            fs::copy_file(intensity_png, plain_png, fs::copy_options::overwrite_existing, copy_ec);
            if (copy_ec)
            {
                LOG_WARN_STREAM("[DataSaver] Failed to create image alias: " << copy_ec.message());
            }
        }

        fs::path intensity_meta = base / (sequence_name + "_intensity_metadata.txt");
        fs::path plain_meta = base / (sequence_name + "_metadata.txt");
        if (fs::exists(intensity_meta) && !fs::exists(plain_meta))
        {
            std::error_code copy_ec;
            fs::copy_file(intensity_meta, plain_meta, fs::copy_options::overwrite_existing, copy_ec);
            if (copy_ec)
            {
                LOG_WARN_STREAM("[DataSaver] Failed to create metadata alias: " << copy_ec.message());
            }
        }

        fs::path intensity_xz_png = base / (sequence_name + "_intensity_xz.png");
        fs::path plain_xz_png = base / (sequence_name + "_xz.png");
        if (fs::exists(intensity_xz_png) && !fs::exists(plain_xz_png))
        {
            std::error_code copy_ec;
            fs::copy_file(intensity_xz_png, plain_xz_png, fs::copy_options::overwrite_existing, copy_ec);
            if (copy_ec)
            {
                LOG_WARN_STREAM("[DataSaver] Failed to create XZ image alias: " << copy_ec.message());
            }
        }

        fs::path intensity_xz_meta = base / (sequence_name + "_intensity_xz_metadata.txt");
        fs::path plain_xz_meta = base / (sequence_name + "_xz_metadata.txt");
        if (fs::exists(intensity_xz_meta) && !fs::exists(plain_xz_meta))
        {
            std::error_code copy_ec;
            fs::copy_file(intensity_xz_meta, plain_xz_meta, fs::copy_options::overwrite_existing, copy_ec);
            if (copy_ec)
            {
                LOG_WARN_STREAM("[DataSaver] Failed to create XZ metadata alias: " << copy_ec.message());
            }
        }

        fs::path intensity_yz_png = base / (sequence_name + "_intensity_yz.png");
        fs::path plain_yz_png = base / (sequence_name + "_yz.png");
        if (fs::exists(intensity_yz_png) && !fs::exists(plain_yz_png))
        {
            std::error_code copy_ec;
            fs::copy_file(intensity_yz_png, plain_yz_png, fs::copy_options::overwrite_existing, copy_ec);
            if (copy_ec)
            {
                LOG_WARN_STREAM("[DataSaver] Failed to create YZ image alias: " << copy_ec.message());
            }
        }

        fs::path intensity_yz_meta = base / (sequence_name + "_intensity_yz_metadata.txt");
        fs::path plain_yz_meta = base / (sequence_name + "_yz_metadata.txt");
        if (fs::exists(intensity_yz_meta) && !fs::exists(plain_yz_meta))
        {
            std::error_code copy_ec;
            fs::copy_file(intensity_yz_meta, plain_yz_meta, fs::copy_options::overwrite_existing, copy_ec);
            if (copy_ec)
            {
                LOG_WARN_STREAM("[DataSaver] Failed to create YZ metadata alias: " << copy_ec.message());
            }
        }
    }

    const std::string logs_dir_name = logs_directory_name_.empty() ? "logs" : logs_directory_name_;
    auto logs_relative = [&](const std::string &relative) {
        return fs::path(logs_dir_name) / relative;
    };

    const std::set<std::string> excluded_roots = {"key_point_frame", logs_dir_name};
    const std::set<std::string> excluded_files = {
            "data_time.txt",
            "pose_cov.txt",
            "times.txt",
            "odom_tum.txt",
            "runtime_params.yaml",
            "optimized_poses_tum.txt",
            "pose_graph.g2o",
            "pose_graph_3d_result.txt",
            "mapping.json",
            "README.txt"
    };

    std::set<std::string> unique_entries;
    std::vector<fs::path> tar_entries;

    fs::path selected_origin_cfg;
    fs::path selected_trajectory_pcd;
    fs::path selected_global_map;
    fs::path selected_optimized_path;
    fs::path selected_intensity_png;
    fs::path selected_intensity_metadata;
    fs::path selected_intensity_xz_png;
    fs::path selected_intensity_xz_metadata;
    fs::path selected_intensity_yz_png;
    fs::path selected_intensity_yz_metadata;
    fs::path selected_runtime_params;

    auto register_entry = [&](const fs::path &relative_path) {
        std::string entry = relative_path.generic_string();
        if (entry == "pcdmap" || entry == "maps")
        {
            return;
        }
        if (unique_entries.insert(entry).second)
        {
            tar_entries.push_back(relative_path);
        }
    };

    auto should_exclude_initial_pair = [&](const std::string &filename) {
        if (!useMultiMode || sequence_name.empty())
            return false;
        const std::string prefix = sequence_name + "_";
        if (filename.size() <= prefix.size())
            return false;
        if (!std::equal(prefix.begin(), prefix.end(), filename.begin()))
            return false;
        const std::string suffix = filename.substr(prefix.size());
        return suffix == "init.pcd" || suffix == "localmap.pcd";
    };

    auto should_skip_for_archive = [&](const fs::path &relative_path) {
        const std::string filename = relative_path.filename().string();
        if (should_exclude_initial_pair(filename))
            return true;
        return excluded_files.count(filename) > 0;
    };

    auto register_optional = [&](const fs::path &relative_path,
                                 fs::path *selected_path = nullptr) {
        if (relative_path.empty())
        {
            return;
        }
        fs::path full = base / relative_path;
        std::error_code check_ec;
        if (!fs::exists(full, check_ec) || check_ec)
        {
            return;
        }
        if (!should_skip_for_archive(relative_path))
        {
            register_entry(relative_path);
        }
        if (selected_path && selected_path->empty())
        {
            *selected_path = relative_path;
        }
    };

    auto require_single = [&](const std::string &desc,
                              const fs::path &relative_path,
                              bool directory = false,
                              bool add_to_archive = true,
                              fs::path *selected_path = nullptr) {
        fs::path full = base / relative_path;
        std::error_code check_ec;
        bool exists = directory ? fs::is_directory(full, check_ec) : fs::exists(full, check_ec);
        if (exists && !check_ec)
        {
            if (add_to_archive && !should_skip_for_archive(relative_path))
            {
                register_entry(relative_path);
            }
            if (selected_path && selected_path->empty())
            {
                *selected_path = relative_path;
            }
            return;
        }
        LOG_WARN_STREAM("[DataSaver] Missing required " << desc << ": " << relative_path.generic_string());
    };

    auto require_candidates = [&](const std::string &desc,
                                  const std::vector<fs::path> &candidates,
                                  bool directory = false,
                                  bool add_to_archive = true,
                                  fs::path *selected_path = nullptr) {
        bool found = false;
        for (const auto &relative_path : candidates)
        {
            fs::path full = base / relative_path;
            std::error_code check_ec;
            bool exists = directory ? fs::is_directory(full, check_ec) : fs::exists(full, check_ec);
            if (exists && !check_ec)
            {
                if (add_to_archive && !should_skip_for_archive(relative_path))
                {
                    register_entry(relative_path);
                }
                if (selected_path && selected_path->empty())
                {
                    *selected_path = relative_path;
                }
                found = true;
                break;
            }
        }
        if (!found)
        {
            std::ostringstream oss;
            for (size_t i = 0; i < candidates.size(); ++i)
            {
                if (i)
                {
                    oss << ", ";
                }
                oss << candidates[i].generic_string();
            }
            LOG_WARN_STREAM("[DataSaver] Missing required " << desc << " (checked: " << oss.str() << ")");
        }
    };

    require_candidates("maps directory",
                       std::vector<fs::path>{fs::path("pcdmap/maps"),
                                             fs::path("maps")},
                       true,
                       true);

    if (saveResultBodyFrame)
    {
        require_candidates("global map point cloud",
                           std::vector<fs::path>{fs::path("pcdmap/global_map_dense_imu.pcd"),
                                                 fs::path("pcdmap/map_icp_imu.pcd"),
                                                 fs::path("pcdmap/map_lio_imu.pcd"),
                                                 fs::path("global_map_dense_imu.pcd"),
                                                 fs::path("map_icp_imu.pcd"),
                                                 fs::path("map_lio_imu.pcd")},
                           false,
                           true,
                           &selected_global_map);

        require_candidates("trajectory point cloud",
                           std::vector<fs::path>{fs::path("pcdmap/traj_pcd_imu.pcd"),
                                                 fs::path("pcdmap/traj_icp_imu.pcd"),
                                                 fs::path("pcdmap/traj_pcd_imu_lio.pcd"),
                                                 fs::path("traj_pcd_imu.pcd"),
                                                 fs::path("traj_icp_imu.pcd"),
                                                 fs::path("traj_pcd_imu_lio.pcd")},
                           false,
                           true,
                           &selected_trajectory_pcd);
    }
    else
    {
        require_candidates("global map point cloud",
                           std::vector<fs::path>{fs::path("pcdmap/global_map_dense_lidar.pcd"),
                                                 fs::path("pcdmap/map_icp.pcd"),
                                                 fs::path("pcdmap/map_lio.pcd"),
                                                 fs::path("global_map_dense_lidar.pcd"),
                                                 fs::path("map_icp.pcd"),
                                                 fs::path("map_lio.pcd")},
                           false,
                           true,
                           &selected_global_map);

        require_candidates("trajectory point cloud",
                           std::vector<fs::path>{fs::path("pcdmap/trajectory_imu.pcd"),
                                                 fs::path("pcdmap/traj_pcd_imu.pcd"),
                                                 fs::path("pcdmap/traj_icp_imu.pcd"),
                                                 fs::path("pcdmap/traj_pcd_imu_lio.pcd"),
                                                 fs::path("pcdmap/trajectory_lidar.pcd"),
                                                 fs::path("pcdmap/traj_pcd_lidar.pcd"),
                                                 fs::path("pcdmap/traj_icp.pcd"),
                                                 fs::path("pcdmap/traj_pcd_lidar_lio.pcd"),
                                                 fs::path("trajectory_imu.pcd"),
                                                 fs::path("traj_pcd_imu.pcd"),
                                                 fs::path("traj_icp_imu.pcd"),
                                                 fs::path("traj_pcd_imu_lio.pcd"),
                                                 fs::path("trajectory_lidar.pcd"),
                                                 fs::path("traj_pcd_lidar.pcd"),
                                                 fs::path("traj_icp.pcd"),
                                                 fs::path("traj_pcd_lidar_lio.pcd")},
                           false,
                           true,
                           &selected_trajectory_pcd);
    }

    require_candidates("optimized trajectory",
                       std::vector<fs::path>{logs_relative("optimized_poses_tum.txt"),
                                             logs_relative("optimized_odom_tum.txt"),
                                             logs_relative("optimized_odom_kitti.txt"),
                                             fs::path("optimized_poses_tum.txt"),
                                             fs::path("optimized_odom_tum.txt"),
                                             fs::path("optimized_odom_kitti.txt")},
                       false,
                       true,
                       &selected_optimized_path);

    require_candidates("origin_position.cfg",
                       std::vector<fs::path>{fs::path("pcdmap/origin_position.cfg"),
                                             fs::path("origin_position.cfg")},
                       false,
                       true,
                       &selected_origin_cfg);

    if (!sequence_name.empty())
    {
        require_candidates("intensity image",
                           std::vector<fs::path>{fs::path("pcdmap/" + sequence_name + ".png"),
                                                 fs::path("pcdmap/" + sequence_name + "_intensity.png"),
                                                 fs::path(sequence_name + ".png"),
                                                 fs::path(sequence_name + "_intensity.png")},
                           false,
                           true,
                           &selected_intensity_png);

        require_candidates("intensity metadata",
                           std::vector<fs::path>{fs::path("pcdmap/" + sequence_name + "_metadata.txt"),
                                                 fs::path("pcdmap/" + sequence_name + "_intensity_metadata.txt"),
                                                 fs::path(sequence_name + "_metadata.txt"),
                                                 fs::path(sequence_name + "_intensity_metadata.txt")},
                           false,
                           true,
                           &selected_intensity_metadata);

        auto register_with_fallback = [&](const fs::path &primary,
                                          const fs::path &secondary,
                                          fs::path *selected_path = nullptr) {
            bool need_fallback = true;
            if (!primary.empty())
            {
                register_optional(primary, selected_path);
                if (selected_path)
                {
                    if (!selected_path->empty())
                    {
                        need_fallback = false;
                    }
                }
                else
                {
                    std::error_code exists_ec;
                    if (fs::exists(base / primary, exists_ec) && !exists_ec)
                    {
                        need_fallback = false;
                    }
                }
            }
            if (need_fallback && !secondary.empty())
            {
                register_optional(secondary, selected_path);
            }
        };

        register_with_fallback(fs::path("pcdmap/" + sequence_name + "_intensity_xz.png"),
                               fs::path(sequence_name + "_intensity_xz.png"),
                               &selected_intensity_xz_png);
        register_with_fallback(fs::path("pcdmap/" + sequence_name + "_intensity_xz_metadata.txt"),
                               fs::path(sequence_name + "_intensity_xz_metadata.txt"),
                               &selected_intensity_xz_metadata);
        register_with_fallback(fs::path("pcdmap/" + sequence_name + "_xz.png"),
                               fs::path(sequence_name + "_xz.png"));
        register_with_fallback(fs::path("pcdmap/" + sequence_name + "_xz_metadata.txt"),
                               fs::path(sequence_name + "_xz_metadata.txt"));
        register_with_fallback(fs::path("pcdmap/" + sequence_name + "_intensity_yz.png"),
                               fs::path(sequence_name + "_intensity_yz.png"),
                               &selected_intensity_yz_png);
        register_with_fallback(fs::path("pcdmap/" + sequence_name + "_intensity_yz_metadata.txt"),
                               fs::path(sequence_name + "_intensity_yz_metadata.txt"),
                               &selected_intensity_yz_metadata);
        register_with_fallback(fs::path("pcdmap/" + sequence_name + "_yz.png"),
                               fs::path(sequence_name + "_yz.png"));
        register_with_fallback(fs::path("pcdmap/" + sequence_name + "_yz_metadata.txt"),
                               fs::path(sequence_name + "_yz_metadata.txt"));
    }

    auto compute_duration_seconds = [&](const std::vector<fs::path> &candidates,
                                        double &duration_out) -> bool {
        for (const auto &relative_path : candidates)
        {
            fs::path full = base / relative_path;
            std::ifstream stream(full.string());
            if (!stream.is_open())
            {
                continue;
            }
            double first = 0.0;
            double last = 0.0;
            double value = 0.0;
            bool has_value = false;
            while (stream >> value)
            {
                if (!has_value)
                {
                    first = value;
                }
                last = value;
                has_value = true;
            }
            if (!has_value)
            {
                continue;
            }
            duration_out = std::max(0.0, last - first);
            return true;
        }
        return false;
    };

    auto compute_mapping_distance = [&](const fs::path &trajectory_rel,
                                        double &distance_out) -> bool {
        if (trajectory_rel.empty())
        {
            return false;
        }
        fs::path full = base / trajectory_rel;
        std::ifstream stream(full.string());
        if (!stream.is_open())
        {
            LOG_WARN_STREAM("[DataSaver] Failed to open trajectory for distance computation: "
                            << full.generic_string());
            return false;
        }

        std::string line;
        Eigen::Vector3d previous = Eigen::Vector3d::Zero();
        bool has_previous = false;
        double total = 0.0;
        while (std::getline(stream, line))
        {
            if (line.empty())
            {
                continue;
            }
            std::istringstream line_stream(line);
            double timestamp = 0.0;
            double x = 0.0;
            double y = 0.0;
            double z = 0.0;
            double qx = 0.0;
            double qy = 0.0;
            double qz = 0.0;
            double qw = 1.0;
            if (!(line_stream >> timestamp >> x >> y >> z >> qx >> qy >> qz >> qw))
            {
                continue;
            }
            Eigen::Vector3d current(x, y, z);
            if (has_previous)
            {
                total += (current - previous).norm();
            }
            previous = current;
            has_previous = true;
        }

        if (!has_previous)
        {
            LOG_WARN_STREAM("[DataSaver] No valid pose samples found in "
                            << full.generic_string() << " for distance computation.");
            return false;
        }
        distance_out = total;
        return true;
    };

    auto write_keyframe_bin = [&](const fs::path &output_abs,
                                  const std::vector<fs::path> &pcd_sources) -> bool {
        std::error_code dir_ec;
        fs::create_directories(output_abs.parent_path(), dir_ec);
        if (dir_ec)
        {
            LOG_WARN_STREAM("[DataSaver] Failed to prepare keyframe bin directory "
                            << output_abs.parent_path().generic_string() << ": " << dir_ec.message());
            return false;
        }

        std::ofstream bin_stream(output_abs.string(), std::ios::binary | std::ios::trunc);
        if (!bin_stream.is_open())
        {
            LOG_WARN_STREAM("[DataSaver] Failed to open keyframe bin for writing: "
                            << output_abs.generic_string());
            return false;
        }

        const uint64_t zero = 0;
        bin_stream.write(reinterpret_cast<const char *>(&zero), sizeof(uint64_t));
        bin_stream.write(reinterpret_cast<const char *>(&zero), sizeof(uint64_t));

        uint64_t total_points = 0;
        uint64_t frames_written = 0;

        for (const auto &source_path : pcd_sources)
        {
            pcl::PointCloud<PointT> cloud;
            if (pcl::io::loadPCDFile<PointT>(source_path.string(), cloud) != 0)
            {
                LOG_WARN_STREAM("[DataSaver] Failed to read keyframe PCD: "
                                << source_path.generic_string());
                continue;
            }
            if (cloud.empty())
            {
                continue;
            }

            for (const auto &point : cloud.points)
            {
                const float record[4] = {
                    static_cast<float>(point.x),
                    static_cast<float>(point.y),
                    static_cast<float>(point.z),
                    static_cast<float>(point.intensity)
                };
                bin_stream.write(reinterpret_cast<const char *>(record), sizeof(record));
            }

            total_points += static_cast<uint64_t>(cloud.size());
            ++frames_written;
        }

        bin_stream.seekp(0, std::ios::beg);
        bin_stream.write(reinterpret_cast<const char *>(&frames_written), sizeof(uint64_t));
        bin_stream.write(reinterpret_cast<const char *>(&total_points), sizeof(uint64_t));
        bin_stream.close();

        if (!frames_written)
        {
            LOG_WARN_STREAM("[DataSaver] No keyframe points were written to "
                            << output_abs.generic_string());
            return false;
        }
        if (!total_points)
        {
            LOG_WARN_STREAM("[DataSaver] Keyframe bin contains zero points: "
                            << output_abs.generic_string());
            return false;
        }

        LOG_INFO_STREAM("[DataSaver] Wrote keyframe bin " << output_abs.generic_string()
                        << " (" << frames_written << " frames, "
                        << total_points << " points)");
        return true;
    };

    // 归档文件名将在后面根据 folder_name 生成
    auto now = std::time(nullptr);
    char time_buf[32] = {0};
#if defined(_GNU_SOURCE) || defined(__APPLE__)
    std::tm lt;
    localtime_r(&now, &lt);
    std::strftime(time_buf, sizeof(time_buf), "%Y%m%d_%H%M%S", &lt);
#else
    std::tm *lt = std::localtime(&now);
    std::strftime(time_buf, sizeof(time_buf), "%Y%m%d_%H%M%S", lt);
#endif
    const std::string scene = sequence_name.empty() ? std::string("scene") : sequence_name;

    ec.clear();
    for (fs::directory_iterator it(base, ec), end; !ec && it != end; ++it)
    {
        fs::path rel = it->path().lexically_relative(base);
        const std::string name = rel.generic_string();
        if (rel.empty())
        {
            continue;
        }

        const std::string root = rel.begin()->string();
        if (excluded_roots.count(root) > 0)
        {
            continue;
        }
        // 跳过 tar.gz 文件
        if (name.size() >= 7 && name.compare(name.size() - 7, 7, ".tar.gz") == 0)
        {
            continue;
        }
        if (should_skip_for_archive(rel))
        {
            continue;
        }
        register_entry(rel);
    }
    if (ec)
    {
        LOG_WARN_STREAM("[DataSaver] Failed to iterate save directory: " << ec.message());
    }

    require_single("trajectory KML", logs_relative("trajectory.kml"), false, false);
    require_single("runtime parameters", fs::path("mapping") / "runtime_params.yaml", false, false, &selected_runtime_params);

    require_single("odometry export", logs_relative("odom_tum.txt"), false, false);

    if (tar_entries.empty())
    {
        LOG_WARN_STREAM("[DataSaver] No artifacts available for archiving; skip compression.");
        return;
    }

    auto shell_quote = [](const std::string &value) {
        std::string quoted;
        quoted.reserve(value.size() + 2);
        quoted.push_back(static_cast<char>(39));
        for (char c : value)
        {
            if (c == static_cast<char>(39))
            {
                quoted += "'\\''";
            }
            else
            {
                quoted.push_back(c);
            }
        }
        quoted.push_back(static_cast<char>(39));
        return quoted;
    };

    fs::path staging_root;
    try
    {
        staging_root = fs::temp_directory_path();
    }
    catch (const std::exception &)
    {
        staging_root = base;
    }
    staging_root /= "ms_mapping_archive_stage_" + std::string(time_buf);
    fs::path pcdmap_root = staging_root / "pcdmap";
    fs::path label_map_root = staging_root / "label_map";

    ec.clear();
    fs::remove_all(staging_root, ec);

    auto copy_into_root = [&](const fs::path &source, const fs::path &destination_root, const fs::path &target_rel) -> bool {
        if (source.empty())
        {
            return false;
        }
        std::error_code exists_ec;
        if (!fs::exists(source, exists_ec))
        {
            LOG_WARN_STREAM("[DataSaver] Staging source missing: " << source.generic_string());
            return false;
        }

        fs::path destination = destination_root / target_rel;
        std::error_code dir_ec;
        fs::create_directories(destination.parent_path(), dir_ec);
        if (dir_ec)
        {
            LOG_WARN_STREAM("[DataSaver] Failed to prepare staging directory for " << destination.generic_string()
                            << ": " << dir_ec.message());
            return false;
        }

        std::error_code clear_ec;
        fs::remove_all(destination, clear_ec);

        std::error_code copy_ec;
        if (fs::is_directory(source))
        {
            fs::copy(source, destination,
                     fs::copy_options::recursive | fs::copy_options::overwrite_existing, copy_ec);
        }
        else
        {
            fs::copy_file(source, destination, fs::copy_options::overwrite_existing, copy_ec);
        }

        if (copy_ec)
        {
            LOG_WARN_STREAM("[DataSaver] Failed to copy staging item " << source.generic_string()
                            << " -> " << destination.generic_string()
                            << ": " << copy_ec.message());
            return false;
        }
        return true;
    };

    auto stage_into_pcdmap = [&](const fs::path &source_rel) -> bool {
        return copy_into_root(base / source_rel, pcdmap_root, source_rel);
    };

    auto stage_into_label_map = [&](const fs::path &source_rel, const fs::path &target_name) -> bool {
        return copy_into_root(base / source_rel, label_map_root, target_name);
    };

    ec.clear();
    fs::create_directories(pcdmap_root, ec);
    if (ec)
    {
        LOG_ERROR_STREAM("[DataSaver] Failed to create archive staging directory: " << ec.message());
        return;
    }

    auto is_same_path = [](const fs::path &lhs, const fs::path &rhs) {
        if (lhs.empty() || rhs.empty())
        {
            return false;
        }
        return lhs.generic_string() == rhs.generic_string();
    };

    auto should_skip_in_pcdmap = [&](const fs::path &entry) {
        if (!entry.empty())
        {
            const std::string root = entry.begin()->string();
            if (root == "mapping")
            {
                return true;
            }
        }
        const std::string filename = entry.filename().string();
        if (is_same_path(entry, selected_intensity_png) || is_same_path(entry, selected_intensity_metadata) ||
            is_same_path(entry, selected_intensity_xz_png) || is_same_path(entry, selected_intensity_xz_metadata) ||
            is_same_path(entry, selected_intensity_yz_png) || is_same_path(entry, selected_intensity_yz_metadata))
        {
            return true;
        }
        if (filename == "trajectory.kml")
        {
            return true;
        }
        // Skip global_map_dense_imu.pcd from pcdmap since it's already in labelmap
        if (filename == "global_map_dense_imu.pcd" && entry.parent_path().filename() == "pcdmap")
        {
            return true;
        }
        if (!sequence_name.empty())
        {
            if (filename == sequence_name + ".png" || filename == sequence_name + "_metadata.txt" ||
                filename == sequence_name + "_intensity_xz.png" ||
                filename == sequence_name + "_intensity_xz_metadata.txt" ||
                filename == sequence_name + "_xz.png" ||
                filename == sequence_name + "_xz_metadata.txt" ||
                filename == sequence_name + "_intensity_yz.png" ||
                filename == sequence_name + "_intensity_yz_metadata.txt" ||
                filename == sequence_name + "_yz.png" ||
                filename == sequence_name + "_yz_metadata.txt")
            {
                return true;
            }
        }
        return false;
    };

    bool staged_any = false;
    for (const auto &entry : tar_entries)
    {
        if (entry.empty())
        {
            continue;
        }

        const std::string root = entry.begin()->string();
        if (root == "pcdmap")
        {
            fs::path subpath;
            for (auto it = std::next(entry.begin()); it != entry.end(); ++it)
            {
                subpath /= *it;
            }

            // Skip global_map_dense_imu.pcd from pcdmap (it's in label_map)
            if (!subpath.empty() && subpath.filename() == "global_map_dense_imu.pcd")
            {
                continue;
            }

            fs::path source_path = base / entry;
            if (subpath.empty())
            {
                // Copy entire pcdmap directory, but filter out global_map_dense_imu.pcd
                std::error_code copy_ec;
                fs::create_directories(pcdmap_root, copy_ec);
                if (copy_ec)
                {
                    LOG_WARN_STREAM("[DataSaver] Failed to create pcdmap directory in staging: "
                                    << copy_ec.message());
                }
                else
                {
                    // Manually iterate and copy files, excluding global_map_dense_imu.pcd
                    for (fs::recursive_directory_iterator it(source_path, copy_ec), end; it != end; ++it)
                    {
                        if (copy_ec)
                        {
                            break;
                        }
                        const fs::path &item = it->path();
                        if (item.filename() == "global_map_dense_imu.pcd")
                        {
                            continue;
                        }
                        fs::path rel = fs::relative(item, source_path, copy_ec);
                        if (copy_ec)
                        {
                            continue;
                        }
                        fs::path dest = pcdmap_root / rel;
                        if (fs::is_directory(item))
                        {
                            fs::create_directories(dest, copy_ec);
                        }
                        else if (fs::is_regular_file(item))
                        {
                            fs::create_directories(dest.parent_path(), copy_ec);
                            fs::copy_file(item, dest, fs::copy_options::overwrite_existing, copy_ec);
                        }
                    }
                    if (!copy_ec)
                    {
                        staged_any = true;
                    }
                }
            }
            else
            {
                if (copy_into_root(source_path, pcdmap_root, subpath))
                {
                    staged_any = true;
                }
            }
            continue;
        }

        if (should_skip_in_pcdmap(entry))
        {
            continue;
        }
        if (stage_into_pcdmap(entry))
        {
            staged_any = true;
        }
    }

    if (!staged_any)
    {
        LOG_WARN_STREAM("[DataSaver] Failed to stage any archive artifacts; skip compression.");
        fs::remove_all(staging_root, ec);
        return;
    }

    auto stage_intensity_into_pcdmap = [&](const fs::path &source_rel) {
        if (source_rel.empty())
        {
            return;
        }
        if (!copy_into_root(base / source_rel, pcdmap_root, source_rel.filename()))
        {
            LOG_WARN_STREAM("[DataSaver] Failed to copy intensity asset into pcdmap: "
                            << source_rel.generic_string());
        }
    };
    stage_intensity_into_pcdmap(selected_intensity_png);
    stage_intensity_into_pcdmap(selected_intensity_metadata);

    bool label_map_ready = false;
    ec.clear();
    fs::create_directories(label_map_root, ec);
    if (ec)
    {
        LOG_WARN_STREAM("[DataSaver] Failed to create label_map staging directory: " << ec.message());
    }
    else
    {
        label_map_ready = true;
        auto stage_label_item = [&](const fs::path &source_rel, const fs::path &target_name) {
            if (source_rel.empty())
            {
                return;
            }
            if (!stage_into_label_map(source_rel, target_name))
            {
                LOG_WARN_STREAM("[DataSaver] Failed to stage label_map item " << target_name.generic_string());
            }
        };

        auto stage_alias_if_exists = [&](const fs::path &relative_path) {
            if (relative_path.empty())
            {
                return;
            }
            if (!fs::exists(base / relative_path))
            {
                return;
            }
            stage_label_item(relative_path, relative_path);
        };

        fs::path label_global_source = fs::path("global_map_dense_imu.pcd");
        if (!fs::exists(base / label_global_source))
        {
            label_global_source = selected_global_map;
        }
        if (!label_global_source.empty())
        {
            const fs::path label_global_target("global_map_dense_imu.pcd");
            if (!stage_into_label_map(label_global_source, label_global_target))
            {
                LOG_WARN_STREAM("[DataSaver] Failed to stage label_map global map: " << label_global_source.generic_string());
            }
            else if (label_global_source.filename() != label_global_target)
            {
                LOG_INFO_STREAM("[DataSaver] Staged " << label_global_source.generic_string()
                                << " as label_map/" << label_global_target.generic_string());
            }
        }

        if (fs::exists(base / logs_relative("trajectory.kml")))
        {
            stage_label_item(logs_relative("trajectory.kml"), fs::path("trajectory.kml"));
        }

        fs::path trajectory_copy_source;
        const std::vector<fs::path> trajectory_candidates = {
            fs::path("pcdmap/trajectory_imu.pcd"),
            fs::path("pcdmap/traj_pcd_imu.pcd"),
            fs::path("pcdmap/traj_icp_imu.pcd"),
            fs::path("pcdmap/trajectory_lidar.pcd"),
            fs::path("pcdmap/traj_pcd_lidar.pcd"),
            fs::path("pcdmap/traj_icp.pcd"),
            fs::path("trajectory_imu.pcd"),
            fs::path("traj_pcd_imu.pcd"),
            fs::path("traj_icp_imu.pcd"),
            fs::path("trajectory_lidar.pcd"),
            fs::path("traj_pcd_lidar.pcd"),
            fs::path("traj_icp.pcd"),
            fs::path("trajectory.pcd"),
            selected_trajectory_pcd
        };
        for (const auto &candidate : trajectory_candidates)
        {
            if (!candidate.empty() && fs::exists(base / candidate))
            {
                trajectory_copy_source = candidate;
                break;
            }
        }
        if (!trajectory_copy_source.empty())
        {
            // Keep original trajectory filename in both label_map and pcdmap
            fs::path trajectory_filename = trajectory_copy_source.filename();
            stage_label_item(trajectory_copy_source, trajectory_filename);
            // Also copy trajectory to pcdmap
            if (!copy_into_root(base / trajectory_copy_source, pcdmap_root, trajectory_filename))
            {
                LOG_WARN_STREAM("[DataSaver] Failed to copy trajectory into pcdmap: "
                                << trajectory_copy_source.generic_string());
            }
        }

        if (!selected_origin_cfg.empty())
        {
            stage_label_item(selected_origin_cfg, fs::path("origin_position.cfg"));
        }

        if (!selected_intensity_png.empty())
        {
            stage_label_item(selected_intensity_png, selected_intensity_png.filename());
        }
        if (!selected_intensity_metadata.empty())
        {
            stage_label_item(selected_intensity_metadata, selected_intensity_metadata.filename());
        }
        if (!selected_intensity_xz_png.empty())
        {
            stage_label_item(selected_intensity_xz_png, selected_intensity_xz_png.filename());
        }
        if (!selected_intensity_xz_metadata.empty())
        {
            stage_label_item(selected_intensity_xz_metadata, selected_intensity_xz_metadata.filename());
        }
        if (!selected_intensity_yz_png.empty())
        {
            stage_label_item(selected_intensity_yz_png, selected_intensity_yz_png.filename());
        }
        if (!selected_intensity_yz_metadata.empty())
        {
            stage_label_item(selected_intensity_yz_metadata, selected_intensity_yz_metadata.filename());
        }
        if (!sequence_name.empty())
        {
            stage_alias_if_exists(fs::path(sequence_name + ".png"));
            stage_alias_if_exists(fs::path(sequence_name + "_metadata.txt"));
            stage_alias_if_exists(fs::path(sequence_name + "_xz.png"));
            stage_alias_if_exists(fs::path(sequence_name + "_xz_metadata.txt"));
            stage_alias_if_exists(fs::path(sequence_name + "_yz.png"));
            stage_alias_if_exists(fs::path(sequence_name + "_yz_metadata.txt"));
        }
    }

    double total_runtime_seconds = 0.0;
    bool total_runtime_available = compute_duration_seconds(
        std::vector<fs::path>{logs_relative("data_time.txt"), logs_relative("times.txt")},
        total_runtime_seconds);
    if (!total_runtime_available)
    {
        LOG_WARN_STREAM("[DataSaver] Failed to compute rosbag duration from "
                        << logs_relative("data_time.txt").generic_string()
                        << " or " << logs_relative("times.txt").generic_string() << ".");
        total_runtime_seconds = 0.0;
    }

    double mapping_distance_meters = 0.0;
    bool mapping_distance_available = compute_mapping_distance(selected_optimized_path, mapping_distance_meters);
    if (!mapping_distance_available)
    {
        mapping_distance_meters = 0.0;
    }

    bool mapping_ready = false;
    fs::path mapping_root = staging_root / "mapping";
    fs::path mapping_keyframes_root = mapping_root / "keyframes";
    fs::path base_mapping_root = base / "mapping";

    if (calibration_source_available && fs::is_directory(base_mapping_root))
    {
        copy_calibration_file(base_mapping_root / "calibration_param.yaml");
    }

    // Always create a fresh mapping directory in staging.
    // Do not reuse the potentially incomplete mapping directory from base.
    ec.clear();
    fs::remove_all(mapping_root, ec);
    ec.clear();
    fs::create_directories(mapping_keyframes_root, ec);
    if (ec)
    {
        LOG_WARN_STREAM("[DataSaver] Failed to create mapping/keyframes directory: "
                        << mapping_keyframes_root.generic_string() << ", " << ec.message());
        mapping_ready = false;
    }
    else
    {
        mapping_ready = true;
        LOG_INFO_STREAM("[DataSaver] Created fresh mapping directory for archive staging.");
    }

    if (mapping_ready)
    {
        std::error_code ensure_ec;
        fs::create_directories(mapping_keyframes_root, ensure_ec);
        if (ensure_ec)
        {
            LOG_WARN_STREAM("[DataSaver] Failed to ensure mapping/keyframes directory: "
                            << mapping_keyframes_root.generic_string() << ", " << ensure_ec.message());
        }
    }

    if (mapping_ready && save_key_frame)
    {
        fs::path base_keyframe_bin = base_mapping_root / "keyframes" / "keyframe_top_front.bin";
        std::error_code copy_ec;
        if (fs::exists(base_keyframe_bin))
        {
            fs::path staged_keyframe_bin = mapping_keyframes_root / "keyframe_top_front.bin";
            fs::copy_file(base_keyframe_bin, staged_keyframe_bin,
                          fs::copy_options::overwrite_existing, copy_ec);
            if (copy_ec)
            {
                LOG_WARN_STREAM("[DataSaver] Failed to copy existing keyframe bin from "
                                << base_keyframe_bin.generic_string() << " to "
                                << staged_keyframe_bin.generic_string() << ": " << copy_ec.message());
            }
            else
            {
                LOG_INFO_STREAM("[DataSaver] Reused existing keyframe bin at "
                                << staged_keyframe_bin.generic_string());
            }
        }
        else
        {
            fs::path keyframe_dir = base / "key_point_frame";
            std::error_code keyframe_ec;
            if (fs::is_directory(keyframe_dir, keyframe_ec))
            {
                std::vector<fs::path> keyframe_sources;
                for (fs::directory_iterator it(keyframe_dir, keyframe_ec), end; !keyframe_ec && it != end; ++it)
                {
                    if (!it->is_regular_file())
                    {
                        continue;
                    }
                    if (it->path().extension() != ".pcd")
                    {
                        continue;
                    }
                    keyframe_sources.emplace_back(it->path());
                }
                if (keyframe_ec)
                {
                    LOG_WARN_STREAM("[DataSaver] Failed to enumerate keyframe directory: "
                                    << keyframe_ec.message());
                }
                if (!keyframe_sources.empty())
                {
                    std::sort(keyframe_sources.begin(), keyframe_sources.end(),
                              [](const fs::path &lhs, const fs::path &rhs) {
                                  return lhs.filename().string() < rhs.filename().string();
                              });
                    fs::path keyframe_bin_path = mapping_keyframes_root / "keyframe_top_front.bin";
                    if (!write_keyframe_bin(keyframe_bin_path, keyframe_sources))
                    {
                        LOG_WARN_STREAM("[DataSaver] Failed to generate keyframe_top_front.bin.");
                    }
                }
                else
                {
                    LOG_WARN_STREAM("[DataSaver] No keyframe PCD files found under "
                                    << keyframe_dir.generic_string());
                }
            }
            else
            {
                LOG_WARN_STREAM("[DataSaver] Keyframe directory missing: "
                                << keyframe_dir.generic_string());
            }
        }
    }

    if (mapping_ready && !selected_runtime_params.empty())
    {
        if (!copy_into_root(base / selected_runtime_params, mapping_root, fs::path("runtime_params.yaml")))
        {
            LOG_WARN_STREAM("[DataSaver] Failed to stage runtime_params.yaml into mapping directory.");
        }
    }

    if (mapping_ready && calibration_source_available)
    {
        copy_calibration_file(mapping_root / "calibration_param.yaml");
    }

    if (mapping_ready && !selected_optimized_path.empty())
    {
        const std::string optimized_name = selected_optimized_path.filename().string();
        const bool is_tum_format = optimized_name.size() >= 8 &&
                                   optimized_name.compare(optimized_name.size() - 8, 8, "_tum.txt") == 0;
        if (is_tum_format)
        {
            fs::path imu_csv_destination = mapping_root / "imu_poses.csv";
            WriteImuPosesCsv(base / selected_optimized_path, imu_csv_destination);
        }
        else
        {
            LOG_WARN_STREAM("[DataSaver] Skip imu_poses.csv export for non-TUM trajectory: "
                            << selected_optimized_path.generic_string());
        }
    }

    if (mapping_ready)
    {
        std::error_code clear_ec;
        fs::remove_all(base_mapping_root, clear_ec);
        std::error_code sync_ec;
        fs::create_directories(base_mapping_root, sync_ec);
        if (sync_ec)
        {
            LOG_WARN_STREAM("[DataSaver] Failed to prepare root mapping directory: "
                            << base_mapping_root.generic_string() << ", " << sync_ec.message());
        }
        else
        {
            // Copy contents of mapping_root to base_mapping_root
            // Note: fs::copy(dir_a, dir_b, recursive) creates dir_b/dir_a, not what we want
            // So we need to copy the contents individually
            sync_ec.clear();
            bool copy_success = true;
            for (fs::directory_iterator it(mapping_root, sync_ec), end; !sync_ec && it != end; ++it)
            {
                const fs::path &source_item = it->path();
                fs::path dest_item = base_mapping_root / source_item.filename();

                std::error_code item_ec;
                if (fs::is_directory(source_item))
                {
                    fs::copy(source_item, dest_item,
                             fs::copy_options::recursive | fs::copy_options::overwrite_existing, item_ec);
                }
                else
                {
                    fs::copy_file(source_item, dest_item,
                                  fs::copy_options::overwrite_existing, item_ec);
                }

                if (item_ec)
                {
                    LOG_WARN_STREAM("[DataSaver] Failed to copy " << source_item.filename().string()
                                    << " to root mapping: " << item_ec.message());
                    copy_success = false;
                }
            }

            if (sync_ec)
            {
                LOG_WARN_STREAM("[DataSaver] Failed to iterate staging mapping directory: "
                                << sync_ec.message());
            }
            else if (copy_success)
            {
                LOG_INFO_STREAM("[DataSaver] Root mapping directory synchronized at "
                                << base_mapping_root.generic_string());
            }
            else
            {
                LOG_WARN_STREAM("[DataSaver] Some items failed to copy to root mapping directory.");
            }
        }
    }

    bool logs_ready = false;
    fs::path logs_source = base / logs_dir_name;
    if (fs::is_directory(logs_source))
    {
        if (copy_into_root(logs_source, staging_root, fs::path(logs_dir_name)))
        {
            logs_ready = true;
        }
        else
        {
            LOG_WARN_STREAM("[DataSaver] Failed to stage logs directory.");
        }
    }

    fs::path sensor_data_source = base / "sensor_data";
    if (fs::is_directory(sensor_data_source))
    {
        if (!copy_into_root(sensor_data_source, staging_root, fs::path("sensor_data")))
        {
            LOG_WARN_STREAM("[DataSaver] Failed to stage sensor_data directory.");
        }
    }
    // Ensure pcdmap 内不会残留 sensor_data 子目录
    {
        fs::path staged_pcdmap_sensor_dir = staging_root / "pcdmap" / "sensor_data";
        std::error_code remove_ec;
        fs::remove_all(staged_pcdmap_sensor_dir, remove_ec);
        if (!remove_ec && fs::exists(staged_pcdmap_sensor_dir))
        {
            LOG_WARN_STREAM("[DataSaver] Unable to remove redundant pcdmap/sensor_data directory.");
        }
    }

    bool mapping_json_ready = false;

    auto format_iso8601 = [](std::time_t timestamp) -> std::string {
        std::tm local_time{};
#if defined(_GNU_SOURCE) || defined(__APPLE__)
        localtime_r(&timestamp, &local_time);
#else
        std::tm *tmp = std::localtime(&timestamp);
        if (tmp)
        {
            local_time = *tmp;
        }
#endif
        char buffer[64] = {0};
        if (std::strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%S%z", &local_time) == 0)
        {
            return std::string();
        }
        std::string iso(buffer);
        if (iso.size() >= 5)
        {
            iso.insert(iso.size() - 2, ":");
        }
        return iso;
    };

    std::string create_time_iso = format_iso8601(now);

    auto format_ctime_like = [](std::time_t timestamp) -> std::string {
    std::tm local_time{};
#if defined(_GNU_SOURCE) || defined(__APPLE__)
    localtime_r(&timestamp, &local_time);
#else
    std::tm *tmp = std::localtime(&timestamp);
    if (tmp)
    {
        local_time = *tmp;
    }
#endif
    std::ostringstream oss;
    oss << std::put_time(&local_time, "%a %b %e %H:%M:%S %Y");
    return oss.str();
    };

    std::string create_time_ctime = format_ctime_like(now);

    auto json_escape = [](const std::string &input) -> std::string {
        std::string output;
        output.reserve(input.size() + 4);
        for (unsigned char ch : input)
        {
            switch (ch)
            {
            case '\"':
                output += "\\\"";
                break;
            case '\\':
                output += "\\\\";
                break;
            case '\b':
                output += "\\b";
                break;
            case '\f':
                output += "\\f";
                break;
            case '\n':
                output += "\\n";
                break;
            case '\r':
                output += "\\r";
                break;
            case '\t':
                output += "\\t";
                break;
            default:
                if (ch < 0x20)
                {
                    char buffer[7];
                    std::snprintf(buffer, sizeof(buffer), "\\u%04x", static_cast<unsigned>(ch));
                    output += buffer;
                }
                else
                {
                    output.push_back(static_cast<char>(ch));
                }
                break;
            }
        }
        return output;
    };

    auto format_compact_time = [](std::time_t timestamp) -> std::string {
        std::tm local_time{};
#if defined(_GNU_SOURCE) || defined(__APPLE__)
        localtime_r(&timestamp, &local_time);
#else
        std::tm *tmp = std::localtime(&timestamp);
        if (tmp)
        {
            local_time = *tmp;
        }
#endif
        char buffer[32] = {0};
        if (std::strftime(buffer, sizeof(buffer), "%Y%m%d%H%M", &local_time) == 0)
        {
            return std::string();
        }
        return std::string(buffer);
    };

    std::string metadata_version_base = format_compact_time(now);
    if (metadata_version_base.empty())
    {
        metadata_version_base = "000000000000";
    }
    // version格式: YYYYMMDDHHmm_0 (不使用 useMultiMode，统一使用 _0)
    std::string metadata_version = metadata_version_base + "_0";

    double origin_latitude = 0.0;
    double origin_longitude = 0.0;
    double origin_altitude = 0.0;
    bool origin_valid = false;
    if (!selected_origin_cfg.empty())
    {
        fs::path origin_path = base / selected_origin_cfg;
        std::ifstream origin_stream(origin_path);
        if (!origin_stream.is_open())
        {
            LOG_WARN_STREAM("[DataSaver] Failed to open origin_position.cfg for version metadata: "
                            << origin_path.generic_string());
        }
        else
        {
            std::string origin_content((std::istreambuf_iterator<char>(origin_stream)),
                                       std::istreambuf_iterator<char>());
            origin_stream.close();
            bool ok_lat = ExtractNumericField(origin_content, "Lat", origin_latitude);
            bool ok_lon = ExtractNumericField(origin_content, "Lon", origin_longitude);
            bool ok_height = ExtractNumericField(origin_content, "Height", origin_altitude);
            origin_valid = ok_lat && ok_lon && ok_height;
            if (!origin_valid)
            {
                LOG_WARN_STREAM("[DataSaver] origin_position.cfg missing fields for version metadata: "
                                << origin_path.generic_string());
            }
        }
    }
    auto read_metadata_param = [](const std::string &name, std::string &out_value) {
        XmlRpc::XmlRpcValue value;
        if (!ros::param::get(name, value))
        {
            return false;
        }
        return XmlRpcValueToString(value, out_value);
    };

    auto read_metadata_hierarchy = [&](const std::string &base_name, std::string &out_value) {
        if (read_metadata_param("/ms_mapping/" + base_name, out_value))
        {
            return true;
        }
        if (read_metadata_param("~" + base_name, out_value))
        {
            return true;
        }
        if (read_metadata_param(base_name, out_value))
        {
            return true;
        }
        return false;
    };

    auto getenv_or_empty = [](const char *key) -> std::string {
        const char *value = std::getenv(key);
        if (value == nullptr)
        {
            return std::string();
        }
        return std::string(value);
    };

    // 从ROS参数服务器读取项目元数据，如缺失则尝试环境变量
    std::string project_id;
    std::string area_id;
    std::string project_name;
    std::string area_name;

    if (!read_metadata_hierarchy("project_id", project_id))
    {
        project_id = getenv_or_empty("MS_MAPPING_PROJECT_ID");
    }
    if (!read_metadata_hierarchy("area_id", area_id))
    {
        area_id = getenv_or_empty("MS_MAPPING_AREA_ID");
    }
    if (!read_metadata_hierarchy("project_name", project_name))
    {
        project_name = getenv_or_empty("MS_MAPPING_PROJECT_NAME");
    }
    if (!read_metadata_hierarchy("area_name", area_name))
    {
        area_name = getenv_or_empty("MS_MAPPING_AREA_NAME");
    }

    const std::string trimmed_project_id = trim_copy(project_id);
    const std::string trimmed_area_id = trim_copy(area_id);
    const std::string trimmed_project_name = trim_copy(project_name);
    const std::string trimmed_area_name = trim_copy(area_name);

    LOG_INFO_STREAM("[DataSaver] Project metadata: project_id=" << trimmed_project_id
                   << ", area_id=" << trimmed_area_id
                   << ", project_name=" << trimmed_project_name
                   << ", area_name=" << trimmed_area_name);

    auto ensure_metadata_not_empty = [&](const std::string &label, const std::string &value) {
        if (!value.empty())
        {
            return;
        }
        LOG_ERROR_STREAM("[DataSaver] Missing required metadata field: " << label
                          << ". 请在启动参数或环境变量中配置该字段。");
        throw std::runtime_error("metadata field missing");
    };

    try
    {
        ensure_metadata_not_empty("project_id", trimmed_project_id);
        ensure_metadata_not_empty("area_id", trimmed_area_id);
        ensure_metadata_not_empty("project_name", trimmed_project_name);
        ensure_metadata_not_empty("area_name", trimmed_area_name);
    }
    catch (const std::exception &)
    {
        fs::remove_all(staging_root, ec);
        return;
    }

    fs::path version_json_path = pcdmap_root / "version.json";
    std::ofstream version_ofs(version_json_path.string(), std::ios::trunc);
    if (!version_ofs.is_open())
    {
        LOG_WARN_STREAM("[DataSaver] Failed to create version.json in staging directory: "
                        << version_json_path.generic_string());
    }
    else
    {
        version_ofs << std::setprecision(15);
        version_ofs << "{\n";
        version_ofs << "  \"area_id\": \"" << json_escape(trimmed_area_id) << "\",\n";
        version_ofs << "  \"area_name\": \"" << json_escape(trimmed_area_name) << "\",\n";
        version_ofs << "  \"create_time\": \"" << create_time_iso << "\",\n";
        version_ofs << "  \"origin_position\": {\n";
        if (origin_valid)
        {
            version_ofs << "    \"altitude\": " << origin_altitude << ",\n";
            version_ofs << "    \"latitude\": " << origin_latitude << ",\n";
            version_ofs << "    \"longitude\": " << origin_longitude << "\n";
        }
        else
        {
            version_ofs << "    \"altitude\": null,\n";
            version_ofs << "    \"latitude\": null,\n";
            version_ofs << "    \"longitude\": null\n";
        }
        version_ofs << "  },\n";
        version_ofs << "  \"parent_map\": {\n";
        version_ofs << "    \"area_id\": \"" << json_escape(trimmed_area_id) << "\",\n";
        version_ofs << "    \"project_id\": \"" << json_escape(trimmed_project_id) << "\"\n";
        version_ofs << "  },\n";
        version_ofs << "  \"project_id\": \"" << json_escape(trimmed_project_id) << "\",\n";
        version_ofs << "  \"project_name\": \"" << json_escape(trimmed_project_name) << "\",\n";
        version_ofs << "  \"version\": \"" << json_escape(metadata_version) << "\"\n";
        version_ofs << "}\n";
        version_ofs.close();
    }

    fs::path mapping_json_path = staging_root / "mapping.json";
    std::ofstream mapping_json_stream(mapping_json_path.string(), std::ios::trunc);
    if (!mapping_json_stream.is_open())
    {
        LOG_WARN_STREAM("[DataSaver] Failed to create mapping.json in staging directory: "
                        << mapping_json_path.generic_string());
    }
    else
    {
        mapping_json_stream << std::setprecision(15);
        mapping_json_stream << "{\n";
        mapping_json_stream << "  \"area_id\": \"" << json_escape(trimmed_area_id) << "\",\n";
        mapping_json_stream << "  \"area_name\": \"" << json_escape(trimmed_area_name) << "\",\n";
        mapping_json_stream << "  \"create_time\": \"" << json_escape(create_time_iso) << "\",\n";
        mapping_json_stream << "  \"map_dynamic_obstract_removal\": {\n";
        mapping_json_stream << "    \"enabled\": 0\n";
        mapping_json_stream << "  },\n";
        mapping_json_stream << "  \"map_refine\": {\n";
        mapping_json_stream << "  \"enabled\": 0\n";
        mapping_json_stream << "  },\n";
        mapping_json_stream << "  \"mapping_distance\": " << mapping_distance_meters << ",\n";
        mapping_json_stream << "  \"mapping_mode\": " << (useMultiMode ? 1 : 0) << ",\n";
        mapping_json_stream << "  \"parent_map\": {\n";
        mapping_json_stream << "    \"area_id\": \"" << json_escape(trimmed_area_id) << "\",\n";
        mapping_json_stream << "    \"project_id\": \"" << json_escape(trimmed_project_id) << "\"\n";
        mapping_json_stream << "  },\n";
        mapping_json_stream << "  \"project_id\": \"" << json_escape(trimmed_project_id) << "\",\n";
        mapping_json_stream << "  \"project_name\": \"" << json_escape(trimmed_project_name) << "\",\n";
        mapping_json_stream << "  \"total_runtime\": " << total_runtime_seconds << ",\n";
        mapping_json_stream << "  \"version\": \"" << json_escape(metadata_version) << "\"\n";
        mapping_json_stream << "}\n";
        mapping_json_stream.close();
        mapping_json_ready = true;

        // Also copy mapping.json to the root directory
        fs::path root_mapping_json = base / "mapping.json";
        std::error_code copy_ec;
        fs::copy_file(mapping_json_path, root_mapping_json, fs::copy_options::overwrite_existing, copy_ec);
        if (copy_ec)
        {
            LOG_WARN_STREAM("[DataSaver] Failed to copy mapping.json to root directory: " << copy_ec.message());
        }
        else
        {
            LOG_INFO_STREAM("[DataSaver] Saved mapping.json to root directory: " << root_mapping_json.generic_string());
        }
    }

    fs::path readme_path = staging_root / "README.txt";
    std::ofstream readme_ofs(readme_path.string(), std::ios::trunc);
    if (!readme_ofs.is_open())
    {
        LOG_WARN_STREAM("[DataSaver] Failed to create README.txt in staging directory: " << readme_path.generic_string());
    }
    else
    {
        readme_ofs << "归档目录结构说明\n";
        readme_ofs << "\n";
        readme_ofs << "├─pcdmap/....................直接替换车端 pcdmap\n";
        readme_ofs << "│  ├─maps/...................定位栅格/分块地图\n";
        readme_ofs << "│  ├─global_map_dense_*.pcd..稠密点云地图 (imu/lidar)\n";
        readme_ofs << "│  ├─trajectory_*.pcd........定位轨迹，命名需与终端配置一致\n";
        readme_ofs << "│  ├─origin_position.cfg.....GNSS 原点信息 (JSON)\n";
        readme_ofs << "│  ├─*_intensity*.png........强度俯视/侧视图\n";
        readme_ofs << "│  ├─*_metadata.txt..........强度图说明\n";
        readme_ofs << "│  └─version.json............项目/区域/原点元数据\n";
        readme_ofs << "├─mapping/.....................建图配置与关键数据（与压缩包一致）\n";
        readme_ofs << "│  ├─runtime_params.yaml.......保存时的参数快照\n";
        readme_ofs << "│  ├─keyframes/................关键帧集合：keyframe_top_front.bin\n";
        readme_ofs << "│  ├─imu_poses.csv.............IMU 轨迹 (供 BEV / 投影使用)\n";
        readme_ofs << "│  ├─calibration_param.yaml....保存时使用的外参 (若可用)\n";
        readme_ofs << "│  └─mapping.json..............建图统计与模式信息\n";
        readme_ofs << "├─" << logs_dir_name << "/........................运行日志与调试数据\n";
        readme_ofs << "│  ├─runtime_log_*.txt.........运行期日志\n";
        readme_ofs << "│  ├─odom_tum.txt..............里程计轨迹 (TUM)\n";
        readme_ofs << "│  ├─pose_graph.g2o............位姿图 (g2o)\n";
        readme_ofs << "│  ├─pose_graph.png............位姿图可视化图片\n";
        readme_ofs << "│  ├─pose_graph_3d_result.txt..位姿图优化结果\n";
        readme_ofs << "│  └─trajectory.kml............建图轨迹 (Google Earth 可视)\n";
        readme_ofs << "├─sensor_data/.................传感器 bin 数据（默认空，可按需放置）\n";
        readme_ofs << "├─mapping.json.................与 mapping/ 内版本一致的顶层副本\n";
        readme_ofs << "└─label_map/...................标注使用的数据副本（点云/轨迹/强度图）\n";
        readme_ofs << "\n";
        readme_ofs << "说明：\n";
        readme_ofs << "1. 将 pcdmap/ 整体拷贝至车端即可使用；trajectory/origin 文件名需与运行配置匹配。\n";
        readme_ofs << "2. mapping/ 内的 keyframes 与 imu_poses.csv 可直接用于回放、标注或 BEV 投影。\n";
        readme_ofs << "3. logs/ 目录提供 g2o、TUM、KML 等调试/评估文件，便于闭环或问题复现。\n";
        readme_ofs << "4. 若更新 runtime_params.yaml 或 origin_position.cfg，请重新调用保存服务以同步版本。\n";
        readme_ofs.close();
        std::error_code readme_copy_ec;
        fs::copy_file(readme_path, base / "README.txt", fs::copy_options::overwrite_existing, readme_copy_ec);
        if (readme_copy_ec)
        {
            LOG_WARN_STREAM("[DataSaver] Failed to copy README.txt to root directory: " << readme_copy_ec.message());
        }
        else
        {
            LOG_INFO_STREAM("[DataSaver] Saved README.txt to root directory.");
        }
    }

    // 构建文件夹名称 - 完全按照 update_archive_metadata.py 的逻辑
    std::string folder_name;

    // Sanitize component function (与 update_archive_metadata.py 保持一致)
    auto sanitize_component = [](const std::string &value) -> std::string {
        std::string sanitized = value;
        for (char &c : sanitized)
        {
            if (c == '/' || c == '\\')
            {
                c = '_';
            }
            else if (c == '\n' || c == '\r')
            {
                c = '_';
            }
        }
        std::size_t start = 0;
        while (start < sanitized.size() && (sanitized[start] == '_' || std::isspace(static_cast<unsigned char>(sanitized[start]))))
        {
            ++start;
        }
        std::size_t end = sanitized.size();
        while (end > start && (sanitized[end - 1] == '_' || std::isspace(static_cast<unsigned char>(sanitized[end - 1]))))
        {
            --end;
        }
        sanitized = sanitized.substr(start, end - start);
        if (sanitized.empty())
        {
            throw std::runtime_error("Name component cannot be empty after sanitization.");
        }
        return sanitized;
    };

    try
    {
        // 标准格式: {project_id}_{area_id}_{project_name}_{area_name}_{version}
        std::string safe_project_id = sanitize_component(trimmed_project_id);
        std::string safe_area_id = sanitize_component(trimmed_area_id);
        std::string safe_project_name = sanitize_component(trimmed_project_name);
        std::string safe_area_name = sanitize_component(trimmed_area_name);
        std::string safe_version = sanitize_component(metadata_version);

        const std::string pcdmap_asset_prefix = safe_project_id + "_" + safe_area_id + "_" + safe_version;
        folder_name = safe_project_id + "_" + safe_area_id + "_" +
                      safe_project_name + "_" + safe_area_name + "_" + safe_version;
        LOG_INFO_STREAM("[DataSaver] Using project metadata for archive naming: " << folder_name);

        auto rename_sequence_pcdmap_assets = [&]() {
            if (sequence_name.empty())
            {
                return;
            }
            if (sequence_name == folder_name)
            {
                return;
            }
            auto rename_asset = [&](const std::string &suffix, const std::string &ext) {
                fs::path old_path = base / "pcdmap" / (sequence_name + suffix + ext);
                if (!fs::exists(old_path))
                {
                    return;
                }
                fs::path new_path = base / "pcdmap" / (pcdmap_asset_prefix + suffix + ext);
                std::error_code eq_ec;
                if (fs::exists(new_path, eq_ec))
                {
                    std::error_code equivalent_ec;
                    if (fs::equivalent(old_path, new_path, equivalent_ec) && !equivalent_ec)
                    {
                        return;
                    }
                }
                std::error_code rename_ec;
                fs::create_directories(new_path.parent_path(), rename_ec);
                rename_ec.clear();
                fs::rename(old_path, new_path, rename_ec);
                if (rename_ec)
                {
                    std::error_code copy_ec;
                    fs::copy_file(old_path, new_path, fs::copy_options::overwrite_existing, copy_ec);
                    if (copy_ec)
                    {
                        LOG_WARN_STREAM("[DataSaver] Failed to rename "
                                        << old_path.generic_string() << " -> " << new_path.generic_string()
                                        << ": " << rename_ec.message() << ", copy_err=" << copy_ec.message());
                        return;
                    }
                    std::error_code remove_ec;
                    fs::remove(old_path, remove_ec);
                }
                LOG_INFO_STREAM("[DataSaver] Renamed pcdmap asset "
                                << old_path.filename().generic_string() << " -> "
                                << new_path.filename().generic_string());
            };

            const std::vector<std::string> png_suffixes = {
                "",
                "_intensity",
                "_xz",
                "_intensity_xz",
                "_yz",
                "_intensity_yz"
            };
            for (const auto &suffix : png_suffixes)
            {
                rename_asset(suffix, ".png");
            }
            const std::vector<std::string> meta_suffixes = {
                "_metadata",
                "_intensity_metadata",
                "_xz_metadata",
                "_intensity_xz_metadata",
                "_yz_metadata",
                "_intensity_yz_metadata"
            };
            for (const auto &suffix : meta_suffixes)
            {
                rename_asset(suffix, ".txt");
            }
        };

        rename_sequence_pcdmap_assets();
    }
    catch (const std::exception &ex)
    {
        LOG_ERROR_STREAM("[DataSaver] Failed to build archive name: " << ex.what());
        fs::remove_all(staging_root, ec);
        return;
    }

    // 压缩包文件名与文件夹名称一致
    std::string archive_name = folder_name + ".tar.gz";
    fs::path archive_path = base / archive_name;

    fs::path named_folder = staging_root / folder_name;
    ec.clear();
    fs::create_directories(named_folder, ec);
    if (ec)
    {
        LOG_ERROR_STREAM("[DataSaver] Failed to create named folder in staging: " << ec.message());
    }
    else
    {
        LOG_INFO_STREAM("[DataSaver] Created archive folder: " << folder_name);

        // 将所有内容移到命名文件夹中
        auto move_if_exists = [&](const std::string &dir_name) {
            fs::path src = staging_root / dir_name;
            if (fs::exists(src))
            {
                fs::path dst = named_folder / dir_name;
                std::error_code move_ec;
                fs::rename(src, dst, move_ec);
                if (move_ec)
                {
                    LOG_WARN_STREAM("[DataSaver] Failed to move " << dir_name << " into named folder: " << move_ec.message());
                    // 尝试复制并删除
                    fs::copy(src, dst, fs::copy_options::recursive | fs::copy_options::overwrite_existing, move_ec);
                    if (!move_ec)
                    {
                        fs::remove_all(src, move_ec);
                    }
                }
            }
        };

        move_if_exists("pcdmap");
        move_if_exists("label_map");
        move_if_exists("mapping");
        move_if_exists("sensor_data");
        move_if_exists(logs_dir_name);
        
        // 移动mapping.json和README.txt
        fs::path mapping_json_src = staging_root / "mapping.json";
        fs::path readme_src = staging_root / "README.txt";
        if (fs::exists(mapping_json_src))
        {
            fs::path dst = named_folder / "mapping.json";
            std::error_code move_ec;
            fs::rename(mapping_json_src, dst, move_ec);
        }
        if (fs::exists(readme_src))
        {
            fs::path dst = named_folder / "README.txt";
            std::error_code move_ec;
            fs::rename(readme_src, dst, move_ec);
        }

        // 重命名 intensity images (参考 update_archive_metadata.py 的 _rename_intensity_assets)
        // 只在有完整项目元数据时重命名
    if (!trimmed_project_id.empty() && !trimmed_area_id.empty() && !metadata_version.empty()) {
            std::vector<std::string> suffix_patterns = {
                "_intensity_xz", "_intensity_yz", "_intensity",
                "_xz", "_yz", ""
            };

            // 构建目标文件名基础部分
            std::string target_base = sanitize_component(trimmed_project_id) + "_" +
                                     sanitize_component(trimmed_area_id) + "_" +
                                     sanitize_component(metadata_version);

            // 使用 recursive_directory_iterator 递归查找所有 PNG 文件
            std::vector<std::pair<fs::path, fs::path>> intensity_pairs;
            try {
                for (const auto& entry : fs::recursive_directory_iterator(named_folder)) {
                    if (!entry.is_regular_file()) continue;
                    if (entry.path().extension() != ".png") continue;

                    // 检查路径中是否包含 pcdmap 或 label_map
                    std::string path_str = entry.path().string();
                    bool in_pcdmap = path_str.find("/pcdmap/") != std::string::npos ||
                                     path_str.find("/pcdmap") == path_str.size() - 7;
                    bool in_label_map = path_str.find("/label_map/") != std::string::npos ||
                                        path_str.find("/label_map") == path_str.size() - 10;

                    if (!in_pcdmap && !in_label_map) continue;

                    // 检查是否有对应的 _metadata.txt 文件
                    std::string stem = entry.path().stem().string();
                    fs::path metadata_file = entry.path().parent_path() / (stem + "_metadata.txt");
                    if (fs::exists(metadata_file)) {
                        intensity_pairs.push_back({entry.path(), metadata_file});
                    }
                }
            } catch (const std::exception& e) {
                LOG_WARN_STREAM("[DataSaver] Error scanning for intensity images: " << e.what());
            }

            if (intensity_pairs.empty()) {
                LOG_INFO_STREAM("[DataSaver] No intensity images found to rename.");
            } else {
                LOG_INFO_STREAM("[DataSaver] Found " << intensity_pairs.size() << " intensity image pair(s) to rename.");
            }

            // 重命名找到的 intensity images
            int renamed_count = 0;
            for (const auto& [png_path, metadata_path] : intensity_pairs) {
                std::string stem = png_path.stem().string();
                std::string suffix = "";

                // 找到匹配的后缀（优先匹配长的）
                bool found_suffix = false;
                for (const auto& pattern : suffix_patterns) {
                    if (!pattern.empty() && stem.size() >= pattern.size() &&
                        stem.substr(stem.size() - pattern.size()) == pattern) {
                        suffix = pattern;
                        found_suffix = true;
                        break;
                    }
                }
                // 如果没有找到特定后缀，使用空后缀
                if (!found_suffix) {
                    suffix = "";
                }

                // 新文件名: {project_id}_{area_id}_{version}{suffix}.png
                std::string new_png_name = target_base + suffix + ".png";
                std::string new_meta_name = target_base + suffix + "_metadata.txt";

                fs::path new_png_path = png_path.parent_path() / new_png_name;
                fs::path new_metadata_path = metadata_path.parent_path() / new_meta_name;

                // 如果文件名已经正确，跳过
                if (png_path == new_png_path && metadata_path == new_metadata_path) {
                    continue;
                }

                std::error_code rename_ec;
                
                // 检查目标文件是否存在
                if (new_png_path != png_path && fs::exists(new_png_path)) {
                    LOG_WARN_STREAM("[DataSaver] Target PNG already exists, skipping: " << new_png_name);
                    continue;
                }
                if (new_metadata_path != metadata_path && fs::exists(new_metadata_path)) {
                    LOG_WARN_STREAM("[DataSaver] Target metadata already exists, skipping: " << new_meta_name);
                    continue;
                }

                // 重命名 PNG 文件
                if (png_path != new_png_path) {
                    fs::rename(png_path, new_png_path, rename_ec);
                    if (rename_ec) {
                        LOG_WARN_STREAM("[DataSaver] Failed to rename PNG: " << png_path.filename()
                                       << " -> " << new_png_name << ": " << rename_ec.message());
                    } else {
                        LOG_INFO_STREAM("[DataSaver] Renamed: " << png_path.filename() << " -> " << new_png_name);
                        renamed_count++;
                    }
                }

                // 重命名 metadata 文件
                if (metadata_path != new_metadata_path) {
                    fs::rename(metadata_path, new_metadata_path, rename_ec);
                    if (rename_ec) {
                        LOG_WARN_STREAM("[DataSaver] Failed to rename metadata: " << metadata_path.filename()
                                       << " -> " << new_meta_name << ": " << rename_ec.message());
                    } else {
                        LOG_INFO_STREAM("[DataSaver] Renamed: " << metadata_path.filename() << " -> " << new_meta_name);
                    }
                }
            }

            if (renamed_count > 0) {
                LOG_INFO_STREAM("[DataSaver] Successfully renamed " << renamed_count << " intensity image file(s).");
            }
        } else {
            LOG_INFO_STREAM("[DataSaver] Skipping intensity image renaming (metadata incomplete).");
        }

        // 更新 origin_position.cfg (参考 update_archive_metadata.py 的 _update_origin_position_metadata)
        fs::path origin_cfg = named_folder / "pcdmap" / "origin_position.cfg";
        if (fs::exists(origin_cfg) && !trimmed_project_name.empty() && !trimmed_area_name.empty()) {
            try {
                // 读取现有内容
                std::ifstream ifs(origin_cfg);
                std::string content((std::istreambuf_iterator<char>(ifs)),
                                   std::istreambuf_iterator<char>());
                ifs.close();

                auto normalize_component = [](const std::string &input) {
                    std::string value = input;
                    for (char &ch : value)
                    {
                        if (ch == '\n' || ch == '\r')
                        {
                            ch = ' ';
                        }
                    }
                    std::size_t begin = 0;
                    std::size_t end = value.size();
                    while (begin < end && (value[begin] == '_' || std::isspace(static_cast<unsigned char>(value[begin]))))
                    {
                        ++begin;
                    }
                    while (end > begin && (value[end - 1] == '_' || std::isspace(static_cast<unsigned char>(value[end - 1]))))
                    {
                        --end;
                    }
                    return value.substr(begin, end - begin);
                };

                auto set_json_string_value = [&](const std::string &key, const std::string &raw_value) {
                    const std::string encoded_value = json_escape(raw_value);
                    const std::string quoted_key = "\"" + key + "\"";
                    std::size_t key_pos = content.find(quoted_key);
                    if (key_pos != std::string::npos)
                    {
                        std::size_t colon_pos = content.find(':', key_pos + quoted_key.size());
                        if (colon_pos != std::string::npos)
                        {
                            std::size_t value_begin = content.find_first_not_of(" \t\r\n", colon_pos + 1);
                            if (value_begin != std::string::npos && content[value_begin] == '"')
                            {
                                std::size_t value_end = value_begin + 1;
                                bool escaped = false;
                                while (value_end < content.size())
                                {
                                    const char ch = content[value_end];
                                    if (ch == '\\' && !escaped)
                                    {
                                        escaped = true;
                                        ++value_end;
                                        continue;
                                    }
                                    if (ch == '"' && !escaped)
                                    {
                                        break;
                                    }
                                    escaped = false;
                                    ++value_end;
                                }
                                if (value_end < content.size())
                                {
                                    content.replace(value_begin, value_end - value_begin + 1,
                                                    "\"" + encoded_value + "\"");
                                    return;
                                }
                            }
                        }
                    }
                    std::size_t last_brace = content.rfind('}');
                    if (last_brace != std::string::npos)
                    {
                        std::string insertion = ",\n   \"" + key + "\": \"" + encoded_value + "\"";
                        content.insert(last_brace, insertion);
                    }
                };

                const std::string address_value = normalize_component(trimmed_project_name) + "_" +
                                                   normalize_component(trimmed_area_name);
                const std::string create_time_value = create_time_ctime + "\n";

                set_json_string_value("address", address_value);
                set_json_string_value("createTime", create_time_value);

                std::ofstream ofs(origin_cfg, std::ios::trunc);
                ofs << content;
                ofs.close();

                LOG_INFO_STREAM("[DataSaver] Updated origin_position.cfg with address/createTime");
            } catch (const std::exception& e) {
                LOG_WARN_STREAM("[DataSaver] Failed to update origin_position.cfg: " << e.what());
            }
        }
    }

    // 压缩包中包含命名文件夹
    std::string command = "tar --warning=no-file-changed --ignore-failed-read -czf " +
                          shell_quote(archive_path.string()) + " -C " + shell_quote(staging_root.string()) + " " +
                          shell_quote(folder_name);

    int ret = std::system(command.c_str());
    if (ret != 0)
    {
        LOG_ERROR_STREAM("[DataSaver] Failed to create archive (exit code " << ret << "): " << archive_path);
    }
    else
    {
        LOG_INFO_STREAM("[DataSaver] Archived outputs to " << archive_path);
        LOG_INFO_STREAM("[DataSaver] Archive folder name: " << folder_name);
    }

    ec.clear();
    fs::remove_all(staging_root, ec);
    if (ec)
    {
        LOG_WARN_STREAM("[DataSaver] Failed to clean archive staging directory: " << ec.message());
    }
}

std::string DataSaver::FindG2oFile(const std::string &directory) const
{
    namespace fs = std::filesystem;

    if (directory.empty() || !fs::exists(directory) || !fs::is_directory(directory))
    {
        return "";
    }

    // Search for .g2o files in the directory (including subdirectories like logs/)
    std::vector<std::string> candidates;

    try
    {
        for (const auto &entry : fs::recursive_directory_iterator(directory))
        {
            if (entry.is_regular_file() && entry.path().extension() == ".g2o")
            {
                candidates.push_back(entry.path().string());
            }
        }
    }
    catch (const std::exception &e)
    {
        LOG_WARN_STREAM("[DataSaver] Error while searching for .g2o files in " << directory << ": " << e.what());
        return "";
    }

    if (candidates.empty())
    {
        return "";
    }

    // Prefer files with "pose_graph" in the name
    for (const auto &candidate : candidates)
    {
        fs::path p(candidate);
        std::string filename = p.filename().string();
        if (filename.find("pose_graph") != std::string::npos)
        {
            LOG_INFO_STREAM("[DataSaver] Found g2o file: " << ColorizePath(candidate, kAnsiG2oPathColor));
            return candidate;
        }
    }

    // Return the first .g2o file found
    LOG_INFO_STREAM("[DataSaver] Found g2o file: " << ColorizePath(candidates[0], kAnsiG2oPathColor));
    return candidates[0];
}

std::string DataSaver::FindTumTrajectoryFile(const std::string &directory) const
{
    namespace fs = std::filesystem;

    if (directory.empty() || !fs::exists(directory) || !fs::is_directory(directory))
    {
        return "";
    }

    // Search for txt files containing "tum" in the filename
    std::vector<std::string> candidates;

    try
    {
        for (const auto &entry : fs::recursive_directory_iterator(directory))
        {
            if (entry.is_regular_file() && entry.path().extension() == ".txt")
            {
                std::string filename = entry.path().filename().string();
                // Look for "tum" in filename (case-insensitive)
                std::string filename_lower = filename;
                std::transform(filename_lower.begin(), filename_lower.end(), filename_lower.begin(), ::tolower);

                if (filename_lower.find("tum") != std::string::npos)
                {
                    // Verify file format by scanning for the first non-comment, non-empty line
                    std::ifstream test_stream(entry.path());
                    if (test_stream.is_open())
                    {
                        std::string line;
                        while (std::getline(test_stream, line))
                        {
                            // Skip comments and empty lines (consistent with Python merge_keyframes_with_tum.py)
                            std::string trimmed = line;
                            trimmed.erase(trimmed.begin(),
                                          std::find_if(trimmed.begin(), trimmed.end(),
                                                       [](unsigned char ch) { return !std::isspace(ch); }));
                            if (trimmed.empty() || trimmed[0] == '#')
                            {
                                continue;
                            }

                            std::istringstream iss(line);
                            double timestamp, x, y, z, qx, qy, qz, qw;
                            // Check if line has exactly 8 fields (TUM format)
                            if (iss >> timestamp >> x >> y >> z >> qx >> qy >> qz >> qw)
                            {
                                // Check that there's no extra data
                                std::string extra;
                                if (!(iss >> extra))
                                {
                                    candidates.push_back(entry.path().string());
                                }
                            }
                            // We only inspect the first meaningful line.
                            break;
                        }
                        test_stream.close();
                    }
                }
            }
        }
    }
    catch (const std::exception &e)
    {
        LOG_WARN_STREAM("[DataSaver] Error while searching for TUM trajectory files in " << directory << ": " << e.what());
        return "";
    }

    if (candidates.empty())
    {
        return "";
    }

    // Prefer files with "optimized" in the name
    for (const auto &candidate : candidates)
    {
        fs::path p(candidate);
        std::string filename = p.filename().string();
        std::string filename_lower = filename;
        std::transform(filename_lower.begin(), filename_lower.end(), filename_lower.begin(), ::tolower);

        if (filename_lower.find("optimized") != std::string::npos)
        {
            LOG_INFO_STREAM("[DataSaver] Found TUM trajectory file: " << ColorizePath(candidate, kAnsiTumPathColor));
            return candidate;
        }
    }

    // Return the first valid TUM file found
    LOG_INFO_STREAM("[DataSaver] Found TUM trajectory file: " << ColorizePath(candidates[0], kAnsiTumPathColor));
    return candidates[0];
}

std::string DataSaver::FindKeyframeBinFile(const std::string &directory) const
{
    namespace fs = std::filesystem;

    if (directory.empty() || !fs::exists(directory) || !fs::is_directory(directory))
    {
        return "";
    }

    const fs::path base(directory);

    try
    {
        for (const auto &entry : fs::recursive_directory_iterator(base))
        {
            if (!entry.is_regular_file() || entry.path().extension() != ".bin")
            {
                continue;
            }
            std::string filename_lower = entry.path().filename().string();
            std::transform(filename_lower.begin(), filename_lower.end(), filename_lower.begin(), ::tolower);
            if (filename_lower.find("keyframe") == std::string::npos)
            {
                continue;
            }
            LOG_INFO_STREAM("[DataSaver] Found keyframe bin file: " << entry.path().string());
            return entry.path().string();
        }
    }
    catch (const std::exception &e)
    {
        LOG_WARN_STREAM("[DataSaver] Error while searching for keyframe bin files in " << directory << ": " << e.what());
        return "";
    }

    LOG_WARN_STREAM("[DataSaver] No keyframe bin file found under " << directory);
    return "";
}

std::string DataSaver::FindBackendOdomFile(const std::string &directory) const
{
    namespace fs = std::filesystem;

    if (directory.empty() || !fs::exists(directory) || !fs::is_directory(directory))
    {
        return "";
    }

    const std::string target = "backend_odom_log.csv";

    try
    {
        for (const auto &entry : fs::recursive_directory_iterator(directory))
        {
            if (!entry.is_regular_file())
            {
                continue;
            }

            std::string filename = entry.path().filename().string();
            std::string lower = filename;
            std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);

            if (lower == target || lower.find("backend_odom_log") != std::string::npos)
            {
                LOG_INFO_STREAM("[DataSaver] Found BACKEND_ODOM_LOG.csv: " << entry.path().string());
                return entry.path().string();
            }
        }
    }
    catch (const std::exception &e)
    {
        LOG_WARN_STREAM("[DataSaver] Error while searching for BACKEND_ODOM_LOG.csv under "
                        << directory << ": " << e.what());
    }

    return "";
}

bool DataSaver::LoadBackendOdomLog(
    const std::string &file_path,
    std::vector<Pose6D, Eigen::aligned_allocator<Pose6D>> &trajectory_out) const
{
    trajectory_out.clear();
    if (file_path.empty())
    {
        return false;
    }

    std::ifstream stream(file_path);
    if (!stream.is_open())
    {
        LOG_WARN_STREAM("[DataSaver] Failed to open BACKEND_ODOM_LOG.csv: " << file_path);
        return false;
    }

    std::string line;
    std::size_t line_number = 0;
    std::size_t skipped = 0;
    bool angles_in_degrees = false;
    bool angle_unit_decided = false;
    constexpr double kDegToRad = 0.0174532925199432957692369077;
    std::string first_bad_line;

    while (std::getline(stream, line))
    {
        ++line_number;
        if (line.empty())
        {
            continue;
        }

        std::replace(line.begin(), line.end(), ',', ' ');
        std::istringstream iss(line);
        std::vector<std::string> tokens;
        std::string token;
        while (iss >> token)
        {
            tokens.push_back(token);
        }
        if (tokens.size() < 7)
        {
            ++skipped;
            if (first_bad_line.empty())
            {
                first_bad_line = line;
            }
            continue;
        }

        // 兼容格式：可能前面有 system_time 等多余字段，取最后 7 个字段为
        // timestamp x y z roll pitch yaw
        const std::size_t start = tokens.size() - 7;
        double values[7] = {0.0};
        bool parse_ok = true;
        for (std::size_t i = 0; i < 7; ++i)
        {
            try
            {
                values[i] = std::stod(tokens[start + i]);
            }
            catch (const std::exception &)
            {
                parse_ok = false;
                break;
            }
        }
        if (!parse_ok)
        {
            ++skipped;
            if (first_bad_line.empty())
            {
                first_bad_line = line;
            }
            continue;
        }

        const double timestamp = values[0];
        double x = values[1];
        double y = values[2];
        double z = values[3];
        double roll = values[4];
        double pitch = values[5];
        double yaw = values[6];

        if (!angle_unit_decided)
        {
            const double max_angle = std::max({std::abs(roll), std::abs(pitch), std::abs(yaw)});
            if (max_angle > 6.4)
            {
                angles_in_degrees = true;
            }
            angle_unit_decided = true;
        }

        static_cast<void>(timestamp);

        if (angles_in_degrees)
        {
            roll = roll * kDegToRad;
            pitch = pitch * kDegToRad;
            yaw = yaw * kDegToRad;
        }

        Pose6D pose;
        pose.x = x;
        pose.y = y;
        pose.z = z;
        pose.setRPY(roll, pitch, yaw);
        pose.valid = true;
        trajectory_out.push_back(pose);
    }

    if (trajectory_out.empty())
    {
        LOG_WARN_STREAM("[DataSaver] BACKEND_ODOM_LOG.csv contained no valid poses: " << file_path
                        << (first_bad_line.empty() ? "" : (", first bad line: \"" + first_bad_line + "\"")));
        return false;
    }

    if (skipped > 0)
    {
        LOG_WARN_STREAM("[DataSaver] Skipped " << skipped << " malformed lines while reading "
                        << file_path);
    }

    LOG_INFO_STREAM("[DataSaver] Loaded " << trajectory_out.size()
                    << " poses from BACKEND_ODOM_LOG.csv"
                    << (angles_in_degrees ? " (angles converted from degrees)" : "")
                    << ": " << file_path);
    return true;
}

bool DataSaver::LoadGlobalMapFromBin(const std::string &bin_file, pcl::PointCloud<PointT>::Ptr globalmap_ptr)
{
    if (bin_file.empty() || !globalmap_ptr)
    {
        LOG_WARN_STREAM("[DataSaver] LoadGlobalMapFromBin called with empty path or null cloud");
        return false;
    }

    namespace fs = std::filesystem;
    if (!fs::exists(bin_file))
    {
        LOG_WARN_STREAM("[DataSaver] Bin file does not exist: " << bin_file);
        return false;
    }

    // First try new FrameWithOutRT + Zstd format
    {
        std::ifstream bin_stream(bin_file, std::ios::binary);
        if (!bin_stream.is_open())
        {
            LOG_ERROR_STREAM("[DataSaver] Failed to open bin file: " << bin_file);
            return false;
        }

        std::vector<FrameWithOutRT> frames;
        CompressModule::LoadFromAllBin(bin_stream, frames);
        bin_stream.close();

        if (!frames.empty())
        {
            LOG_INFO_STREAM("[DataSaver] Detected FrameWithOutRT keyframe bin format: frames=" << frames.size()
                            << ", file=" << bin_file);

            globalmap_ptr->clear();
            CompressModule decompressor;

            std::size_t total_points = 0;
            for (const auto &frame : frames)
            {
                // 这里 CompressModule 接口使用 std::shared_ptr，与 PCL 本身无交互，不涉及跨库兼容问题。
                std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> cloud(new pcl::PointCloud<pcl::PointXYZI>());
                double ts = 0.0;
                Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
                decompressor.DecodeWithOutRt(frame, cloud, ts, pose);

                if (!cloud || cloud->empty())
                {
                    continue;
                }

                pcl::PointCloud<PointT> transformed;
                Eigen::Matrix4f pose_f = pose.cast<float>();
                pcl::transformPointCloud(*cloud, transformed, pose_f);
                total_points += transformed.size();
                *globalmap_ptr += transformed;
            }

            LOG_INFO_STREAM("[DataSaver] Loaded " << total_points
                            << " points from keyframe bin (FrameWithOutRT), accumulated cloud size="
                            << globalmap_ptr->size());
            return !globalmap_ptr->empty();
        }
    }

    // Fallback to legacy flat XYZI format: [uint64 frame_count][uint64 point_count][points...]
    {
        std::ifstream bin_stream(bin_file, std::ios::binary);
        if (!bin_stream.is_open())
        {
            LOG_ERROR_STREAM("[DataSaver] Failed to open bin file (legacy path): " << bin_file);
            return false;
        }

        uint64_t frame_count = 0;
        uint64_t point_count = 0;
        bin_stream.read(reinterpret_cast<char *>(&frame_count), sizeof(uint64_t));
        bin_stream.read(reinterpret_cast<char *>(&point_count), sizeof(uint64_t));

        if (!bin_stream)
        {
            LOG_ERROR_STREAM("[DataSaver] Failed to read legacy bin header: " << bin_file);
            return false;
        }

        if (point_count == 0)
        {
            LOG_WARN_STREAM("[DataSaver] Legacy bin header indicates 0 points: " << bin_file);
            return false;
        }

        LOG_INFO_STREAM("[DataSaver] Loading legacy bin file: " << bin_file
                        << " (frames=" << frame_count << ", points=" << point_count << ")");

        globalmap_ptr->clear();
        globalmap_ptr->reserve(static_cast<size_t>(point_count));

        for (uint64_t i = 0; i < point_count; ++i)
        {
            float record[4];
            bin_stream.read(reinterpret_cast<char *>(record), sizeof(record));
            if (!bin_stream)
            {
                LOG_WARN_STREAM("[DataSaver] Failed reading legacy point " << i << " from bin file: " << bin_file);
                break;
            }

            PointT point;
            point.x = record[0];
            point.y = record[1];
            point.z = record[2];
            point.intensity = record[3];
            globalmap_ptr->push_back(point);
        }

        bin_stream.close();

        LOG_INFO_STREAM("[DataSaver] Loaded " << globalmap_ptr->size()
                        << " points from legacy bin file");
        return !globalmap_ptr->empty();
    }
}

bool DataSaver::LoadGlobalMapFromPCDDirectory(const std::string &cloud_directory,
                                              pcl::PointCloud<PointT>::Ptr globalmap_ptr,
                                              bool keyframes_in_body_frame)
{
    if (cloud_directory.empty() || !globalmap_ptr)
    {
        return false;
    }

    namespace fs = std::filesystem;
    if (!fs::exists(cloud_directory) || !fs::is_directory(cloud_directory))
    {
        LOG_WARN_STREAM("[DataSaver] Keyframe directory does not exist: " << cloud_directory);
        return false;
    }

    Eigen::Matrix4f lidar_to_body = Eigen::Matrix4f::Identity();
    if (!keyframes_in_body_frame)
    {
        lidar_to_body = BodyToLidarMatrixF().inverse();
    }

    // Find all PCD files in the directory
    std::vector<std::string> pcd_files;
    try
    {
    for (const auto &entry : fs::directory_iterator(cloud_directory))
    {
        if (entry.is_regular_file() && entry.path().extension() == ".pcd")
        {
            pcd_files.push_back(entry.path().string());
        }
    }
    }
    catch (const std::exception &e)
    {
        LOG_ERROR_STREAM("[DataSaver] Error while scanning directory " << cloud_directory << ": " << e.what());
        return false;
    }

    if (pcd_files.empty())
    {
        LOG_WARN_STREAM("[DataSaver] No PCD files found in directory: " << cloud_directory);
        return false;
    }

    // Sort files by numeric index (assumes naming like 0.pcd, 1.pcd, ...)
    std::sort(pcd_files.begin(), pcd_files.end(), [](const std::string &a, const std::string &b) {
        fs::path path_a(a), path_b(b);
        std::string stem_a = path_a.stem().string();
        std::string stem_b = path_b.stem().string();

        // Try to extract numeric value
        char *end_a, *end_b;
        long num_a = std::strtol(stem_a.c_str(), &end_a, 10);
        long num_b = std::strtol(stem_b.c_str(), &end_b, 10);

        if (end_a != stem_a.c_str() && end_b != stem_b.c_str())
        {
            return num_a < num_b;
        }
        return a < b;
    });

    globalmap_ptr->clear();
    LOG_INFO_STREAM("[DataSaver] Loading " << pcd_files.size() << " keyframe PCD files from " << cloud_directory);

    TicToc ticToc;
    size_t loaded_count = 0;
    for (const auto &file_path : pcd_files)
    {
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
        if (pcl::io::loadPCDFile<PointT>(file_path, *cloud) == -1)
        {
            LOG_WARN_STREAM("[DataSaver] Failed to load PCD file: " << file_path);
            continue;
        }

        if (!keyframes_in_body_frame)
        {
            pcl::transformPointCloud(*cloud, *cloud, lidar_to_body);
        }

        *globalmap_ptr += *cloud;
        ++loaded_count;

        if (loaded_count % 500 == 0)
        {
            LOG_INFO_STREAM("[DataSaver] Loaded " << loaded_count << " keyframes...");
        }
    }

    if (!globalmap_ptr->empty())
    {
        auto filtered = boost::make_shared<pcl::PointCloud<PointT>>();
        custom_filter::VoxelFilterOptimized<PointT>(globalmap_ptr, filtered, 0.5f);
        {
            const auto stats = custom_filter::GetLastVoxelFilterStats();
            LOG_INFO_STREAM("[DataSaver] Voxel filter (PCD dir) leaf=" << stats.voxel_size
                            << "m, input=" << stats.input_points
                            << ", output=" << stats.output_points
                            << ", skipped=" << stats.skipped_points);
        }
        *globalmap_ptr = *filtered;
        LOG_INFO_STREAM("[DataSaver] Global map assembled from " << loaded_count << " PCD files with "
                        << globalmap_ptr->size() << " points, build_time=" << ticToc.toc() / 1000.0 << "s");
        return true;
    }

    return false;
}

bool DataSaver::LoadGridMapTiles(const std::string &map_dir, pcl::PointCloud<PointT>::Ptr globalmap_ptr) const
{
    if (map_dir.empty() || !globalmap_ptr)
    {
        return false;
    }

    namespace fs = std::filesystem;
    fs::path grid_dir;

    // 优先通过 pcd_info.csv 定位网格地图目录
    try
    {
        for (const auto &entry : fs::recursive_directory_iterator(map_dir))
        {
            if (!entry.is_regular_file())
            {
                continue;
            }
            if (entry.path().filename() == "pcd_info.csv")
            {
                grid_dir = entry.path().parent_path();
                LOG_INFO_STREAM("[DataSaver] Found grid map index file: " << entry.path().string());
                break;
            }
        }
    }
    catch (const std::exception &e)
    {
        LOG_WARN_STREAM("[DataSaver] Failed to search pcd_info.csv under " << map_dir << ": " << e.what());
    }

    std::vector<fs::path> candidates;
    if (!grid_dir.empty())
    {
        candidates.push_back(grid_dir);
    }
    else
    {
        // 回退：兼容旧目录结构 pcd/maps 或 pcdmap/maps
        candidates.push_back(fs::path(map_dir) / "pcd" / "maps");
        candidates.push_back(fs::path(map_dir) / "pcdmap" / "maps");
    }

    std::error_code ec;
    for (const auto &dir : candidates)
    {
        if (dir.empty() || !fs::exists(dir, ec) || !fs::is_directory(dir, ec))
        {
            continue;
        }

        std::vector<fs::path> pcd_files;
        try
        {
            for (const auto &entry : fs::directory_iterator(dir))
            {
                if (entry.is_regular_file() && entry.path().extension() == ".pcd")
                {
                    pcd_files.push_back(entry.path());
                }
            }
        }
        catch (const std::exception &e)
        {
            LOG_WARN_STREAM("[DataSaver] Failed to read grid map directory " << dir << ": " << e.what());
            continue;
        }

        if (pcd_files.empty())
        {
            continue;
        }

        std::sort(pcd_files.begin(), pcd_files.end());
        globalmap_ptr->clear();
        std::size_t loaded_files = 0;

        for (const auto &pcd_path : pcd_files)
        {
            pcl::PointCloud<PointT> tile;
            if (pcl::io::loadPCDFile<PointT>(pcd_path.string(), tile) != 0 || tile.empty())
            {
                LOG_WARN_STREAM("[DataSaver] Failed to load grid tile: " << pcd_path);
                continue;
            }
            *globalmap_ptr += tile;
            ++loaded_files;
        }

        if (globalmap_ptr->empty())
        {
            LOG_WARN_STREAM("[DataSaver] Grid map directory " << dir << " contains only empty tiles.");
            continue;
        }

        const float leaf = static_cast<float>(std::max(0.1, map_viewer_size));
        auto filtered = boost::make_shared<pcl::PointCloud<PointT>>();
        custom_filter::VoxelFilterOptimized<PointT>(globalmap_ptr, filtered, leaf);
        {
            const auto stats = custom_filter::GetLastVoxelFilterStats();
            LOG_INFO_STREAM("[DataSaver] Voxel filter (grid map) leaf=" << stats.voxel_size
                            << "m, input=" << stats.input_points
                            << ", output=" << stats.output_points
                            << ", skipped=" << stats.skipped_points);
        }
        *globalmap_ptr = *filtered;

        LOG_INFO_STREAM("[DataSaver] Loaded grid map from " << dir
                        << " (" << loaded_files << " files, points=" << globalmap_ptr->size() << ")");
        return true;
    }

    LOG_WARN_STREAM("[DataSaver] Grid map tiles not found under " << map_dir);
    return false;
}

void DataSaver::GenerateVersionJson(const std::string &output_path)
{
    namespace fs = std::filesystem;

    // JSON escape function
    auto json_escape = [](const std::string &input) -> std::string {
        std::string output;
        output.reserve(input.size() + 4);
        for (unsigned char ch : input)
        {
            switch (ch)
            {
            case '\"':
                output += "\\\"";
                break;
            case '\\':
                output += "\\\\";
                break;
            case '\b':
                output += "\\b";
                break;
            case '\f':
                output += "\\f";
                break;
            case '\n':
                output += "\\n";
                break;
            case '\r':
                output += "\\r";
                break;
            case '\t':
                output += "\\t";
                break;
            default:
                if (ch < 0x20)
                {
                    char buffer[7];
                    std::snprintf(buffer, sizeof(buffer), "\\u%04x", static_cast<unsigned>(ch));
                    output += buffer;
                }
                else
                {
                    output.push_back(static_cast<char>(ch));
                }
                break;
            }
        }
        return output;
    };

    // Get current time
    auto now = std::chrono::system_clock::now();
    std::time_t now_time_t = std::chrono::system_clock::to_time_t(now);

    // Format ISO 8601 timestamp
    std::tm local_time_struct;
#ifdef _WIN32
    localtime_s(&local_time_struct, &now_time_t);
#else
    localtime_r(&now_time_t, &local_time_struct);
#endif
    std::ostringstream iso_builder;
    iso_builder << std::put_time(&local_time_struct, "%Y-%m-%dT%H:%M:%S");
    std::ostringstream tz_builder;
    tz_builder << std::put_time(&local_time_struct, "%z");
    std::string tz = tz_builder.str();
    if (tz.size() == 5)
    {
        iso_builder << tz.substr(0, 3) << ":" << tz.substr(3);
    }
    else if (!tz.empty())
    {
        iso_builder << tz;
    }
    std::string create_time_iso = iso_builder.str();

    // Format compact version string (YYYYMMDDHHmm)
    char version_buffer[16];
    std::strftime(version_buffer, sizeof(version_buffer), "%Y%m%d%H%M", &local_time_struct);
    std::string metadata_version(version_buffer);

    // Try to read origin position from pcdmap/origin_position.cfg
    double origin_latitude = 0.0;
    double origin_longitude = 0.0;
    double origin_altitude = 0.0;
    bool origin_valid = false;

    fs::path origin_cfg_path = fs::path(pcdmap_root_path_) / "origin_position.cfg";
    if (fs::exists(origin_cfg_path))
    {
        std::ifstream origin_stream(origin_cfg_path);
        if (origin_stream.is_open())
        {
            std::string origin_content((std::istreambuf_iterator<char>(origin_stream)),
                                       std::istreambuf_iterator<char>());
            origin_stream.close();
            bool ok_lat = ExtractNumericField(origin_content, "Lat", origin_latitude);
            bool ok_lon = ExtractNumericField(origin_content, "Lon", origin_longitude);
            bool ok_height = ExtractNumericField(origin_content, "Height", origin_altitude);
            origin_valid = ok_lat && ok_lon && ok_height;
        }
    }

    // 从ROS参数服务器读取项目元数据
    auto trim_copy = [](const std::string &value) -> std::string {
        if (value.empty())
        {
            return std::string();
        }
        std::size_t begin = 0;
        std::size_t end = value.size();
        while (begin < end && std::isspace(static_cast<unsigned char>(value[begin])))
        {
            ++begin;
        }
        while (end > begin && std::isspace(static_cast<unsigned char>(value[end - 1])))
        {
            --end;
        }
        return value.substr(begin, end - begin);
    };

    auto read_metadata_param = [](const std::string &param_name, std::string &out_value) {
        XmlRpc::XmlRpcValue value;
        if (!ros::param::get(param_name, value))
        {
            return false;
        }
        return XmlRpcValueToString(value, out_value);
    };

    auto read_metadata_hierarchy = [&](const std::string &base_name, std::string &out_value) {
        if (read_metadata_param("/ms_mapping/" + base_name, out_value))
        {
            return true;
        }
        if (read_metadata_param("~" + base_name, out_value))
        {
            return true;
        }
        if (read_metadata_param(base_name, out_value))
        {
            return true;
        }
        return false;
    };

    auto getenv_or_empty = [](const char *key) -> std::string {
        const char *value = std::getenv(key);
        if (value == nullptr)
        {
            return std::string();
        }
        return std::string(value);
    };

    std::string project_id;
    std::string area_id;
    std::string project_name;
    std::string area_name;

    if (!read_metadata_hierarchy("project_id", project_id))
    {
        project_id = getenv_or_empty("MS_MAPPING_PROJECT_ID");
    }
    if (!read_metadata_hierarchy("area_id", area_id))
    {
        area_id = getenv_or_empty("MS_MAPPING_AREA_ID");
    }
    if (!read_metadata_hierarchy("project_name", project_name))
    {
        project_name = getenv_or_empty("MS_MAPPING_PROJECT_NAME");
    }
    if (!read_metadata_hierarchy("area_name", area_name))
    {
        area_name = getenv_or_empty("MS_MAPPING_AREA_NAME");
    }

    const std::string trimmed_project_id = trim_copy(project_id);
    const std::string trimmed_area_id = trim_copy(area_id);
    const std::string trimmed_project_name = trim_copy(project_name);
    const std::string trimmed_area_name = trim_copy(area_name);

    LOG_INFO_STREAM("[DataSaver] Project metadata: project_id=" << trimmed_project_id
                   << ", area_id=" << trimmed_area_id
                   << ", project_name=" << trimmed_project_name
                   << ", area_name=" << trimmed_area_name);

    auto ensure_metadata_not_empty = [&](const std::string &label, const std::string &value) {
        if (!value.empty())
        {
            return;
        }
        LOG_ERROR_STREAM("[DataSaver] Missing required metadata field: " << label
                          << ". version.json 生成被跳过。");
        throw std::runtime_error("metadata field missing");
    };

    try
    {
        ensure_metadata_not_empty("project_id", trimmed_project_id);
        ensure_metadata_not_empty("area_id", trimmed_area_id);
        ensure_metadata_not_empty("project_name", trimmed_project_name);
        ensure_metadata_not_empty("area_name", trimmed_area_name);
    }
    catch (const std::exception &)
    {
        return;
    }

    // version格式: YYYYMMDDHHmm_0
    std::string version_with_suffix = metadata_version + "_0";

    // Write version.json
    std::ofstream version_ofs(output_path, std::ios::trunc);
    if (!version_ofs.is_open())
    {
        LOG_WARN_STREAM("[DataSaver] Failed to create version.json at: " << output_path);
        return;
    }

    version_ofs << std::setprecision(15);
    version_ofs << "{\n";
    version_ofs << "  \"area_id\": \"" << json_escape(trimmed_area_id) << "\",\n";
    version_ofs << "  \"area_name\": \"" << json_escape(trimmed_area_name) << "\",\n";
    version_ofs << "  \"create_time\": \"" << json_escape(create_time_iso) << "\",\n";
    version_ofs << "  \"origin_position\": {\n";
    if (origin_valid)
    {
        version_ofs << "    \"altitude\": " << origin_altitude << ",\n";
        version_ofs << "    \"latitude\": " << origin_latitude << ",\n";
        version_ofs << "    \"longitude\": " << origin_longitude << "\n";
    }
    else
    {
        version_ofs << "    \"altitude\": null,\n";
        version_ofs << "    \"latitude\": null,\n";
        version_ofs << "    \"longitude\": null\n";
    }
    version_ofs << "  },\n";
    version_ofs << "  \"parent_map\": {\n";
    version_ofs << "    \"area_id\": \"" << json_escape(trimmed_area_id) << "\",\n";
    version_ofs << "    \"project_id\": \"" << json_escape(trimmed_project_id) << "\"\n";
    version_ofs << "  },\n";
    version_ofs << "  \"project_id\": \"" << json_escape(trimmed_project_id) << "\",\n";
    version_ofs << "  \"project_name\": \"" << json_escape(trimmed_project_name) << "\",\n";
    version_ofs << "  \"version\": \"" << json_escape(version_with_suffix) << "\"\n";
    version_ofs << "}\n";
    version_ofs.close();

    LOG_INFO_STREAM("[DataSaver] Generated version.json at: " << output_path);
}

void DataSaver::DumpRuntimeConfig(const std::string &directory) const
{
    namespace fs = std::filesystem;
    fs::path base_path = directory.empty() ? fs::path(save_directory) : fs::path(directory);
    if (base_path.empty())
    {
        LOG_WARN_STREAM("[DataSaver] DumpRuntimeConfig skipped: base directory is empty.");
        return;
    }

    fs::path mapping_root = base_path / "mapping";
    std::error_code ec;
    fs::create_directories(mapping_root, ec);
    if (ec)
    {
        LOG_WARN_STREAM("[DataSaver] Failed to prepare mapping directory for runtime_params.yaml: "
                        << mapping_root.generic_string() << ", " << ec.message());
        return;
    }

    fs::path output_path = mapping_root / "runtime_params.yaml";

    std::ofstream ofs(output_path.string());
    if (!ofs.is_open())
    {
        LOG_WARN_STREAM("[DataSaver] Failed to write runtime config dump to "
                        << output_path.generic_string());
        return;
    }

    ofs << "# Runtime parameters captured when saving the map" << std::endl;

    // 添加项目元数据
    std::string project_id = "";
    std::string area_id = "";
    std::string project_name = "";
    std::string area_name = "";
    ros::param::get("/ms_mapping/project_id", project_id);
    ros::param::get("/ms_mapping/area_id", area_id);
    ros::param::get("/ms_mapping/project_name", project_name);
    ros::param::get("/ms_mapping/area_name", area_name);

    if (!project_id.empty() || !area_id.empty() || !project_name.empty() || !area_name.empty())
    {
        ofs << "metadata:" << std::endl;
        if (!project_id.empty())
        {
            ofs << "  project_id: \"" << project_id << "\"" << std::endl;
        }
        if (!area_id.empty())
        {
            ofs << "  area_id: \"" << area_id << "\"" << std::endl;
        }
        if (!project_name.empty())
        {
            ofs << "  project_name: \"" << project_name << "\"" << std::endl;
        }
        if (!area_name.empty())
        {
            ofs << "  area_name: \"" << area_name << "\"" << std::endl;
        }
        ofs << std::endl;
    }

    std::vector<std::string> namespaces = {"common", "lio", "pgo"};
    bool wrote_any = false;
    for (const auto &ns : namespaces)
    {
        XmlRpc::XmlRpcValue value;
        if (ros::param::get(ns, value))
        {
            EmitYamlKey(ofs, ns, value, 0);
            ofs << std::endl;
            wrote_any = true;
        }
    }

    if (!wrote_any)
    {
        ofs << "common: {}" << std::endl;
    }

    ofs.close();
    LOG_INFO_STREAM("[DataSaver] Runtime parameters stored at " << output_path.generic_string());
}

Eigen::Matrix4d DataSaver::BodyToLidarMatrix() const
{
    EnsureTransformCache();
    return body_to_lidar_matrix_d_;
}

bool DataSaver::EnsureLegacyTrajectoryLoaded()
{
    if (!legacy_trajectory_.empty())
    {
        legacy_mode_enabled_ = true;
        return true;
    }
    return LoadLegacyTrajectory();
}

bool DataSaver::LoadLegacyTrajectory()
{
    namespace fs = std::filesystem;

    legacy_trajectory_.clear();
    legacy_mode_enabled_ = false;

    if (map_directory.empty())
    {
        LOG_ERROR_STREAM("[DataSaver] Legacy trajectory requested but map_directory is empty");
        return false;
    }

    const std::vector<std::string> candidates = {
        "trajectory_imu.pcd",
        "trajectory_dense_imu.pcd",
        "trajectory.pcd"
    };

    std::vector<fs::path> search_roots;
    search_roots.emplace_back(fs::path(map_directory));
    const fs::path pcdmap_dir = fs::path(map_directory) / "pcdmap";
    if (fs::exists(pcdmap_dir) && fs::is_directory(pcdmap_dir))
    {
        search_roots.push_back(pcdmap_dir);
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr trajectory(new pcl::PointCloud<pcl::PointXYZI>());
    bool loaded = false;

    for (const auto &root : search_roots)
    {
        for (const auto &name : candidates)
        {
            fs::path candidate_path = root / name;
            if (!fs::exists(candidate_path))
            {
                continue;
            }
            if (pcl::io::loadPCDFile(candidate_path.string(), *trajectory) == 0 && !trajectory->empty())
            {
                LOG_INFO_STREAM("[DataSaver] Loaded legacy trajectory from " << candidate_path
                                << " (" << trajectory->size() << " poses)");
                loaded = true;
                break;
            }
            LOG_WARN_STREAM("[DataSaver] Failed to read legacy trajectory PCD: " << candidate_path);
        }
        if (loaded)
        {
            break;
        }
    }

    if (loaded && trajectory && !trajectory->empty())
    {
        legacy_trajectory_.reserve(trajectory->size());
        for (std::size_t i = 0; i < trajectory->size(); ++i)
        {
            const auto &point = trajectory->at(static_cast<int>(i));
            Eigen::Vector3d position(point.x, point.y, point.z);

            const auto &prev_point = trajectory->at(static_cast<int>(i > 0 ? i - 1 : i));
            const auto &next_point = trajectory->at(static_cast<int>((i + 1 < trajectory->size()) ? i + 1 : i));

            Eigen::Vector3d prev(prev_point.x, prev_point.y, prev_point.z);
            Eigen::Vector3d next(next_point.x, next_point.y, next_point.z);
            Eigen::Vector3d direction = next - prev;

            if (direction.head<2>().norm() < 1e-6)
            {
                direction = Eigen::Vector3d(next_point.x - point.x, next_point.y - point.y, next_point.z - point.z);
            }
            if (direction.head<2>().norm() < 1e-6)
            {
                direction = Eigen::Vector3d(point.x - prev_point.x, point.y - prev_point.y, point.z - prev_point.z);
            }

            const double yaw = std::atan2(direction.y(), direction.x());
            const double horizontal = std::hypot(direction.x(), direction.y());
            const double pitch = std::atan2(direction.z(), std::max(1e-6, horizontal));
            const double roll = 0.0;

            Pose6D pose;
            pose.x = position.x();
            pose.y = position.y();
            pose.z = position.z();
            pose.setRPY(roll, pitch, yaw);
            pose.valid = true;

            legacy_trajectory_.push_back(pose);
        }

        legacy_mode_enabled_ = true;
        LOG_INFO_STREAM("[DataSaver] Legacy trajectory loaded from PCD with poses=" << legacy_trajectory_.size());
        return true;
    }

    const std::string backend_csv = FindBackendOdomFile(map_directory);
    if (!backend_csv.empty() && LoadBackendOdomLog(backend_csv, legacy_trajectory_))
    {
        legacy_mode_enabled_ = true;
        LOG_INFO_STREAM("[DataSaver] Legacy trajectory loaded from BACKEND_ODOM_LOG.csv with poses=" << legacy_trajectory_.size());
        return true;
    }

    LOG_ERROR_STREAM("[DataSaver] Legacy trajectory not found under " << map_directory
                     << " (missing trajectory_*.pcd and BACKEND_ODOM_LOG.csv)");
    return false;
}

bool DataSaver::LoadLegacyGlobalMap(pcl::PointCloud<PointT>::Ptr globalmap_ptr,
                                    bool apply_downsample)
{
    if (!globalmap_ptr)
    {
        return false;
    }

    namespace fs = std::filesystem;
    if (map_directory.empty())
    {
        LOG_WARN_STREAM("[DataSaver] LoadLegacyGlobalMap requested but map_directory is empty.");
    }
    else
    {
        // 优先匹配常见命名的全局地图文件，然后再兜底匹配任意 global*.pcd。
        const std::array<std::string, 5> preferred_names = {
            "global_map_imu.pcd",
            "global_map_dense_imu.pcd",
            "global_map_dense_lidar.pcd",
            "global_map_lidar.pcd",
            "global_map.pcd"
        };

        auto is_preferred_name = [&](const std::string &filename) -> bool {
            return std::find(preferred_names.begin(), preferred_names.end(), filename) != preferred_names.end();
        };

        std::vector<fs::path> preferred_candidates;
        std::vector<fs::path> generic_candidates;

        fs::path root(map_directory);
        std::error_code ec;
        if (fs::exists(root, ec) && fs::is_directory(root, ec))
        {
            fs::recursive_directory_iterator it(root, ec), end;
            for (; it != end && !ec; it.increment(ec))
            {
                const fs::path &path = it->path();
                if (!fs::is_regular_file(path, ec))
                {
                    continue;
                }
                if (path.extension() != ".pcd")
                {
                    continue;
                }
                const std::string filename = path.filename().string();

                if (is_preferred_name(filename))
                {
                    preferred_candidates.push_back(path);
                }
                else if (filename.size() >= 6 && filename.rfind("global", 0) == 0)
                {
                    generic_candidates.push_back(path);
                }
            }
        }
        else
        {
            LOG_WARN_STREAM("[DataSaver] LoadLegacyGlobalMap: map_directory is not a directory or does not exist: "
                            << map_directory);
        }

        auto try_load = [&](const fs::path &candidate_path) -> bool {
            if (!fs::exists(candidate_path))
            {
                return false;
            }

            if (pcl::io::loadPCDFile<PointT>(candidate_path.string(), *globalmap_ptr) == 0 && !globalmap_ptr->empty())
            {
                if (apply_downsample)
                {
                    // 对旧图全局地图做体素下采样：
                    // - M2F 模式 (useMultiMode=true, baseline=1)：固定使用 0.5m 网格
                    // - 其他模式：沿用 map_viewer_size 配置
                    double leaf_size = map_viewer_size;
                    if (useMultiMode && baseline == 1)
                    {
                        leaf_size = 0.5;
                    }
                    const float leaf = static_cast<float>(std::max(0.1, leaf_size));
                    auto filtered = boost::make_shared<pcl::PointCloud<PointT>>();
                    custom_filter::VoxelFilterOptimized<PointT>(globalmap_ptr, filtered, leaf);
                    {
                        const auto stats = custom_filter::GetLastVoxelFilterStats();
                        LOG_INFO_STREAM("[DataSaver] Voxel filter (legacy map) leaf=" << stats.voxel_size
                                        << "m, input=" << stats.input_points
                                        << ", output=" << stats.output_points
                                        << ", skipped=" << stats.skipped_points);
                    }
                    *globalmap_ptr = *filtered;

                    LOG_INFO_STREAM("[DataSaver] Loaded legacy global map from " << candidate_path
                                    << " (downsampled points=" << globalmap_ptr->size() << ")");
                }
                else
                {
                    LOG_INFO_STREAM("[DataSaver] Loaded legacy global map from " << candidate_path
                                    << " (original resolution, points=" << globalmap_ptr->size() << ")");
                }
                return true;
            }

            LOG_WARN_STREAM("[DataSaver] Failed to read legacy global map PCD: " << candidate_path);
            return false;
        };

        // 先按优先命名加载
        for (const auto &path : preferred_candidates)
        {
            if (try_load(path))
            {
                return true;
            }
        }
        // 再兜底加载任意 global*.pcd
        for (const auto &path : generic_candidates)
        {
            if (try_load(path))
            {
                return true;
            }
        }
    }

    if (LoadGridMapTiles(map_directory, globalmap_ptr))
    {
        return true;
    }

    LOG_ERROR_STREAM("[DataSaver] Legacy global map PCD not found under " << map_directory);
    return false;
}

bool DataSaver::LoadLegacyTrajectoryPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr &trajectory_out)
{
    namespace fs = std::filesystem;
    if (!trajectory_out)
    {
        trajectory_out.reset(new pcl::PointCloud<pcl::PointXYZ>());
    }
    trajectory_out->clear();

    if (map_directory.empty())
    {
        LOG_WARN_STREAM("[DataSaver] Legacy trajectory merge requested but map_directory is empty.");
        return false;
    }

    std::vector<std::string> candidates_body = {
        "pcdmap/trajectory_imu.pcd",
        "trajectory_imu.pcd",
        "pcdmap/traj_pcd_imu.pcd",
        "traj_pcd_imu.pcd"
    };
    std::vector<std::string> candidates_lidar = {
        "pcdmap/trajectory_lidar.pcd",
        "trajectory_lidar.pcd",
        "pcdmap/traj_pcd_lidar.pcd",
        "traj_pcd_lidar.pcd"
    };

    // 优先匹配当前导出坐标系的轨迹文件；同时允许兜底读取另一坐标系的命名。
    std::vector<std::string> search_list;
    if (saveResultBodyFrame)
    {
        search_list = candidates_body;
        search_list.insert(search_list.end(), candidates_lidar.begin(), candidates_lidar.end());
    }
    else
    {
        search_list = candidates_lidar;
        search_list.insert(search_list.end(), candidates_body.begin(), candidates_body.end());
    }

    for (const auto &rel : search_list)
    {
        fs::path candidate = fs::path(map_directory) / rel;
        if (!fs::exists(candidate))
        {
            continue;
        }
        pcl::PointCloud<pcl::PointXYZI> traj_in;
        if (pcl::io::loadPCDFile(candidate.string(), traj_in) != 0 || traj_in.empty())
        {
            LOG_WARN_STREAM("[DataSaver] Failed to load legacy trajectory PCD: " << candidate);
            continue;
        }
        trajectory_out->reserve(trajectory_out->size() + traj_in.size());
        for (const auto &p : traj_in)
        {
            trajectory_out->push_back(pcl::PointXYZ(p.x, p.y, p.z));
        }
        LOG_INFO_STREAM("[DataSaver] Loaded legacy trajectory for merge: " << candidate
                        << " (" << traj_in.size() << " poses)");
        return true;
    }

    if (EnsureLegacyTrajectoryLoaded() && !legacy_trajectory_.empty())
    {
        trajectory_out->reserve(trajectory_out->size() + legacy_trajectory_.size());
        for (const auto &p : legacy_trajectory_)
        {
            trajectory_out->push_back(pcl::PointXYZ(p.x, p.y, p.z));
        }
        LOG_INFO_STREAM("[DataSaver] Reconstructed legacy trajectory points from cached poses ("
                        << legacy_trajectory_.size() << " poses)");
        return true;
    }

    LOG_WARN_STREAM("[DataSaver] Legacy trajectory PCD not found under " << map_directory);
    return false;
}

bool DataSaver::ExportLegacyTrajectoryMeasurements(std::vector<Measurement> &measurements)
{
    measurements.clear();

    auto ensure_trajectory = [&]() -> bool {
        if (!legacy_trajectory_.empty())
        {
            return true;
        }
        const std::string tum = FindTumTrajectoryFile(map_directory);
        if (tum.empty())
        {
            return false;
        }
        std::ifstream tum_in(tum);
        std::string line;
        while (std::getline(tum_in, line))
        {
            std::istringstream iss(line);
            double ts, x, y, z, qx, qy, qz, qw;
            if (!(iss >> ts >> x >> y >> z >> qx >> qy >> qz >> qw))
                continue;
            Pose6D p;
            p.x = x;
            p.y = y;
            p.z = z;
            Eigen::Quaterniond q(qw, qx, qy, qz);
            Eigen::Vector3d rpy = q.toRotationMatrix().eulerAngles(0, 1, 2);
            p.roll = rpy[0];
            p.pitch = rpy[1];
            p.yaw = rpy[2];
            p.valid = true;
            legacy_trajectory_.push_back(p);
        }
        return !legacy_trajectory_.empty();
    };

    if (!ensure_trajectory())
    {
        return false;
    }

    measurements.reserve(legacy_trajectory_.size());
    double cumulative_distance = 0.0;
    for (std::size_t i = 0; i < legacy_trajectory_.size(); ++i)
    {
        Measurement m;
        const auto &p = legacy_trajectory_.at(i);
        m.key_pose = p;
        m.updated_pose = p;
        m.global_pose = p;
        m.global_pose.valid = true;
        m.isKey = true;
        m.distance = cumulative_distance;
        m.odom_time = static_cast<double>(i);
        m.lidar_time = m.odom_time;
        m.global_score = 0.0;

        measurements.push_back(m);

        if (i + 1 < legacy_trajectory_.size())
        {
            const auto &next = legacy_trajectory_.at(i + 1);
            Eigen::Vector3d curr(p.x, p.y, p.z);
            Eigen::Vector3d nxt(next.x, next.y, next.z);
            cumulative_distance += (nxt - curr).norm();
        }
    }
    LOG_INFO_STREAM("[DataSaver] Exported legacy trajectory to measurements for visualization: "
                    << measurements.size() << " poses");
    return true;
}

void DataSaver::ReadPosesAndPointClouds(const std::string &tum_file, const std::string &cloud_directory,
                                        std::vector<Measurement> &measurements, pcl::PointCloud<PointT>::Ptr globalmap_ptr,
                                        bool keyframes_in_body_frame,
                                        bool use_keyframe_map)
{
    namespace fs = std::filesystem;

    measurements.clear();
    if (!globalmap_ptr)
    {
        LOG_ERROR_STREAM("[DataSaver] Global map pointer is null; aborting load");
        return;
    }

    globalmap_ptr->clear();

    // Try flexible file finding if the specified file doesn't exist
    std::string actual_tum_file = tum_file;
    if (!fs::exists(tum_file))
    {
        LOG_WARN_STREAM("[DataSaver] TUM file not found at " << tum_file << ", searching for alternative...");
        fs::path tum_dir = fs::path(tum_file).parent_path();
        if (tum_dir.empty())
        {
            tum_dir = fs::path(cloud_directory).parent_path();
        }
        actual_tum_file = FindTumTrajectoryFile(tum_dir.string());
        if (!actual_tum_file.empty())
        {
            LOG_INFO_STREAM("[DataSaver] Using alternative TUM file: "
                            << ColorizePath(actual_tum_file, kAnsiTumPathColor));
        }
    }

    const bool tum_available = !actual_tum_file.empty() && fs::exists(actual_tum_file);
    const bool keyframe_dir_available = fs::exists(fs::path(cloud_directory));
    LOG_INFO_STREAM("[DataSaver] Prior map probe: tum_available=" << tum_available
                    << ", keyframe_dir=" << (keyframe_dir_available ? cloud_directory : "(missing)")
                    << ", map_dir=" << map_directory);
    std::size_t tum_pose_count = 0;

    const float scan_leaf = static_cast<float>(scan_filter_size);
    constexpr float map_leaf = 0.5f;

    Eigen::Matrix4f lidar_to_body = Eigen::Matrix4f::Identity();
    if (!keyframes_in_body_frame)
    {
        lidar_to_body = BodyToLidarMatrixF().inverse();
    }

    bool standard_loaded = false;
    // 当 use_keyframe_map=false 时（例如 M2F 模式），跳过 TUM+PCD 重建路径，
    // 保证 globalmap_ptr 最终来自 global_*.pcd / grid 或其他 legacy 地图源。
    if (use_keyframe_map && tum_available && keyframe_dir_available)
    {
        std::ifstream tum_stream(actual_tum_file);
        if (!tum_stream.is_open())
        {
            LOG_ERROR_STREAM("[DataSaver] Failed to open pose file: " << actual_tum_file);
        }
        else
        {
            std::string line;
            while (std::getline(tum_stream, line))
            {
                if (line.empty())
                {
                    continue;
                }

                std::istringstream iss(line);
                double timestamp, x, y, z, qx, qy, qz, qw;
                if (!(iss >> timestamp >> x >> y >> z >> qx >> qy >> qz >> qw))
                {
                    LOG_WARN_STREAM("[DataSaver] Skipping malformed TUM pose line: " << line);
                    continue;
                }

                gtsam::Rot3 rot = gtsam::Rot3::Quaternion(qw, qx, qy, qz);
                gtsam::Point3 trans(x, y, z);
                gtsam::Pose3 pose3(rot, trans);

                Pose6D pose = GtsamPose2Pose6D(pose3);
                Measurement measurement;
                measurement.odom_time = timestamp;
                measurement.lidar_time = timestamp;
                measurement.key_pose = pose;
                measurement.updated_pose = pose;
                measurement.global_pose = pose;
                measurement.global_pose.valid = true;
                measurement.isKey = true;
                measurements.push_back(measurement);
                ++tum_pose_count;
            }

            if (!measurements.empty())
            {
                TicToc ticToc;
                for (std::size_t index = 0; index < measurements.size(); ++index)
                {
                    std::string file_name = cloud_directory + std::to_string(index) + ".pcd";
                    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
                    if (pcl::io::loadPCDFile<pcl::PointXYZI>(file_name, *cloud) == -1)
                    {
                        LOG_WARN_STREAM("[DataSaver] Failed to read keyframe cloud: " << file_name);
                        continue;
                    }

                    if (!keyframes_in_body_frame)
                    {
                        pcl::transformPointCloud(*cloud, *cloud, lidar_to_body);
                    }

                    if (!useRawCloud && scan_leaf > 0.0f)
                    {
                        auto filtered_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
                        custom_filter::VoxelFilterOptimized<pcl::PointXYZI>(cloud, filtered_cloud, scan_leaf);
                        cloud = filtered_cloud;
                    }

                    measurements[index].lidar = cloud;

                    Pose3 pose = Pose6dTogtsamPose3(measurements.at(index).updated_pose);
                    Eigen::Matrix4f transformation = pose.matrix().cast<float>();
                    pcl::PointCloud<pcl::PointXYZI> transformed_cloud;
                    pcl::transformPointCloud(*cloud, transformed_cloud, transformation);
                    *globalmap_ptr += transformed_cloud;

                    if (index % 500 == 0)
                    {
                        LOG_INFO_STREAM("[DataSaver] Loaded keyframe " << index << " from " << file_name);
                    }
                }

                LOG_INFO_STREAM("[DataSaver] Parsed TUM poses: " << tum_pose_count
                                << ", matched keyframe clouds: " << measurements.size());

                auto filtered = boost::make_shared<pcl::PointCloud<PointT>>();
                custom_filter::VoxelFilterOptimized<PointT>(globalmap_ptr, filtered, map_leaf);
                {
                    const auto stats = custom_filter::GetLastVoxelFilterStats();
                    LOG_INFO_STREAM("[DataSaver] Voxel filter (TUM load) leaf=" << stats.voxel_size
                                    << "m, input=" << stats.input_points
                                    << ", output=" << stats.output_points
                                    << ", skipped=" << stats.skipped_points);
                }
                *globalmap_ptr = *filtered;
                LOG_INFO_STREAM("[DataSaver] Global map assembled with " << globalmap_ptr->size()
                                << " points, build_time=" << ticToc.toc() / 1000.0 << "s");
                standard_loaded = true;
            }
            else
            {
                LOG_WARN_STREAM("[DataSaver] No poses parsed from TUM file: " << actual_tum_file);
            }
        }
    }

    if (standard_loaded)
    {
        legacy_mode_enabled_ = false;
        return;
    }

    // Try loading from keyframe directory or keyframe bin (preferred), then fall back to legacy global map.
    LOG_WARN_STREAM("[DataSaver] Standard TUM+PCD loading failed, trying alternative formats...");

    // 1) 首先尝试关键帧 PCD 目录（无轨迹，仅用于某些模式下以关键帧重建地图）。
    //    当 use_keyframe_map=false 时（如 M2F 基线），跳过该路径，改由 global_*.pcd/grid 提供先验地图。
    if (use_keyframe_map && keyframe_dir_available)
    {
        if (LoadGlobalMapFromPCDDirectory(cloud_directory, globalmap_ptr, keyframes_in_body_frame))
        {
            LOG_INFO_STREAM("[DataSaver] Successfully loaded global map from PCD directory (M2F mode, no trajectory)");
            measurements.clear();  // M2F mode doesn't需要 measurements
            legacy_mode_enabled_ = false;
            return;
        }
        else
        {
            LOG_WARN_STREAM("[DataSaver] Failed to assemble map from PCD directory: " << cloud_directory);
        }
    }

    // 2) 其次尝试 keyframe bin（mapping/keyframes 或 pcdmap/keyframes 等）
    fs::path map_dir = map_directory.empty()
                           ? fs::path(cloud_directory).parent_path()
                           : fs::path(map_directory);

    LOG_WARN_STREAM("[DataSaver] Bin search root: " << map_dir);
    std::string bin_file = FindKeyframeBinFile(map_dir.string());
    if (use_keyframe_map && !bin_file.empty())
    {
        LOG_WARN_STREAM("[DataSaver] Trying keyframe bin: " << bin_file);

        std::ifstream bin_stream(bin_file, std::ios::binary);
        std::vector<FrameWithOutRT> frames;
        if (bin_stream.is_open())
        {
            CompressModule::LoadFromAllBin(bin_stream, frames);
            bin_stream.close();
        }
        else
        {
            LOG_WARN_STREAM("[DataSaver] Failed to open keyframe bin: " << bin_file);
        }

        if (!frames.empty())
            {
                // Decode each frame into measurements and accumulate global map (bin作为关键帧来源)
                measurements.clear();
                globalmap_ptr->clear();
                LOG_INFO_STREAM("[DataSaver] Keyframe bin parsed: frames=" << frames.size()
                                << " (expect pose count == frame count)");
                CompressModule decompressor;

            Eigen::Matrix4f lidar_to_body = Eigen::Matrix4f::Identity();
            Eigen::Matrix4d lidar_to_body_d = Eigen::Matrix4d::Identity();
            Eigen::Matrix4f body_to_lidar = Eigen::Matrix4f::Identity();
            Eigen::Matrix4d body_to_lidar_d = Eigen::Matrix4d::Identity();

            if (!keyframes_in_body_frame)
            {
                lidar_to_body = BodyToLidarMatrixF().inverse();
                lidar_to_body_d = BodyToLidarMatrix().inverse();
                body_to_lidar = BodyToLidarMatrixF();
                body_to_lidar_d = BodyToLidarMatrix();
            }

            const float scan_leaf = static_cast<float>(scan_filter_size);
            constexpr float map_leaf = 0.5f;

            // [FIX] 检测是否可能是旧版本的不一致bin文件
            // 旧版本bug: 当saveResultBodyFrame=false时，bin中位姿是lidar frame，但点云是body frame
            bool legacy_inconsistent_bin = false;
            if (!keyframes_in_body_frame)
            {
                // 如果检测到bin文件应该是lidar frame，但实际上旧版本可能点云是body frame
                // 我们需要特殊处理：点云不转换，但位姿需要转换
                LOG_WARN_STREAM("[DataSaver] Detected potential legacy bin format (lidar frame). "
                                << "Attempting compatibility mode for old bin files with frame mismatch.");
            }

            for (const auto &frame : frames)
            {
                // Decode into a PCL Ptr; wrap a non-owning std::shared_ptr for decoder API.
                pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_boost(new pcl::PointCloud<pcl::PointXYZI>());
                std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> cloud_std(
                    cloud_boost.get(), [](pcl::PointCloud<pcl::PointXYZI> *) {});
                double ts = 0.0;
                Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
                decompressor.DecodeWithOutRt(frame, cloud_std, ts, pose);

                // [FIX] 修复旧版本bin文件的坐标系不一致问题
                // 新版本bin文件：点云和位姿坐标系一致
                // 旧版本bin文件：如果saveResultBodyFrame=false，点云是body frame，位姿是lidar frame
                if (!keyframes_in_body_frame)
                {
                    // 情况1: 新版本正确的bin文件 - 点云和位姿都是lidar frame
                    //        需要将两者都转换到body frame
                    // 情况2: 旧版本有bug的bin文件 - 点云是body frame，位姿是lidar frame
                    //        只需要将位姿转换到body frame

                    // 由于无法区分这两种情况，我们采用以下策略：
                    // - 位姿始终转换（从lidar frame到body frame）
                    // - 点云不转换（假设是body frame）
                    // 这样对于旧版本bin文件是正确的，对于新版本bin文件会有问题
                    // 但由于旧版本bin文件更常见，这是更安全的选择
                    pose = lidar_to_body_d * pose;
                    // 点云不转换，保持原样（假设是body frame）
                }
                else
                {
                    // bin文件存储的是body frame，点云和位姿都不需要转换
                }

                if (!useRawCloud && scan_leaf > 0.0f)
                {
                    auto filtered_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
                    custom_filter::VoxelFilterOptimized<pcl::PointXYZI>(cloud_boost, filtered_cloud, scan_leaf);
                    cloud_boost = filtered_cloud;
                }

                // Build measurement from pose matrix
                Measurement measurement;
                measurement.isKey = true;
                measurement.odom_time = ts;
                measurement.lidar_time = ts;

                Eigen::Vector3d t = pose.block<3, 1>(0, 3);
                Eigen::Matrix3d R = pose.block<3, 3>(0, 0);
                Eigen::Vector3d rpy = R.eulerAngles(0, 1, 2);

                measurement.key_pose.x = t.x();
                measurement.key_pose.y = t.y();
                measurement.key_pose.z = t.z();
                measurement.key_pose.roll = rpy[0];
                measurement.key_pose.pitch = rpy[1];
                measurement.key_pose.yaw = rpy[2];
                measurement.key_pose.valid = true;
                measurement.updated_pose = measurement.key_pose;
                measurement.global_pose = measurement.key_pose;
                measurement.global_pose.valid = true;
                measurement.global_score = 0.0;
                measurement.distance = 0.0;

                measurement.lidar = cloud_boost;
                measurements.push_back(measurement);

                Pose3 pose3 = Pose6dTogtsamPose3(measurement.updated_pose);
                Eigen::Matrix4f transformation = pose3.matrix().cast<float>();
                pcl::PointCloud<pcl::PointXYZI> transformed_cloud;
                pcl::transformPointCloud(*cloud_boost, transformed_cloud, transformation);
                *globalmap_ptr += transformed_cloud;
            }

            // 为测量补全累计里程
            double cumulative_distance = 0.0;
            for (std::size_t i = 0; i + 1 < measurements.size(); ++i)
            {
                const Pose6D &curr = measurements[i].updated_pose;
                const Pose6D &next = measurements[i + 1].updated_pose;
                Eigen::Vector3d c(curr.x, curr.y, curr.z);
                Eigen::Vector3d n(next.x, next.y, next.z);
                cumulative_distance += (n - c).norm();
                measurements[i + 1].distance = cumulative_distance;
            }

            // Optional voxel filter for the assembled map
            auto filtered = boost::make_shared<pcl::PointCloud<PointT>>();
                custom_filter::VoxelFilterOptimized<PointT>(globalmap_ptr, filtered, map_leaf);
                *globalmap_ptr = *filtered;

                LOG_INFO_STREAM("[DataSaver] Loaded " << measurements.size()
                                << " keyframes from keyframe bin and assembled map, points=" << globalmap_ptr->size());
                LOG_WARN_STREAM("[DataSaver] Bin stats: frames=" << frames.size()
                                << ", decoded keyframes=" << measurements.size()
                                << " (should match if bin carries per-frame pose/cloud)");
                legacy_mode_enabled_ = false;
                return;
            }
            else
            {
            LOG_WARN_STREAM("[DataSaver] Failed to load keyframe bin (no frames parsed): " << bin_file);
        }
    }
    else
    {
        LOG_WARN_STREAM("[DataSaver] Keyframe bin not found under " << map_dir);
    }

    // 3) 再尝试旧图自带的全局地图（global_*.pcd 或 grid map）。
    //    - 若同时存在 legacy 轨迹（TUM / 轨迹 PCD / BACKEND），则在此处一并转换为 Measurement，
    //      这样 M2F 多段扩图在 SaveData 时可以按段导出 old_session 轨迹 TUM / PCD / KML。
    //    - 若没有轨迹，则保持以往行为：仅加载全局地图，不构造旧测量。
    if (LoadLegacyGlobalMap(globalmap_ptr))
    {
        if (EnsureLegacyTrajectoryLoaded() && !legacy_trajectory_.empty())
        {
            double cumulative_distance = 0.0;
            measurements.reserve(legacy_trajectory_.size());
            for (std::size_t i = 0; i < legacy_trajectory_.size(); ++i)
            {
                Measurement measurement;
                const Pose6D &pose = legacy_trajectory_.at(i);
                measurement.key_pose = pose;
                measurement.updated_pose = pose;
                measurement.global_pose = pose;
                measurement.global_pose.valid = true;
                measurement.distance = cumulative_distance;
                measurement.odom_time = static_cast<double>(i);
                measurement.lidar_time = measurement.odom_time;
                measurement.isKey = true;
                measurements.push_back(measurement);

                if (i + 1 < legacy_trajectory_.size())
                {
                    const Pose6D &next_pose = legacy_trajectory_.at(i + 1);
                    Eigen::Vector3d curr_xyz(pose.x, pose.y, pose.z);
                    Eigen::Vector3d next_xyz(next_pose.x, next_pose.y, next_pose.z);
                    cumulative_distance += (next_xyz - curr_xyz).norm();
                }
            }

            LOG_INFO_STREAM("[DataSaver] Loaded global map and legacy trajectory; converted to "
                            << measurements.size() << " measurements");
            legacy_mode_enabled_ = true;
            return;
        }

        LOG_INFO_STREAM("[DataSaver] Loaded standalone global map (global_*.pcd or grid tiles); skipping prior keyframes/trajectory");
        measurements.clear();
        legacy_mode_enabled_ = true;
        return;
    }
    else
    {
        LOG_WARN_STREAM("[DataSaver] Standalone global map files not found under " << map_directory);
    }

    // 4) 最后，尝试仅用 legacy 轨迹（无任何旧地图）进行可视化
    LOG_WARN_STREAM("[DataSaver] Falling back to legacy map import from " << map_directory);
    if (!EnsureLegacyTrajectoryLoaded())
    {
        measurements.clear();
        legacy_mode_enabled_ = false;
        return;
    }

    double cumulative_distance = 0.0;
    measurements.reserve(legacy_trajectory_.size());
    for (std::size_t i = 0; i < legacy_trajectory_.size(); ++i)
    {
        Measurement measurement;
        const Pose6D &pose = legacy_trajectory_.at(i);
        measurement.key_pose = pose;
        measurement.updated_pose = pose;
        measurement.global_pose = pose;
        measurement.global_pose.valid = true;
        measurement.distance = cumulative_distance;
        measurement.odom_time = static_cast<double>(i);
        measurement.lidar_time = measurement.odom_time;
        measurement.isKey = true;
        measurements.push_back(measurement);

        if (i + 1 < legacy_trajectory_.size())
        {
            const Pose6D &next_pose = legacy_trajectory_.at(i + 1);
            Eigen::Vector3d curr_xyz(pose.x, pose.y, pose.z);
            Eigen::Vector3d next_xyz(next_pose.x, next_pose.y, next_pose.z);
            cumulative_distance += (next_xyz - curr_xyz).norm();
        }
    }

    LOG_INFO_STREAM("[DataSaver] Legacy trajectory converted to " << measurements.size() << " measurements");

    if (!LoadLegacyGlobalMap(globalmap_ptr))
    {
        globalmap_ptr->clear();
    }

    legacy_mode_enabled_ = true;
}

gtsam::NonlinearFactorGraph
DataSaver::BuildFactorGraph(const std::string &g2o_file, gtsam::Values &initial_values)
{
    using namespace gtsam;
    using Edge = std::pair<int, int>;
    using EdgeSet = std::unordered_set<Edge, boost::hash<Edge>>;
    std::cout << "Building graph from PGO file! " << std::endl;

    namespace fs = std::filesystem;
    initial_values.clear();

    NonlinearFactorGraph graph;

    auto continuous_noise = noiseModel::Diagonal::Sigmas(
        (Eigen::Matrix<double, 6, 1>() << Eigen::Vector3d::Constant(1e-4),
         Eigen::Vector3d::Constant(1e-3)).finished());

    // Try flexible file finding if the specified file doesn't exist
    std::string actual_g2o_file = g2o_file;
    if (!g2o_file.empty() && !fs::exists(g2o_file))
    {
        LOG_WARN_STREAM("[DataSaver] G2O file not found at " << g2o_file << ", searching for alternative...");
        fs::path g2o_dir = fs::path(g2o_file).parent_path();
        if (g2o_dir.empty())
        {
            g2o_dir = map_directory;
        }
        actual_g2o_file = FindG2oFile(g2o_dir.string());
        if (!actual_g2o_file.empty())
        {
            LOG_INFO_STREAM("[DataSaver] Using alternative G2O file: "
                            << ColorizePath(actual_g2o_file, kAnsiG2oPathColor));
        }
    }

    if (actual_g2o_file.empty() || !fs::exists(actual_g2o_file))
    {
        LOG_WARN_STREAM("[DataSaver] Pose graph file missing: " << g2o_file
                         << " — falling back to legacy trajectory values.");

        const std::string backend_csv = FindBackendOdomFile(map_directory);
        if (!backend_csv.empty())
        {
            std::vector<Pose6D, Eigen::aligned_allocator<Pose6D>> odom_traj;
            if (LoadBackendOdomLog(backend_csv, odom_traj))
            {
                legacy_trajectory_ = odom_traj;
                for (std::size_t i = 0; i < odom_traj.size(); ++i)
                {
                    initial_values.insert(X(static_cast<int>(i)), Pose6dTogtsamPose3(odom_traj.at(i)));
                    if (i > 0)
                    {
                        Pose3 prev = Pose6dTogtsamPose3(odom_traj.at(i - 1));
                        Pose3 curr = Pose6dTogtsamPose3(odom_traj.at(i));
                        Pose3 between = prev.between(curr);
                        graph.emplace_shared<BetweenFactor<Pose3>>(
                            X(static_cast<int>(i - 1)), X(static_cast<int>(i)), between, continuous_noise);
                    }
                }

                if (!odom_traj.empty())
                {
                    graph.addPrior(X(0), Pose6dTogtsamPose3(odom_traj.front()), continuous_noise);
                }

                LOG_INFO_STREAM("[DataSaver] Reconstructed pose graph from BACKEND_ODOM_LOG.csv with "
                                << odom_traj.size() << " poses and "
                                << (odom_traj.size() > 0 ? odom_traj.size() - 1 : 0) << " odom factors.");
                legacy_mode_enabled_ = false;
                return graph;
            }
            else
            {
                LOG_WARN_STREAM("[DataSaver] BACKEND_ODOM_LOG.csv present but failed to parse: " << backend_csv);
            }
        }
        else
        {
            LOG_WARN_STREAM("[DataSaver] No BACKEND_ODOM_LOG.csv found under " << map_directory);
        }

        // 优先尝试加载 TUM 轨迹（例如 optimized_poses_tum.txt），仅用于可视化，不进入优化
        const std::string tum_fallback = FindTumTrajectoryFile(map_directory);
        if (!tum_fallback.empty())
        {
            std::ifstream tum_in(tum_fallback);
            std::string line;
            legacy_trajectory_.clear();
            while (std::getline(tum_in, line))
            {
                std::istringstream iss(line);
                double ts, x, y, z, qx, qy, qz, qw;
                if (!(iss >> ts >> x >> y >> z >> qx >> qy >> qz >> qw))
                    continue;
                Pose6D p;
                p.x = x;
                p.y = y;
                p.z = z;
                Eigen::Quaterniond q(qw, qx, qy, qz);
                Eigen::Vector3d rpy = q.toRotationMatrix().eulerAngles(0, 1, 2);
                p.roll = rpy[0];
                p.pitch = rpy[1];
                p.yaw = rpy[2];
                p.valid = true;
                legacy_trajectory_.push_back(p);
            }

            if (!legacy_trajectory_.empty())
            {
                for (std::size_t i = 0; i < legacy_trajectory_.size(); ++i)
                {
                    initial_values.insert(X(static_cast<int>(i)), Pose6dTogtsamPose3(legacy_trajectory_.at(i)));
                }
                LOG_INFO_STREAM("[DataSaver] Loaded TUM trajectory for M2F visualization: poses="
                                << legacy_trajectory_.size() << " from " << tum_fallback);
                legacy_mode_enabled_ = true;
                return graph;
            }
        }

        if (!EnsureLegacyTrajectoryLoaded())
        {
            LOG_ERROR_STREAM("[DataSaver] Unable to load legacy trajectory from " << map_directory
                             << " (no g2o/TUM/BACKEND_ODOM_LOG).");
            return graph;
        }

        for (std::size_t i = 0; i < legacy_trajectory_.size(); ++i)
        {
            initial_values.insert(X(static_cast<int>(i)), Pose6dTogtsamPose3(legacy_trajectory_.at(i)));
        }

        LOG_INFO_STREAM("[DataSaver] Initialized " << legacy_trajectory_.size()
                        << " legacy poses as initial graph values.");
        legacy_mode_enabled_ = true;
        return graph;
    }

    legacy_mode_enabled_ = false;

    std::ifstream g2o_stream(actual_g2o_file);
    std::string line;
    int line_number = 0;
    std::unordered_map<int, int> id_map, id_map_test; // 原始ID到新ID的映射
    int new_id = 0;                                   // 新ID
    EdgeSet existing_edges;

    int num_continuous_edges = 0;
    int num_non_continuous_edges = 0;
    const int start_line = 0;
    bool anchor_vertex_present = false;
    bool anchor_prior_loaded = false;
    bool first_vertex_recorded = false;
    Symbol first_vertex_key;
    Pose3 first_vertex_pose = Pose3::Identity();
    //    const int start_line = 10; // 从第10行开始读取
    while (std::getline(g2o_stream, line))
    {
        std::istringstream iss(line);
        std::string tag;
        iss >> tag;
        if (tag.empty())
            continue;
        if (tag[0] == '#') {
            // read the next token as actual tag, enabling commented metadata lines
            std::string commentTag;
            if (!(iss >> commentTag))
                continue;
            tag = commentTag;
        }
        if (tag == "VERTEX_SE3:QUAT")
        {
            int original_id;
            double x, y, z, qx, qy, qz, qw;
            iss >> original_id >> x >> y >> z >> qx >> qy >> qz >> qw;
            if (original_id < start_line && original_id >= 0)
                continue;

            if (original_id == kG2oAnchorId)
            {
                anchor_vertex_present = true;
                continue;
            }

            // std::cout << "add node: " << original_id << " " << new_id << std::endl;
            Eigen::Quaterniond quat(qw, qx, qy, qz);
            Pose3 pose(Rot3(quat), Point3(x, y, z));
            // 为顶点创建一个新的ID并添加到映射中
            Symbol key = X(new_id);
            id_map[original_id] = new_id;
            initial_values.insert(key, pose);
            if (!first_vertex_recorded)
            {
                first_vertex_recorded = true;
                first_vertex_key = key;
                first_vertex_pose = pose;
            }
            new_id++;
        }
        else if (tag == "GNSS_PRIOR")
        {
            std::string subtype;
            int original_id;
            if (!(iss >> subtype >> original_id))
                continue;

            auto mapped = id_map.find(original_id);
            if (mapped == id_map.end())
                continue;

            const Symbol key = X(mapped->second);
            std::string robust_type = "NONE";
            double robust_param = 0.0;

            if (subtype == "XYZ")
            {
                double px = 0.0, py = 0.0, pz = 0.0;
                int sigma_dim = 0;
                if (!(iss >> px >> py >> pz >> sigma_dim) || sigma_dim != 3)
                    continue;

                Eigen::VectorXd sigmas(sigma_dim);
                for (int i = 0; i < sigma_dim; ++i)
                {
                    if (!(iss >> sigmas(i)))
                        sigmas(i) = 0.0;
                }
                if (iss >> robust_type >> robust_param)
                {
                    // ok
                }
                else
                {
                    robust_type = "NONE";
                    robust_param = 0.0;
                    iss.clear();
                }

                GnssNoiseSerialization serialization;
                serialization.sigmas = sigmas;
                serialization.robust_type = robust_type;
                serialization.robust_param = robust_param;

                SharedNoiseModel noise = ReconstructNoiseModel(serialization);
                graph.emplace_shared<GPSFactor>(key, Point3(px, py, pz), noise);
            }
            else if (subtype == "POSE")
            {
                double px = 0.0, py = 0.0, pz = 0.0;
                double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;
                int sigma_dim = 0;
                if (!(iss >> px >> py >> pz >> qx >> qy >> qz >> qw >> sigma_dim) || sigma_dim != 6)
                    continue;

                Eigen::VectorXd sigmas(sigma_dim);
                for (int i = 0; i < sigma_dim; ++i)
                {
                    if (!(iss >> sigmas(i)))
                        sigmas(i) = 0.0;
                }
                if (iss >> robust_type >> robust_param)
                {
                    // ok
                }
                else
                {
                    robust_type = "NONE";
                    robust_param = 0.0;
                    iss.clear();
                }

                GnssNoiseSerialization serialization;
                serialization.sigmas = sigmas;
                serialization.robust_type = robust_type;
                serialization.robust_param = robust_param;

                const Eigen::Quaterniond quat(qw, qx, qy, qz);
                Pose3 pose(Rot3(quat), Point3(px, py, pz));
                SharedNoiseModel noise = ReconstructNoiseModel(serialization);
                graph.emplace_shared<PriorFactor<Pose3>>(key, pose, noise);
            }
            else if (subtype == "XY")
            {
                double px = 0.0, py = 0.0;
                int sigma_dim = 0;
                if (!(iss >> px >> py >> sigma_dim) || sigma_dim != 2)
                    continue;

                Eigen::VectorXd sigmas(sigma_dim);
                for (int i = 0; i < sigma_dim; ++i)
                {
                    if (!(iss >> sigmas(i)))
                        sigmas(i) = 0.0;
                }
                if (iss >> robust_type >> robust_param)
                {
                    // ok
                }
                else
                {
                    robust_type = "NONE";
                    robust_param = 0.0;
                    iss.clear();
                }

                GnssNoiseSerialization serialization;
                serialization.sigmas = sigmas;
                serialization.robust_type = robust_type;
                serialization.robust_param = robust_param;

                SharedNoiseModel noise = ReconstructNoiseModel(serialization);
                graph.emplace_shared<XYPriorFactor>(key, Point2(px, py), noise);
            }

            continue;
        }
        else if (tag == "EDGE_SE3:QUAT")
        {
            int id1, id2;
            double x, y, z, qx, qy, qz, qw;
            iss >> id1 >> id2 >> x >> y >> z >> qx >> qy >> qz >> qw;
            if ((id1 < start_line && id1 >= 0) || (id2 < start_line && id2 >= 0))
                continue;

            const bool id1_is_anchor = anchor_vertex_present && id1 == kG2oAnchorId;
            const bool id2_is_anchor = anchor_vertex_present && id2 == kG2oAnchorId;

            // 检查映射中是否存在这两个ID
            auto it1 = id_map.find(id1);
            auto it2 = id_map.find(id2);
            if (!id1_is_anchor && it1 == id_map.end())
                continue; // 忽略未映射的边
            if (!id2_is_anchor && it2 == id_map.end())
                continue;

            // set noise model
            Eigen::Matrix<double, 6, 6> information_matrix = Eigen::Matrix<double, 6, 6>::Zero();
            for (int i = 0; i < 6; ++i)
            {
                for (int j = i; j < 6; ++j)
                {
                    iss >> information_matrix(i, j);
                    if (i != j)
                    {
                        information_matrix(j, i) = information_matrix(i, j); // 填充对称部分
                    }
                }
            }
            // g2o order (translation first) -> GTSAM Pose3 order (rotation first).
            information_matrix = SwapPose3InformationOrder(information_matrix);

            Eigen::Quaterniond quat(qw, qx, qy, qz);
            Pose3 relative_pose(Rot3(quat), Point3(x, y, z));

            if (id1_is_anchor || id2_is_anchor)
            {
                const int pose_original_id = id1_is_anchor ? id2 : id1;
                auto pose_it = id_map.find(pose_original_id);
                if (pose_it == id_map.end())
                    continue;

                SharedNoiseModel prior_noise = noiseModel::Gaussian::Information(information_matrix);
                Symbol pose_key = X(pose_it->second);
                Pose3 prior_pose = id1_is_anchor ? relative_pose : relative_pose.inverse();
                graph.emplace_shared<PriorFactor<Pose3>>(pose_key, prior_pose, prior_noise);
                anchor_prior_loaded = true;
                continue;
            }

            Symbol key1 = X(it1->second);
            Symbol key2 = X(it2->second);

            // 检查边是否已存在
            Edge edge(id1, id2);
            if (existing_edges.find(edge) != existing_edges.end())
            {
                std::cout << "repeat edge: " << it1->second << " -> " << it2->second << std::endl;
                std::cout << "repeat cov: " << information_matrix.inverse().diagonal().matrix().transpose()
                          << std::endl;
                continue; // 如果边已存在，跳过
            }
            existing_edges.insert(edge); // 添加新边到集合

            SharedNoiseModel noise_model = noiseModel::Gaussian::Covariance(information_matrix.inverse());
            // 根据是相邻节点还是非相邻节点选择不同的噪声模型
            bool is_continuous = (it1->second + 1) == it2->second;
            // auto noise_model = is_continuous ? continuous_noise : non_continuous_noise;
            graph.emplace_shared<BetweenFactor<Pose3>>(key1, key2, relative_pose, noise_model);

            if (is_continuous)
            {
                id_map_test[id1] = id2;
                num_continuous_edges++;
            }
            else
            {
                num_non_continuous_edges++;
            }
        }
    }
    std::cout << BOLDRED << "edge and loop numbers: " << num_continuous_edges << " " << num_non_continuous_edges << std::endl;
    std::cout << BOLDRED << "node numbers new_id: " << initial_values.size() - 1 << " " << new_id << std::endl;

    if (!anchor_prior_loaded && first_vertex_recorded)
    {
        graph.addPrior(first_vertex_key, first_vertex_pose, continuous_noise);
    }

    return graph;
}
