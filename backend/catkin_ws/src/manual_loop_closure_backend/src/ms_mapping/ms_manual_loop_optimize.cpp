#include "base_type.hpp"
#include "../data_saver.h"

#include <ros/ros.h>

#include <algorithm>
#include <chrono>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

namespace fs = std::filesystem;

namespace {

struct Options
{
    fs::path session_root;
    fs::path g2o_path;
    fs::path tum_path;
    fs::path keyframe_dir;
    fs::path constraints_csv;
    fs::path output_dir;
    double map_voxel_leaf = 0.2;
    bool skip_map_build = false;
};

struct ManualConstraintSpec
{
    bool enabled = false;
    int source_id = -1;
    int target_id = -1;
    gtsam::Pose3 measured;
    gtsam::SharedNoiseModel noise;
};

std::string TrimCopy(const std::string &value)
{
    const std::size_t first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos)
    {
        return "";
    }
    const std::size_t last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
}

std::string ShellQuote(const std::string &value)
{
    std::string quoted = "'";
    for (const char ch : value)
    {
        if (ch == '\'')
        {
            quoted += "'\\''";
        }
        else
        {
            quoted.push_back(ch);
        }
    }
    quoted.push_back('\'');
    return quoted;
}

bool ParseBool(const std::string &text, bool &value)
{
    std::string normalized = TrimCopy(text);
    std::transform(
        normalized.begin(),
        normalized.end(),
        normalized.begin(),
        [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });

    if (normalized == "1" || normalized == "true" || normalized == "yes")
    {
        value = true;
        return true;
    }
    if (normalized == "0" || normalized == "false" || normalized == "no")
    {
        value = false;
        return true;
    }
    return false;
}

std::vector<std::string> SplitCsvLine(const std::string &line)
{
    std::vector<std::string> cells;
    std::stringstream stream(line);
    std::string cell;
    while (std::getline(stream, cell, ','))
    {
        cells.push_back(TrimCopy(cell));
    }
    return cells;
}

bool LoadSaveBodyFrameFlag(const std::string &session_root, bool &save_body_frame)
{
    const fs::path root(session_root);
    const std::vector<fs::path> candidates = {
        root / "mapping" / "runtime_params.yaml",
        root / "runtime_params.yaml",
    };

    for (const auto &candidate : candidates)
    {
        std::ifstream stream(candidate);
        if (!stream.is_open())
        {
            continue;
        }

        std::string line;
        while (std::getline(stream, line))
        {
            if (line.find("saveResultBodyFrame") == std::string::npos)
            {
                continue;
            }
            const auto colon = line.find(':');
            if (colon == std::string::npos)
            {
                continue;
            }

            bool parsed_value = true;
            if (ParseBool(line.substr(colon + 1), parsed_value))
            {
                save_body_frame = parsed_value;
                return true;
            }
        }
    }

    return false;
}

void InitializeOfflineDefaults()
{
    useRawCloud = false;
    // Keep the input keyframes at full saved resolution so the export map voxel
    // becomes the only downsampling stage that controls the final map density.
    scan_filter_size = 0.0;
    map_saved_size = 0.2;
    grid_map_downsample_size = 0.2;
    localization_map_grid_size = 240.0;
    intensity_image_resolution = 0.1;
    save_grid_map = false;
    save_intensity_image = false;
    saveResultBodyFrame = true;
    q_body_sensor = Eigen::Quaterniond::Identity();
    t_body_sensor = Eigen::Vector3d::Zero();
}

bool ParseArgs(int argc, char **argv, Options &options)
{
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg(argv[i]);
        if (arg == "--help" || arg == "-h")
        {
            return false;
        }
        if (arg == "--skip-map-build")
        {
            options.skip_map_build = true;
            continue;
        }
        if (i + 1 >= argc)
        {
            throw std::runtime_error("Missing value for argument: " + arg);
        }

        const fs::path value = fs::path(argv[++i]).lexically_normal();
        if (arg == "--session-root")
        {
            options.session_root = value;
        }
        else if (arg == "--g2o")
        {
            options.g2o_path = value;
        }
        else if (arg == "--tum")
        {
            options.tum_path = value;
        }
        else if (arg == "--keyframe-dir")
        {
            options.keyframe_dir = value;
        }
        else if (arg == "--constraints-csv")
        {
            options.constraints_csv = value;
        }
        else if (arg == "--output-dir")
        {
            options.output_dir = value;
        }
        else if (arg == "--map-voxel-leaf")
        {
            options.map_voxel_leaf = std::stod(value.string());
        }
        else
        {
            throw std::runtime_error("Unknown argument: " + arg);
        }
    }

    if (options.session_root.empty() || options.g2o_path.empty() || options.tum_path.empty() ||
        options.keyframe_dir.empty() || options.constraints_csv.empty() || options.output_dir.empty())
    {
        throw std::runtime_error(
            "Missing required arguments. Required: --session-root --g2o --tum "
            "--keyframe-dir --constraints-csv --output-dir");
    }
    return true;
}

void PrintUsage(const char *binary_name)
{
    std::cout << "Usage:\n"
              << "  " << binary_name
              << " --session-root <dir>"
              << " --g2o <pose_graph.g2o>"
              << " --tum <optimized_poses_tum.txt>"
              << " --keyframe-dir <key_point_frame>"
              << " --constraints-csv <manual_loop_constraints.csv>"
              << " --output-dir <manual_loop_runs/...>"
              << " [--map-voxel-leaf <meters>]"
              << " [--skip-map-build]\n";
}

gtsam::SharedNoiseModel MakePoseNoise(
    double sigma_tx,
    double sigma_ty,
    double sigma_tz,
    double sigma_roll_deg,
    double sigma_pitch_deg,
    double sigma_yaw_deg)
{
    const double deg_to_rad = M_PI / 180.0;
    gtsam::Vector6 sigmas;
    sigmas << sigma_roll_deg * deg_to_rad,
        sigma_pitch_deg * deg_to_rad,
        sigma_yaw_deg * deg_to_rad,
        sigma_tx,
        sigma_ty,
        sigma_tz;
    return gtsam::noiseModel::Diagonal::Sigmas(sigmas);
}

std::vector<ManualConstraintSpec> LoadConstraintsCsv(const fs::path &path)
{
    std::ifstream stream(path);
    if (!stream.is_open())
    {
        throw std::runtime_error("Failed to open constraints CSV: " + path.string());
    }

    std::string header_line;
    if (!std::getline(stream, header_line))
    {
        throw std::runtime_error("Constraints CSV is empty: " + path.string());
    }
    const auto header = SplitCsvLine(header_line);

    std::map<std::string, std::size_t> indices;
    for (std::size_t i = 0; i < header.size(); ++i)
    {
        indices[header[i]] = i;
    }

    const std::vector<std::string> required = {
        "enabled",
        "source_id",
        "target_id",
        "tx",
        "ty",
        "tz",
        "qx",
        "qy",
        "qz",
        "qw",
        "sigma_tx",
        "sigma_ty",
        "sigma_tz",
        "sigma_roll_deg",
        "sigma_pitch_deg",
        "sigma_yaw_deg",
    };
    for (const auto &field : required)
    {
        if (indices.find(field) == indices.end())
        {
            throw std::runtime_error("Constraints CSV missing required column: " + field);
        }
    }

    std::vector<ManualConstraintSpec> constraints;
    std::string line;
    int line_number = 1;
    while (std::getline(stream, line))
    {
        ++line_number;
        if (TrimCopy(line).empty())
        {
            continue;
        }

        const auto cells = SplitCsvLine(line);
        auto cell = [&](const std::string &name) -> const std::string & {
            const auto it = indices.find(name);
            if (it == indices.end() || it->second >= cells.size())
            {
                throw std::runtime_error(
                    "Malformed constraints CSV row " + std::to_string(line_number));
            }
            return cells[it->second];
        };

        ManualConstraintSpec constraint;
        bool enabled = false;
        if (!ParseBool(cell("enabled"), enabled))
        {
            throw std::runtime_error(
                "Invalid enabled value on CSV row " + std::to_string(line_number));
        }
        constraint.enabled = enabled;
        constraint.source_id = std::stoi(cell("source_id"));
        constraint.target_id = std::stoi(cell("target_id"));

        const double tx = std::stod(cell("tx"));
        const double ty = std::stod(cell("ty"));
        const double tz = std::stod(cell("tz"));
        const double qx = std::stod(cell("qx"));
        const double qy = std::stod(cell("qy"));
        const double qz = std::stod(cell("qz"));
        const double qw = std::stod(cell("qw"));

        const Eigen::Quaterniond quat(qw, qx, qy, qz);
        constraint.measured = gtsam::Pose3(
            gtsam::Rot3(quat),
            gtsam::Point3(tx, ty, tz));
        constraint.noise = MakePoseNoise(
            std::stod(cell("sigma_tx")),
            std::stod(cell("sigma_ty")),
            std::stod(cell("sigma_tz")),
            std::stod(cell("sigma_roll_deg")),
            std::stod(cell("sigma_pitch_deg")),
            std::stod(cell("sigma_yaw_deg")));
        constraints.push_back(constraint);
    }

    return constraints;
}

void SaveTum(const fs::path &path,
             const std::vector<Measurement> &measurements,
             const gtsam::Values &optimized)
{
    std::ofstream stream(path);
    if (!stream.is_open())
    {
        throw std::runtime_error("Failed to open TUM output: " + path.string());
    }

    stream.setf(std::ios::fixed);
    stream.precision(9);
    for (std::size_t i = 0; i < measurements.size(); ++i)
    {
        const auto key = X(static_cast<int>(i));
        if (!optimized.exists(key))
        {
            throw std::runtime_error("Optimized values missing key X" + std::to_string(i));
        }

        const gtsam::Pose3 pose = optimized.at<gtsam::Pose3>(key);
        const auto quat = pose.rotation().toQuaternion();
        stream << measurements[i].odom_time << " "
               << pose.x() << " " << pose.y() << " " << pose.z() << " "
               << quat.x() << " " << quat.y() << " " << quat.z() << " " << quat.w()
               << "\n";
    }
}

pcl::PointCloud<PointT>::Ptr BuildOptimizedMap(
    const std::vector<Measurement> &measurements,
    const gtsam::Values &optimized,
    float voxel_leaf)
{
    auto merged = boost::make_shared<pcl::PointCloud<PointT>>();
    for (std::size_t i = 0; i < measurements.size(); ++i)
    {
        if (!measurements[i].lidar || measurements[i].lidar->empty())
        {
            continue;
        }
        const auto key = X(static_cast<int>(i));
        if (!optimized.exists(key))
        {
            continue;
        }

        const Eigen::Matrix4f transform =
            optimized.at<gtsam::Pose3>(key).matrix().cast<float>();
        pcl::PointCloud<PointT> transformed;
        pcl::transformPointCloud(*measurements[i].lidar, transformed, transform);
        *merged += transformed;
    }

    if (!merged->empty() && voxel_leaf > 0.0f)
    {
        auto filtered = boost::make_shared<pcl::PointCloud<PointT>>();
        pcl::VoxelGrid<PointT> voxel;
        voxel.setLeafSize(voxel_leaf, voxel_leaf, voxel_leaf);
        voxel.setInputCloud(merged);
        voxel.filter(*filtered);
        filtered->width = filtered->size();
        filtered->height = 1;
        filtered->is_dense = false;
        merged.swap(filtered);
    }

    return merged;
}

void SaveTrajectoryPcd(const fs::path &path,
                       const std::vector<Measurement> &measurements,
                       const gtsam::Values &optimized)
{
    pcl::PointCloud<pcl::PointXYZI> trajectory_cloud;
    trajectory_cloud.points.reserve(measurements.size());

    for (std::size_t i = 0; i < measurements.size(); ++i)
    {
        const auto key = X(static_cast<int>(i));
        if (!optimized.exists(key))
        {
            continue;
        }

        const gtsam::Pose3 pose = optimized.at<gtsam::Pose3>(key);
        pcl::PointXYZI point;
        point.x = static_cast<float>(pose.x());
        point.y = static_cast<float>(pose.y());
        point.z = static_cast<float>(pose.z());
        point.intensity = static_cast<float>(measurements[i].odom_time);
        trajectory_cloud.points.push_back(point);
    }

    trajectory_cloud.width = trajectory_cloud.points.size();
    trajectory_cloud.height = 1;
    trajectory_cloud.is_dense = true;
    if (trajectory_cloud.points.empty() ||
        pcl::io::savePCDFileBinary(path.string(), trajectory_cloud) != 0)
    {
        throw std::runtime_error("Failed to save optimized trajectory point cloud.");
    }
}

bool GeneratePoseGraphImage(const fs::path &g2o_path, const fs::path &output_path)
{
    const fs::path script_path = fs::path(ROOT_DIR) / "scripts" / "visualize_pose_graph.py";
    if (!fs::exists(script_path))
    {
        std::cerr << "[ManualLoopOptimize] visualize_pose_graph.py not found: "
                  << script_path << std::endl;
        return false;
    }

    const std::string command =
        "MPLBACKEND=Agg python3 " + ShellQuote(script_path.string()) +
        " --g2o " + ShellQuote(g2o_path.string()) +
        " --output " + ShellQuote(output_path.string());
    return std::system(command.c_str()) == 0;
}

void SaveReportJson(const fs::path &path,
                    const Options &options,
                    std::size_t total_constraints,
                    std::size_t enabled_constraints,
                    const std::vector<Measurement> &measurements,
                    const gtsam::NonlinearFactorGraph &graph,
                    const pcl::PointCloud<PointT>::Ptr &map_cloud,
                    bool map_built,
                    double map_build_elapsed_sec)
{
    std::ofstream stream(path);
    if (!stream.is_open())
    {
        throw std::runtime_error("Failed to open report json: " + path.string());
    }

    stream << "{\n";
    stream << "  \"session_root\": \"" << options.session_root.string() << "\",\n";
    stream << "  \"input_g2o\": \"" << options.g2o_path.string() << "\",\n";
    stream << "  \"input_tum\": \"" << options.tum_path.string() << "\",\n";
    stream << "  \"input_keyframe_dir\": \"" << options.keyframe_dir.string() << "\",\n";
    stream << "  \"constraints_csv\": \"" << options.constraints_csv.string() << "\",\n";
    stream << "  \"output_dir\": \"" << options.output_dir.string() << "\",\n";
    stream << "  \"map_voxel_leaf\": " << options.map_voxel_leaf << ",\n";
    stream << "  \"total_constraints\": " << total_constraints << ",\n";
    stream << "  \"enabled_constraints\": " << enabled_constraints << ",\n";
    stream << "  \"optimized_pose_count\": " << measurements.size() << ",\n";
    stream << "  \"factor_count\": " << graph.size() << ",\n";
    stream << "  \"map_point_count\": " << (map_cloud ? map_cloud->size() : 0) << ",\n";
    stream << "  \"map_built\": " << (map_built ? "true" : "false") << ",\n";
    stream << "  \"map_build_elapsed_sec\": " << std::fixed << std::setprecision(6)
           << map_build_elapsed_sec << "\n";
    stream << "}\n";
}

} // namespace

int main(int argc, char **argv)
{
    try
    {
        Options options;
        if (!ParseArgs(argc, argv, options))
        {
            PrintUsage(argv[0]);
            return EXIT_SUCCESS;
        }

        ros::init(argc, argv, "manual_loop_optimize", ros::init_options::AnonymousName);
        InitializeOfflineDefaults();

        if (!fs::exists(options.session_root) || !fs::is_directory(options.session_root))
        {
            throw std::runtime_error("session_root does not exist: " + options.session_root.string());
        }
        if (!fs::exists(options.g2o_path))
        {
            throw std::runtime_error("g2o file does not exist: " + options.g2o_path.string());
        }
        if (!fs::exists(options.tum_path))
        {
            throw std::runtime_error("tum file does not exist: " + options.tum_path.string());
        }
        if (!fs::exists(options.keyframe_dir) || !fs::is_directory(options.keyframe_dir))
        {
            throw std::runtime_error("keyframe directory does not exist: " + options.keyframe_dir.string());
        }
        if (!fs::exists(options.constraints_csv))
        {
            throw std::runtime_error(
                "constraints csv does not exist: " + options.constraints_csv.string());
        }

        fs::create_directories(options.output_dir);

        bool keyframes_in_body_frame = true;
        if (!LoadSaveBodyFrameFlag(options.session_root.string(), keyframes_in_body_frame))
        {
            std::cerr << "[ManualLoopOptimize] Failed to resolve saveResultBodyFrame from session. "
                      << "Assuming body/imu frame keyframes." << std::endl;
        }

        DataSaver loader;
        loader.setMapDir(options.session_root.string());

        std::vector<Measurement> measurements;
        auto prior_map = boost::make_shared<pcl::PointCloud<PointT>>();
        std::string keyframe_dir_string = options.keyframe_dir.string();
        if (!keyframe_dir_string.empty() && keyframe_dir_string.back() != '/')
        {
            keyframe_dir_string.push_back('/');
        }
        loader.ReadPosesAndPointClouds(
            options.tum_path.string(),
            keyframe_dir_string,
            measurements,
            prior_map,
            keyframes_in_body_frame,
            true);
        if (measurements.empty())
        {
            throw std::runtime_error("Failed to load keyframe measurements from session.");
        }

        gtsam::Values initial_values;
        gtsam::NonlinearFactorGraph graph = loader.BuildFactorGraph(
            options.g2o_path.string(),
            initial_values);
        if (initial_values.empty())
        {
            throw std::runtime_error("Failed to build factor graph from input g2o.");
        }

        const auto constraints = LoadConstraintsCsv(options.constraints_csv);
        std::size_t enabled_constraints = 0;
        for (const auto &constraint : constraints)
        {
            if (!constraint.enabled)
            {
                continue;
            }
            if (constraint.source_id < 0 || constraint.target_id < 0 ||
                constraint.source_id >= static_cast<int>(measurements.size()) ||
                constraint.target_id >= static_cast<int>(measurements.size()))
            {
                throw std::runtime_error(
                    "Constraint index out of range: target=" +
                    std::to_string(constraint.target_id) +
                    " source=" + std::to_string(constraint.source_id));
            }

            graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
                X(constraint.target_id),
                X(constraint.source_id),
                constraint.measured,
                constraint.noise);
            ++enabled_constraints;
        }
        if (enabled_constraints == 0)
        {
            std::cout << "[ManualLoopOptimize] No enabled manual constraints found in CSV. "
                         "Proceeding with the filtered input graph only."
                      << std::endl;
        }

        gtsam::LevenbergMarquardtParams params;
        params.setMaxIterations(50);
        params.setVerbosityLM("SILENT");
        gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial_values, params);
        const gtsam::Values optimized = optimizer.optimize();

        const fs::path output_g2o = options.output_dir / "pose_graph.g2o";
        if (!loader.WriteGraphG2o(output_g2o.string(), graph, optimized))
        {
            throw std::runtime_error("Failed to write optimized g2o.");
        }

        const fs::path output_tum = options.output_dir / "optimized_poses_tum.txt";
        SaveTum(output_tum, measurements, optimized);

        pcl::PointCloud<PointT>::Ptr optimized_map;
        bool map_built = false;
        double map_build_elapsed_sec = 0.0;
        if (options.skip_map_build)
        {
            std::cout << "[ManualLoopOptimize] Map rebuild deferred until export." << std::endl;
        }
        else
        {
            const auto map_stage_start = std::chrono::steady_clock::now();
            optimized_map = BuildOptimizedMap(
                measurements,
                optimized,
                static_cast<float>(options.map_voxel_leaf));
            const fs::path output_map = options.output_dir / "global_map_manual_imu.pcd";
            if (optimized_map->empty() ||
                pcl::io::savePCDFileBinary(output_map.string(), *optimized_map) != 0)
            {
                throw std::runtime_error("Failed to save optimized point cloud map.");
            }

            const fs::path output_trajectory_pcd = options.output_dir / "trajectory.pcd";
            SaveTrajectoryPcd(output_trajectory_pcd, measurements, optimized);
            map_built = true;
            map_build_elapsed_sec =
                std::chrono::duration<double>(std::chrono::steady_clock::now() - map_stage_start).count();
        }

        const fs::path output_png = options.output_dir / "pose_graph.png";
        GeneratePoseGraphImage(output_g2o, output_png);

        const fs::path output_report = options.output_dir / "manual_loop_report.json";
        SaveReportJson(
            output_report,
            options,
            constraints.size(),
            enabled_constraints,
            measurements,
            graph,
            optimized_map,
            map_built,
            map_build_elapsed_sec);

        if (options.constraints_csv.parent_path() != options.output_dir)
        {
            fs::copy_file(
                options.constraints_csv,
                options.output_dir / "manual_loop_constraints.csv",
                fs::copy_options::overwrite_existing);
        }

        std::cout << "[ManualLoopOptimize] Finished successfully." << std::endl;
        std::cout << "  output_dir: " << options.output_dir << std::endl;
        std::cout << "  enabled_constraints: " << enabled_constraints << std::endl;
        std::cout << "  pose_count: " << measurements.size() << std::endl;
        std::cout << "  factor_count: " << graph.size() << std::endl;
        std::cout << "  map_points: " << (optimized_map ? optimized_map->size() : 0) << std::endl;
        return EXIT_SUCCESS;
    }
    catch (const std::exception &e)
    {
        std::cerr << "[ManualLoopOptimize] ERROR: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
}
