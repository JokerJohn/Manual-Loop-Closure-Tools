# Troubleshooting

[English](Troubleshooting.md) | [中文](Troubleshooting-zh.md)

## GUI Opens but Point Clouds Look Wrong

Check:

- whether the selected session has the correct `g2o`, `TUM`, and `key_point_frame` files
- whether `source` and `target` are selected in the intended order
- whether the point-cloud mode is `Temporal Window` or `RS Spatial Submap`

## Python Optimizer Not Available

Run:

```bash
python scripts/check_env.py
```

and verify that Python GTSAM can be imported.

## C++ Backend Missing

This is expected for Python-only usage.

Only install the C++ backend if you explicitly need fallback or parity checks.

## Exported Map Too Sparse

Check the export map voxel in `Advanced`.

- `0.0` means no export downsampling
- values larger than `0.0` will reduce map density

## Session Browser Opens at Home

The GUI remembers the last successful session path. If it still resets unexpectedly, verify that the session was actually loaded successfully before reopening the file dialog.
