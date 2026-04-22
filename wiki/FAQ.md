# FAQ

[English](https://github.com/JokerJohn/Manual-Loop-Closure-Tools/wiki/FAQ) | [中文](https://github.com/JokerJohn/Manual-Loop-Closure-Tools/wiki/FAQ-zh)

## Is ROS required?

No for normal use.

The standalone tool is designed around a Python-first workflow. ROS / catkin are only needed if you want to keep the legacy C++ optimizer as a fallback.

## Why are there both `Original` and `Working` trajectories?

`Original` is the immutable baseline.

`Working` is the graph currently being edited and re-optimized.

## Does `Replace` overwrite the original edge?

Not in the input graph.

It replaces the selected edge only inside the working session semantics.

## Why is Python slower than C++ in parity tests?

The Python backend prioritizes easier deployment and GUI integration. The current parity results show much simpler installation at the cost of higher optimization runtime on the tested sessions.

## Why does a PALoc session show one more g2o vertex than TUM or PCD frames?

This usually comes from the PALoc export path, not from the GUI.

Some older PALoc exports wrote the full pose graph but missed the last keyframe when saving `optimized_poses_tum.txt` and `key_point_frame/*.pcd`.

The GUI now trims a simple trailing unmatched g2o vertex automatically so these sessions can still be loaded, but regenerating the PALoc result after fixing the PALoc save path is still recommended.

## Where should I start reading?

Start from:

1. [Quick Start](https://github.com/JokerJohn/Manual-Loop-Closure-Tools/wiki/Quick-Start)
2. [GUI Workflow](https://github.com/JokerJohn/Manual-Loop-Closure-Tools/wiki/GUI-Workflow)
3. [Graph Editing](https://github.com/JokerJohn/Manual-Loop-Closure-Tools/wiki/Graph-Editing)
4. [Troubleshooting](https://github.com/JokerJohn/Manual-Loop-Closure-Tools/wiki/Troubleshooting)
