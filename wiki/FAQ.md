# FAQ

[English](FAQ.md) | [中文](FAQ-zh.md)

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

## Where should I start reading?

Start from:

1. [Quick Start](Quick-Start.md)
2. [GUI Workflow](GUI-Workflow.md)
3. [Graph Editing](Graph-Editing.md)
4. [Troubleshooting](Troubleshooting.md)
