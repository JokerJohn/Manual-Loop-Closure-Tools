# GUI Workflow

[English](GUI-Workflow.md) | [中文](GUI-Workflow-zh.md)

## Main Layout

The GUI is divided into four main areas:

- trajectory panel
- point-cloud review panel
- right-side control tabs (`Summary` / `Advanced`)
- bottom tabs (`Graph Changes` / `Execution Log`)

## Trajectory Panel

- `Nodes` selects source / target node pairs.
- `Edges` inspects or revises existing loop edges.
- `Working` and `Original` compare the edited graph against the baseline graph.
- `Ghost` overlays the other trajectory as a lightweight reference.

## Point Cloud Review

- `Preview`, `Final`, and `Compare` control the display mode.
- `Top`, `Side-Y`, and `Side-X` are camera presets.
- In edit mode, source is the only editable object; target stays fixed.

## Right Control Tabs

### Summary

- summary cards
- delta
- registration
- actions

### Advanced

- shared variance settings
- backend selection
- export map voxel

## Bottom Tabs

- `Graph Changes` records accepted manual edits and disabled/restored loop actions.
- `Execution Log` stores optimization and GICP logs.
