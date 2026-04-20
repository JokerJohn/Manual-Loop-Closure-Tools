# Graph Editing

[English](Graph-Editing.md) | [中文](Graph-Editing-zh.md)

## Editing Philosophy

The tool distinguishes between:

- adding a new manual loop
- replacing an existing loop edge
- disabling an existing loop edge
- restoring a disabled edge

## Add Manual Loop

Use this when the graph does not already contain the constraint you want.

Workflow:

1. pick a source / target pair
2. refine the initial pose if necessary
3. run GICP
4. verify the result in the viewer
5. click `Add`

## Replace Existing Loop

Use this when an existing loop edge is present but unreliable.

Workflow:

1. switch to `Edges`
2. select the existing loop
3. run GICP from the current preview seed
4. click `Replace`

Internally this keeps the original graph untouched, disables the selected existing loop in the working graph, and applies the new manual result instead.

## Disable / Restore

- `Disable` removes an existing loop from the working graph only.
- `Restore` brings the disabled loop back into the working graph.

## Export Behavior

Export is based on the working graph state after optimization.

This means:

- disabled loops are excluded
- replacement loops are exported through the accepted working graph state
- original input files are not overwritten
