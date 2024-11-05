# Tello Behavior Tree

The `tello_bt` package implements a behavior tree (BT) for managing and executing plugins that define various behaviors for the Tello drone. This package uses the `py-trees` library to structure and run the behavior tree.

## What is a Behavior Tree?

A behavior tree is a model that organizes defined behaviors into a tree structure, allowing them to be executed (or "ticked") in each iteration. While similar to a Finite State Machine (FSM), a behavior tree supports more complex scenarios with greater flexibility and simplicity.

## Usage

To use this package, ensure that:

- Your plugins inherit from `plugin_server_base.PluginBase`.
- You define a `tick()` method in each plugin that specifies its behavior.

The `tello_bt` package will manage the lifecycle of the behavior tree and control when each plugin's `tick()` method is called, based on the behavior tree's structure and conditions.

For detailed examples and configuration, refer to the plugin documentation and sample behavior trees provided.
