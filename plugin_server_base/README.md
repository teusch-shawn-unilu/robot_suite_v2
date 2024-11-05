# Plugin Server Base

The `plugin_server_base` package serves as a bridge between the `tello_bt` framework and any custom plugins you create. It allows you to develop plugins independently, without requiring direct use of `tello_bt` or the `py-trees` library (which manages the behavior tree).

Essentially, this package provides a ROS 2 node with a preconfigured ROS 2 service, enabling the behavior tree to trigger the execution (or "ticking") of your plugin.

## Getting Started

Using this package is straightforward and requires only two steps:

1. **Inherit from `plugin_server_base.PluginBase`:** Use this as the base class for your node instead of `rclpy.Node`.
2. **Define the `tick(self)` method:** Implement the `tick()` method in your node class. This method is essential; it will serve as the main loop for your plugin.

> [!NOTE]
> The `tick()` method functions like a loop, either called by the behavior tree or, if in standalone mode, called at a default rate of 30 Hz.

## Parameters

The following parameters can be configured in the `plugin_server_base` package:

| Parameter Name | Default Value | Description                                                                                           |
| -------------- | ------------- | ----------------------------------------------------------------------------------------------------- |
| `tick_rate`    | 30            | When in standalone mode, this parameter sets the rate (in Hz) at which the `tick()` method is called. |
| `standalone`   | true          | If set to `true`, the plugin runs in standalone mode, meaning it operates without the behavior tree.  |
