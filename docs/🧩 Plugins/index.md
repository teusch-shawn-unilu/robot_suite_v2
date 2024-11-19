# About Plugins

What is a plugin, you may ask? Simply put, a plugin is a ROS2 package designed to add new functionality that doesn't need to run constantly, unlike the `tello_driver`. Plugins are optional components that can be dynamically controlled by the behavior tree (`tello_bt`), enabling flexible and modular task execution.

The core idea of plugins is their ability to work together seamlessly, allowing for different types of functionality to be combined to achieve complex tasks. For example:
- A default plugin shipped with the system is a **hand gesture detector**, which interprets gestures and converts them into drone commands.
- Another example could be a **person follower plugin**, where the drone autonomously tracks and follows a person.

In the future, the selection of which plugin to run can be configured using a behavior tree and a configuration file, making the system even more adaptable.

---

## How To Create a Plugin

As mentioned, a plugin is just a ROS2 package. You can create a new package under the `plugins` folder or import your own pre-existing package.

When developing your plugin, you need to use the [plugin_server_base](../📦 Packages/plugin_server_base.md) package. This package provides a base class (`PluginBase`) for implementing plugins. Your node should look like this:

!!! danger

    You must implement the `tick` function. You can think of it as an inifite while loop. This function should not contain any inifinte loop inside!!

!!! example

    ```python
    from plugin_server_base.plugin_base import PluginBase
    from typing import Optional, Any
    
    class MyNode(PluginBase):
        def __init__(self, node_name: str):
            super().__init__(node_name)
    
            # Initialize your plugin here
    
        def tick(self, blackboard: Optional[dict[str, Any]] = None):
            # Implement the main logic of your plugin here
            pass
    ```
For a more detailed explanation of plugin_server_base, refer to its [documentation](../📦 Packages/plugin_server_base.md).

## Frequently Asked Questions


!!! question "Does my plugin need to have all its nodes based on `PluginBase`?"

    No. If you have a main node that handles input and produces output, and your other nodes depend on its data, you do not need to base all nodes on `PluginBase`. Only the node interacting with the behavior tree (`tello_bt`) requires this.


!!! question "When does my node strictly require the use of `PluginBase`?"

    If you want your node to be controlled by the behavior tree (`tello_bt`), you must base it on `PluginBase`. This ensures compatibility and control integration.

