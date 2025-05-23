import py_trees


class CanRunPlugin(py_trees.behaviour.Behaviour):
    """A helper class to check whether a plugin is selected to run.

    Uses the blackboard with name PluginsBlackboard, where it has
    a key selected_plugin. If the value is the plugin_name, it returns
    SUCCESS and FAILURE otherwise.
    """

    def __init__(self, name: str, plugin_name: str):
        super().__init__(name)
        self.plugin_name = plugin_name

    def setup(self):  # type: ignore
        self.blackboard = py_trees.blackboard.Client(name="PluginsBlackboard")
        self.blackboard.register_key(
            "selected_plugin", access=py_trees.common.Access.READ
        )

    def update(self):
        selected_plugin = self.blackboard.selected_plugin

        if selected_plugin == self.plugin_name:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE
