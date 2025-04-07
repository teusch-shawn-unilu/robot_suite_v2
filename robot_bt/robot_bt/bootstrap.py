import importlib
import py_trees
from typing import Callable, Optional
from rclpy.node import Node


class BootstrapError(Exception):
    """
    Custom exception for bootstrap-related errors.

    Raised when invalid input is provided to the `bootstrap_bt` function, such as
    a file name containing a `.py` extension.
    """

    pass


def bootstrap_bt(
    file_name: str,
) -> Optional[Callable[[Node], py_trees.behaviour.Behaviour]]:
    """
    Dynamically imports a module and retrieves the `bootstrap` function for a behavior tree.

    Args:
        file_name (str): Name of the file (without the `.py` extension) containing the
                         `bootstrap` function within the `tello_bt.bt` package.

    Returns:
        Optional[Callable[[Node], py_trees.behaviour.Behaviour]]:
            A callable function that accepts an `rclpy.node.Node` instance and returns
            a `py_trees.behaviour.Behaviour` object. Returns `None` if the module or function
            cannot be found.

    Raises:
        BootstrapError: If the file name contains a `.py` extension.
        ImportError: If the module does not contain a `bootstrap` function or cannot be imported.
    """
    if file_name.find(".py") != -1:
        raise BootstrapError("File name contains .py extension.")

    module_name = f"tello_bt.bt.{file_name}"
    try:
        module = importlib.import_module(module_name)

        if hasattr(module, "bootstrap"):
            return module.bootstrap
        else:
            raise ImportError(
                f"Module '{module_name}' does not have a 'bootstrap' function."
            )
    except ImportError as e:
        print(f"Error importing module: {e}")
        return None
