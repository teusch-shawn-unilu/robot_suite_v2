import importlib
import py_trees
from typing import Callable, Optional
from rclpy.node import Node
from rclpy.logging import get_logger


class BootstrapError(Exception):
    """
    Custom exception for bootstrap-related errors.
    """

    pass


def bootstrap_bt(
    file_name: str, module_name: Optional[str] = None
) -> Callable[[Node], py_trees.behaviour.Behaviour]:
    """
    Dynamically imports a module and retrieves the `bootstrap` function for a behavior tree.

    Args:
        file_name (str): Name of the file (without the `.py` extension) containing the
                        `bootstrap`.
        module_name (Optional[str]): Module/Package name from which it should try
            to import the file_name. If set to None, it will search on `robot_bt.bt`.
            In case your file is located under `pkg_name.sub_pkg.file.py`, you need to
            include `pkg_name.sub_pkg` as your module_name


    Returns:
        Optional[Callable[[Node], py_trees.behaviour.Behaviour]]:
            A callable function that accepts an `rclpy.node.Node` instance and returns
            a `py_trees.behaviour.Behaviour` object. Returns `None` if the module or function
            cannot be found.

    Raises:
        BootstrapError: If the file name contains a `.py` extension.
        ImportError: If the module does not contain a `bootstrap` function or cannot be imported.
    """
    logger = get_logger("robot_bt.bootstrap")

    if file_name.find(".py") != -1:
        logger.warning(f"File name `{file_name}` contains .py extension. Removing it.")
        file_name.removesuffix(".py")

    if module_name is None:
        module_name = f"robot_bt.bt.{file_name}"
    else:
        module_name = f"{module_name}.file_name"

    try:
        module = importlib.import_module(module_name)

        if hasattr(module, "bootstrap"):
            return module.bootstrap
        else:
            err_msg = f"Unable to find 'bootstrap' function on `{module_name}.py`."
            logger.error(err_msg)
            raise BootstrapError(err_msg)
    except ImportError as e:
        err_msg = f"Error while importing module {module_name}: {e}"
        logger.error(err_msg)
        raise BootstrapError(err_msg)
    except Exception as e:
        err_msg = f"Unknown error occured while importing module {module_name}: {e}"
        logger.error(err_msg)
        raise BootstrapError(err_msg)
