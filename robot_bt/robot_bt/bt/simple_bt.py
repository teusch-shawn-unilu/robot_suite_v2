import py_trees
from rclpy.node import Node


def create_tree():  # Leaf nodes
    success_node = py_trees.behaviours.Success(name="Always Succeed")
    failure_node = py_trees.behaviours.Failure(name="Always Fail")

    # Decorator
    invert_failure = py_trees.decorators.Inverter(name="Inverter", child=failure_node)

    # Composite
    sequence = py_trees.composites.Sequence(name="Simple Sequence", memory=False)
    sequence.add_children([success_node, invert_failure])

    return sequence


def bootstrap(ros_node: Node) -> py_trees.behaviour.Behaviour:
    tree = create_tree()
    return tree
