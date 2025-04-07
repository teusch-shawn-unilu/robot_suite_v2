import py_trees
import subprocess


class IsDroneConnected(py_trees.behaviour.Behaviour):
    _ip_address: str = "192.168.10.1"

    def setup(self):  # type: ignore
        pass

    def update(self) -> py_trees.common.Status:
        ret = subprocess.run(
            ["ping", "-c 1", "-W 0.1", self._ip_address],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

        return (
            py_trees.common.Status.SUCCESS
            if ret.returncode == 0
            else py_trees.common.Status.FAILURE
        )
