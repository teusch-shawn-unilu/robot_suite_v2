from abc import abstractmethod
from typing import Any, Optional
from numpy._typing import NDArray


class Model:
    @abstractmethod
    def predict(self, img: NDArray) -> Any:
        pass
