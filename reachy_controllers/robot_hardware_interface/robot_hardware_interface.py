from abc import ABC, abstractmethod
from typing import List


class RobotHardwareABC(ABC):
    @abstractmethod
    def get_joint_names(self) -> List[str]:
        ...

    @abstractmethod
    def get_joint_positions(self) -> List[float]:
        ...
