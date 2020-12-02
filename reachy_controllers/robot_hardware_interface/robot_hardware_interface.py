from abc import ABC, abstractmethod
from typing import List, Dict


class RobotHardwareABC(ABC):
    @abstractmethod
    def get_joint_names(self) -> List[str]:
        ...

    @abstractmethod
    def get_joint_positions(self) -> List[float]:
        ...

    @abstractmethod
    def set_goal_positions(self, goal_positions: Dict[str, float]) -> None:
        ...

    @abstractmethod
    def set_compliance(self, compliances: Dict[str, bool]) -> bool:
        ...
