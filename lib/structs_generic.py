from dataclasses import dataclass
from abc import ABC, abstractmethod
from logging import Logger
from typing import Optional, List, Union

@dataclass
class FowardKinematicsDescription:
    """Forward Kinematics Parameters"""
    theta_1: float # rad
    theta_2: float # rad
    theta_3: float # rad
    theta_4: float # rad
    lambd: float # cm

@dataclass
class InverseKinematicsDescription:
    """Inverse Kinematics Parameters"""
    theta_r: float # rad
    theta_p: float # rad
    theta_j1: float # rad
    theta_j2: float # rad
    lambd: float # cm

class DriverInterfaceABC(ABC):
    """Class to receive commands from the pipeline"""
    def __init__(self, logger: Optional[Logger] = None) -> None:
        super().__init__()
        self.logger = logger

    @abstractmethod
    def drive(self, forward_kinematics_params: FowardKinematicsDescription) -> None:
        """Send drive control"""
        raise NotImplementedError()
    
    # @abstractmethod
    # def drive_inverse(self, inverse_kinematics_params: InverseKinematicsDescription) -> None:
    #     """Send inverse drive control, should probably NOT exist"""
    #     raise NotImplementedError()

class KinematicsManagerABC(ABC):
    """Class to manage kinematics, and transmit commands to driver interfaces"""
    def __init__(
            self,
            driver_interfaces: List[DriverInterfaceABC] = [],
            logger: Optional[Logger] = None
            ) -> None:
        super().__init__()
        self.logger = logger
        self.driver_interfaces = driver_interfaces
    
    def forward_kinematics(self, forward_kinematics_params: FowardKinematicsDescription) -> bool:
        """Compute forward kinematics and send drive command to all driver interfaces."""
        inv_params = self.compute_forward_kinematics(forward_kinematics_params)
        if inv_params is None: # Was seen as invalid !
            return False
        
        for driver in self.driver_interfaces: # Issue all drive commands
            driver.drive(forward_kinematics_params)
        return True

    def inverse_kinematics(self, inverse_kinematics_params: InverseKinematicsDescription) -> bool:
        """Compute inverse kinematics and send drive command to all driver interfaces."""
        fwd_params = self.compute_inverse_kinematics(inverse_kinematics_params)
        if fwd_params is None: # Was seen as invalid !
            return False

        for driver in self.driver_interfaces: # Issue all drive commands
            driver.drive(fwd_params)
        return True

    @abstractmethod
    def compute_forward_kinematics(self, forward_kinematics_params: FowardKinematicsDescription) -> Union[InverseKinematicsDescription, None]:
        """Compute forward kinematics. If invalid, returns None."""
        raise NotImplementedError()
    
    @abstractmethod
    def compute_inverse_kinematics(self, inverse_kinematics_params: InverseKinematicsDescription) -> Union[FowardKinematicsDescription, None]:
        """Compute inverse kinematics. If invalid, returns None."""
        raise NotImplementedError()
    
class UserInterfaceABC(ABC):
    """Class to send commands to the pipeline"""

    def __init__(self, kinematics_manager: KinematicsManagerABC, logger: Optional[Logger] = None) -> None:
        super().__init__()
        self.logger = logger
        self.kinematics_manager = kinematics_manager

        