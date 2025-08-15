import time
import numpy as np
from logging import Logger
from matplotlib.backend_bases import Event
from typing import Optional, Union, Tuple
from .structs_generic import UserInterfaceABC, DriverInterfaceABC, KinematicsManagerABC, FowardKinematicsDescription, InverseKinematicsDescription
from .lib_serial import ThreadedSerialHandler
from .opensurgbot_kinevizu.kinevisu.kinevisu import OpensurgbotViz
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

try:
    from typing import override
except ImportError:
    def override(func): # dummy decorator for older python versions
        return func

class KinematicsManager(KinematicsManagerABC):
    """Class to manage kinematics, and transmit commands to driver interfaces"""
    # theta_1_offset, theta_2_offset, theta_3_offset, theta_4_offset = 0.0, np.radians(-6), np.radians(7), np.radians(-7) # rad, meaning: at angles = +offset, neutral position of the tip
    theta_1_offset, theta_2_offset, theta_3_offset, theta_4_offset = 0.0, 0.0, 0.0, 0.0
    @override
    def compute_forward_kinematics(self, forward_kinematics_params: FowardKinematicsDescription) -> Union[InverseKinematicsDescription, None]:
        """Compute forward kinematics. If invalid, returns None.""" 
        # TODO: use matrix like for inverse
        theta_1_offset, theta_2_offset, theta_3_offset, theta_4_offset = OpensurgbotViz.theta_1_offset, OpensurgbotViz.theta_2_offset, OpensurgbotViz.theta_3_offset, OpensurgbotViz.theta_4_offset
        theta_1, theta_2, theta_3, theta_4 = forward_kinematics_params.theta_1, forward_kinematics_params.theta_2, forward_kinematics_params.theta_3, forward_kinematics_params.theta_4
        theta_1, theta_2, theta_3, theta_4 = theta_1 - theta_1_offset, theta_2 - theta_2_offset, theta_3 - theta_3_offset, theta_4 - theta_4_offset # Apply offsets

        # FORWARD KINEMATICS
        lambd = forward_kinematics_params.lambd
        pitch, roll, jaw1, jaw2 = 0., 0., 0., 0.

        roll = 270/170*theta_2
        
        rel_matrix = np.array([
            [-80/80       , 0       , 0     ],
            [-70/80*110/90, 110/90  , 0     ],
            [-70/80*110/90, 0       , 110/90],
        ])
        pitch, jaw1, jaw2 = rel_matrix @ np.array([theta_1, theta_3, theta_4])

        inv_params = InverseKinematicsDescription(roll, pitch, jaw1, jaw2, lambd)
        ret = self._condition_check(forward_kinematics_params, inv_params)
        if self.logger:
            self.logger.debug(f"Computed forward kinematics: valid={ret}, fwd={forward_kinematics_params}, inv={inv_params}")
        if ret is False: # Out of bounds
            return None
        return inv_params
    
    @override
    def compute_inverse_kinematics(self, inverse_kinematics_params: InverseKinematicsDescription) -> Union[FowardKinematicsDescription, None]:
        """Compute inverse kinematics. If invalid, returns None."""
        roll, pitch, jaw1, jaw2 = inverse_kinematics_params.theta_r, inverse_kinematics_params.theta_p, inverse_kinematics_params.theta_j1, inverse_kinematics_params.theta_j2

        # INVERSE KINEMATICS
        lambd = inverse_kinematics_params.lambd
        theta_2 = 170/270*roll

        out_vec3 = np.array([pitch, jaw1, jaw2])
        rel_matrix = np.array([
            [-80/80       , 0       , 0     ],
            [-70/80*110/90, 110/90  , 0     ],
            [-70/80*110/90, 0       , 110/90],
        ])
        in_vec3 = np.linalg.solve(rel_matrix, out_vec3)
        theta_1, theta_3, theta_4 = in_vec3[0], in_vec3[1], in_vec3[2]

        # Check conditions
        fwd_params = FowardKinematicsDescription(theta_1, theta_2, theta_3, theta_4, lambd)
        ret = self._condition_check(fwd_params, inverse_kinematics_params)
        if self.logger:
            self.logger.debug(f"Computed inverse kinematics: valid={ret}, fwd={fwd_params}, inv={inverse_kinematics_params}")
        if ret is False: # Out of bounds
            return None
        return fwd_params

    def _condition_check(self, fwd_params: FowardKinematicsDescription, inv_params: InverseKinematicsDescription) -> bool:
        """True if in bounds, False otherwise."""
        roll, pitch, jaw1, jaw2 = inv_params.theta_r, inv_params.theta_p, inv_params.theta_j1, inv_params.theta_j2
        theta_1, _, theta_3, theta_4 = fwd_params.theta_1, fwd_params.theta_2, fwd_params.theta_3, fwd_params.theta_4
        theta_1_offset, theta_2_offset, theta_3_offset, theta_4_offset = self.theta_1_offset, self.theta_2_offset, self.theta_3_offset, self.theta_4_offset

        # Trivial boundary conditions
        if ((np.degrees(pitch) < -80 or np.degrees(pitch) > 80) or
            (np.degrees(roll) < -270 or np.degrees(roll)) > 270 or
            (np.degrees(jaw1) < -110 or np.degrees(jaw1)) > 110 or
            (np.degrees(jaw2) < -110 or np.degrees(jaw2)) > 110
            ):
            return False

        # Variable boundary conditions
        delta_boundary = 0
        if theta_4 - theta_4_offset < np.radians(-90):
            delta_boundary += 80/60*(theta_3 - theta_3_offset + np.radians(90))
        elif np.radians(90) < theta_3 - theta_3_offset:
            delta_boundary += 80/60*(theta_4 - theta_4_offset - np.radians(90))
        if ((theta_3 - theta_3_offset - (theta_4 - theta_4_offset) > 0.01) or
            (theta_1 - theta_1_offset < -np.radians(80) + delta_boundary or theta_1 - theta_1_offset > np.radians(80) + delta_boundary)
         ):
            return False

        return True
    
class UserInterface3DViz(UserInterfaceABC):
    """User interface with 3D visualization."""

    def __init__(self, kinematics_manager: KinematicsManagerABC, logger: Union[Optional[Logger], None] = None) -> None:
        super().__init__(kinematics_manager, logger)
        self.viz = OpensurgbotViz()

        # === Add custom button ===
        self.viz.ext_create_user_button("Reconnect", self._on_reconnect_click, width=0.15, x=0.8)
        self.viz.ext_create_user_button("Send command (as inverse!)", self._on_send_click, width=0.21)
        self.viz.ext_create_user_button("Random position", self._on_random_click, width=0.18, x=0.43)

        # === Add CAN slider ===
        can_slider_ax = plt.axes((0.1, 0.3, 0.01, 0.3), facecolor="blue")  # Position: [left, bottom, width, height]
        self.can_slider = Slider(can_slider_ax, '$\\lambda$ (cm)', -6, 6, valinit=0, orientation="vertical")
        self.viz.reset_button.on_clicked(self._reset_can_slider)

    def _reset_can_slider(self, event: Event) -> None:
        self.can_slider.set_val(-6)        

    def _on_reconnect_click(self, event: Event) -> None: # What to do on user button click
        for interface in self.kinematics_manager.driver_interfaces:
            if isinstance(interface, DriverInterface): # Which has serial connection
                interface.serial.stop()
                time.sleep(1)
                interface.serial.start_threaded()

    def _on_send_click(self, event: Event) -> None: # What to do on user button click
        if self.logger:
            self.logger.debug("User button Send click detected")

        # Retrieve **inverse** parameters 
        inv_params = InverseKinematicsDescription(
            theta_r=self.viz.roll,
            theta_p=self.viz.pitch,
            theta_j1=self.viz.jaw1,
            theta_j2=self.viz.jaw2,
            lambd=self.can_slider.val
        )
        if not self.kinematics_manager.inverse_kinematics(inv_params):
            if self.logger:
                self.logger.info(f"WARNING: invalid kinematic parameters inv_params={inv_params}")
    
    def _on_random_click(self, event: Event) -> None: # What to do on user button click
        if self.logger:
            self.logger.debug("User button Random click detected")
        while True:
            sample = np.random.random(4)
            theta_1 = np.radians((sample[0] - 0.5)*2 * (78.5 - 5)) # 5° border margin
            theta_2 = np.radians((sample[0] - 0.5)*2 * (170. - 5)) # 5° border margin
            theta_3 = np.radians((sample[0] - 0.5)*2 * (150. - 5)) # 5° border margin
            theta_4 = np.radians((sample[0] - 0.5)*2 * (150. - 5)) # 5° border margin
            lambd = (np.random.random() - 0.5)*2 * 6
            fwd_params = FowardKinematicsDescription(theta_1, theta_2, theta_3, theta_4, lambd)
            rev_params = self.kinematics_manager.compute_forward_kinematics(fwd_params)
            if rev_params is not None:
                self.viz.sliders[0].set_val(np.degrees(rev_params.theta_r))
                self.viz.sliders[1].set_val(np.degrees(rev_params.theta_p))
                self.viz.sliders[2].set_val(np.degrees(rev_params.theta_j1))
                self.viz.sliders[3].set_val(np.degrees(rev_params.theta_j2))
                self.viz.sliders[4].set_val(np.degrees(fwd_params.theta_1))
                self.viz.sliders[5].set_val(np.degrees(fwd_params.theta_2))
                self.viz.sliders[6].set_val(np.degrees(fwd_params.theta_3))
                self.viz.sliders[7].set_val(np.degrees(fwd_params.theta_4))
                self.can_slider.set_val(lambd)
                self.viz.on_angle_slider_change(0)
                self.viz.on_control_slider_change(0)
                break


    def run(self, is_blocking: bool = True) -> None:
        """Display the figure.
        
        Args:
            is_blocking (bool, optional): If False, is non blocking by enabling matplotlib interacting mode, blocking if True. Defaults to True"""
        if self.logger:
            self.logger.debug(f"Running UserInterface3DViz with is_blocking={is_blocking}")
        self.viz.run(is_blocking)

class DriverInterface(DriverInterfaceABC):
    serial_baudrate = 115200
    servo_max_speed = 100.0 # deg per s
    ZDT_max_speed = 2 # cm per s # Max speed ~6cm /s

    def __init__(self, serial_port: Optional[str] = None, serial_timeout: float = 0.1, serial_wait_time: float = 0.1, logger: Optional[Logger] = None) -> None:
        super().__init__(logger)
        self.serial = ThreadedSerialHandler(port=serial_port,baudrate=DriverInterface.serial_baudrate, timeout=serial_timeout, wait_time=serial_wait_time, logger=logger)
    
    @override
    def drive(self, forward_kinematics_params: FowardKinematicsDescription, timeout: float = 10.0) -> None:
        """Send drive control and wait for the movement to finish. Will use: command id 101 - opensurgbot drive all blocking"""
        # ==== Building the message ====
        msg = '101'
        # === servos ===
        th1, th2, th3, th4 = forward_kinematics_params.theta_1, forward_kinematics_params.theta_2, forward_kinematics_params.theta_3, forward_kinematics_params.theta_4
        msg += f",{-np.degrees(th1):.2f}" # Invert sign: negative of dial angle convention
        msg += f",{-np.degrees(th2):.2f}" # Invert sign: negative of dial angle convention
        msg += f",{-np.degrees(th3):.2f}" # Invert sign: negative of dial angle convention
        msg += f",{-np.degrees(th4):.2f}" # Invert sign: negative of dial angle convention
        msg += f",{self.servo_max_speed:.2f}"
        
        # === ZDT Linear actuator ===
        lambd = forward_kinematics_params.lambd
        steps, speed = self._get_zdt_args(lambd, self.ZDT_max_speed)
        msg += f",1,{steps},{speed},0,{round(timeout*1000)}"

        # ==== Sending the message ====
        self.serial.queue_message(msg)
        if self.logger:
            self.logger.debug(f"Queued serial message msg={msg}")
        
        # ==== Waiting for acknowledgment ====
        cmd_begin_time = time.time()
        acked = False
        while time.time() - cmd_begin_time < timeout: # Wait for ack*
            recv = self.serial.receive_line(timeout=timeout)
            if recv and recv.startswith("250"):
                acked = True
                if self.logger:
                    self.logger.debug(f"Received message acknowledgment: {recv}")
                break
        if not acked:
            if self.logger:
                self.logger.warning(f"Did not receive message acknowledgment in {timeout} seconds")
            return
        
        done = False
        while time.time() - cmd_begin_time < timeout: # Wait for done
            recv = self.serial.receive_line(timeout=timeout)
            if recv and recv.startswith("249"):
                done = True
                break
        if not done:
            if self.logger:
                self.logger.warning(f"Did not receive command success message in {timeout} seconds")
            return

    def _get_zdt_args(self, alg_dist: float, max_speed: float) -> Tuple[int,int]:
        """Convert parameters
         Args:
            alg_idst (float): in cm
            max_speed (float): in cm/s"""
        ZDTStepPerTurn = 0.1 # cm (screw step)
        ZDTStepRot = 1.8 # degrees (rotational step)
        ZDTMicrostep = 16 # Multiplier
        normdist = (alg_dist+6) # Move the 0 to the edge

        n_step = round(normdist * (360/ZDTStepPerTurn) * (1/(ZDTStepRot/ZDTMicrostep)))
        speed_rpm = round(max_speed * 60 / ZDTStepPerTurn)

        return (n_step, speed_rpm)