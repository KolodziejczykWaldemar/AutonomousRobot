import time
from typing import Tuple, Union

import numpy as np
from adafruit_motorkit import MotorKit

from encoders import EncoderController
from pid import PIDController

MM_IN_M = 1000
S_IN_MIN = 60
ONE_ROT_DEG = 360


class RobotDriver:
    def __init__(self,
                 wheel_diameter: float,
                 wheelbase: float,
                 max_angular_velocity: float,
                 encoder_controller: EncoderController,
                 pid_controller: PIDController) -> None:
        """
        RobotDriver class.
        Args:
            wheel_diameter(float): Diameter of wheel in millimeters.
            wheelbase(float): Distance between two wheels in millimeters.
            max_angular_velocity(float): Maximal angular velocity of motor in RPM (rotations per minute).
            encoder_controller(EncoderController):
            pid_controller(PIDController):

        """
        self.__wheel_diameter_mm = wheel_diameter
        self.__wheelbase_mm = wheelbase
        self.__max_angular_velocity_rpm = max_angular_velocity

        self.__encoder_controller = encoder_controller
        self.__pid_controller = pid_controller

        kit = MotorKit(0x40)
        self.__motor_right = kit.motor2 # reversed direction
        self.__motor_left = kit.motor1 # proper direction

        self.__circumference_m = np.pi * self.__wheel_diameter_mm / MM_IN_M
        self.__max_linear_velocity_rads = 2 * np.pi * self.__max_angular_velocity_rpm / S_IN_MIN
        self.__max_linear_velocity_ms = self.convert_angular_to_linear_velocity(self.__max_linear_velocity_rads)
        print(self.__max_linear_velocity_ms)

    def run_motor_left(self, scaled_velocity):
        self.__motor_left.throttle = scaled_velocity

    def run_motor_right(self, scaled_velocity):
        self.__motor_right.throttle = - scaled_velocity

    def stop(self) -> None:
        self.run_motor_right(0)
        self.run_motor_left(0)

    def drive_forward(self,
                      linear_velocity_ms: float,
                      distance_m: float) -> None:
        duration_s = distance_m / linear_velocity_ms
        scaled_velocity = linear_velocity_ms / self.__max_linear_velocity_ms

        encoder_steps, _ = self.convert_distance_to_encoder_steps(distance_m, round_steps=False)
        velocity_sts = encoder_steps / duration_s
        # TODO finish it!

        self.run_motor_right(scaled_velocity)
        self.run_motor_left(scaled_velocity)

        start = time.time()
        while time.time() - start < duration_s:
            self.__encoder_controller.update_counters()
        # time.sleep(duration_s)
        self.stop()
        print(time.time())
        breaking_time = 1
        while time.time() - start < breaking_time:
            self.__encoder_controller.update_counters()

    def drive_backward(self,
                       linear_velocity_ms: float,
                       distance_m: float) -> None:
        # TODO
        pass

    def turn(self,
             angle_deg: float,
             linear_velocity_ms: float) -> None:
        """
        Method for turning left and right by given angle wih linear velocity.
        Args:
            angle_deg(float): Angle in degrees.
            linear_velocity_ms(float): Linear velocity in m/s.

        """
        distance_m = self.__wheelbase_mm / MM_IN_M * np.pi * angle_deg / ONE_ROT_DEG

        duration_s = distance_m / linear_velocity_ms
        scaled_velocity = linear_velocity_ms / self.__max_linear_velocity_ms

        self.run_motor_right(scaled_velocity)
        self.run_motor_left(-scaled_velocity)
        time.sleep(duration_s)
        self.stop()

    def accelerate_up(self,
                      acceleration_ms2: float,
                      final_linear_velocity_ms: float) -> None:
        # TODO
        pass

    def accelerate_down(self,
                        deceleration_ms2: float,
                        final_linear_velocity_ms: float) -> None:
        # TODO
        pass

    def get_encoder_controller(self):
        return self.__encoder_controller

    def convert_encoder_steps_to_distance(self, encoder_steps: Union[int, float]) -> float:
        """
        Method converts steps of encoder to real distance in meters.

        Args:
            encoder_steps(Union[int, float]): Number of encoder steps.

        Returns:
            float: Distance in meters.

        """
        return self.__circumference_m * encoder_steps / self.__encoder_controller.get_encoder_resolution()

    def convert_distance_to_encoder_steps(self,
                                          distance_m: float,
                                          round_steps: bool = False) -> Tuple[Union[int, float], float]:
        """
        Method converts given distance in meters to encoder steps.
        If argument round_steps = True, full steps are returned. After rounding down steps, an underestimation
        may happen. Therefore, apart from result of conversion, distance error in meters is returned.

        Args:
            distance_m(float): Distance in meters.
            round_steps(bool): Flag for rounding down encoder steps, by default False.

        Returns:
            Tuple[Union[int, float], float]: Number of encoder steps and distance error occurring during the rounding process.

        """
        encoder_steps = self.__encoder_controller.get_encoder_resolution() * distance_m / self.__circumference_m
        if round_steps:
            rounded_encoder_steps = int(encoder_steps)
            distance_difference_m = self.convert_encoder_steps_to_distance(encoder_steps - rounded_encoder_steps)
            return rounded_encoder_steps, distance_difference_m
        else:
            return encoder_steps, 0.0

    def convert_encoder_steps_to_angle(self, encoder_steps: Union[int, float]) -> float:
        """
        Method converts steps of encoder to rotation angle in degrees (not radians!).

        Args:
            encoder_steps(Union[int, float]): Number of encoder steps.

        Returns:
            float: Angle in degrees.

        """
        return 360 * encoder_steps / self.__encoder_controller.get_encoder_resolution()

    def convert_angle_to_encoder_steps(self,
                                       angle_deg: float,
                                       round_steps: bool = False) -> Tuple[Union[int, float], float]:
        """
        Method converts given angle in degrees (not radians!) to encoder steps.

        If argument round_steps = True, full steps are returned. After rounding down steps, an
        underestimation may happen. Therefore, apart from result of conversion, angle error in degrees is returned.

        Args:
            angle_deg(float): Angle in degrees.
            round_steps(bool): Flag for rounding down encoder steps, by default False.

        Returns:
            Tuple[Union[int, float], float]: Number of encoder steps and angle error occurring during the rounding process.

        """

        encoder_steps = self.__encoder_controller.get_encoder_resolution() * angle_deg / ONE_ROT_DEG
        if round_steps:
            rounded_encoder_steps = int(encoder_steps)
            angle_difference_deg = self.convert_encoder_steps_to_angle(encoder_steps - rounded_encoder_steps)
            return rounded_encoder_steps, angle_difference_deg
        else:
            return encoder_steps, 0.0

    def convert_angular_to_linear_velocity(self, angular_velocity_rads: float) -> float:
        """
        Method converts angular velocity in radians/s to linear velocity in m/s.

        Args:
            angular_velocity_rads(float): Angular velocity in radians/s, calculated with formula w=2*PI*f.

        Returns:
            float: Linear velocity in m/s, calculated with formula v=2*PI*f*R.

        """
        return angular_velocity_rads * self.__wheel_diameter_mm / (2 * MM_IN_M)

    def convert_linear_to_angular_velocity(self, linear_velocity_ms: float) -> float:
        """
        Method converts linear velocity in m/s to angular velocity in radians/s.

        Args:
            linear_velocity_ms(float): Linear velocity in m/s, calculated with formula v=2*PI*f*R.

        Returns:
            float: Angular velocity in radians/s, calculated with formula w=2*PI*f.

        """
        return linear_velocity_ms * (2 * MM_IN_M) / self.__wheel_diameter_mm


if __name__ == '__main__':
    print('start')
    kit = MotorKit(0x40)
    print(kit.motor1)
    print(type(kit.motor2))
