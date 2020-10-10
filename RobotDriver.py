import time
from typing import Tuple, Union

import numpy as np
from adafruit_motorkit import MotorKit


MM_IN_M = 1000
S_IN_MIN = 60
ONE_ROT_DEG = 360


class RobotDriver:
    def __init__(self,
                 wheel_diameter: float,
                 wheelbase: float,
                 encoder_resolution: int,
                 max_angular_velocity: float) -> None:
        """
        RobotDriver class.
        Args:
            wheel_diameter: (float) Diameter of wheel in millimeters.
            wheelbase: (float) Distance between two wheels in millimeters.
            encoder_resolution: (int) Number od distinguishable steps during one wheel revolution.
            max_angular_velocity: (float) Maximal angular velocity of motor in RPM (rotations per minute).

        """
        self.__wheel_diameter_mm = wheel_diameter
        self.__wheelbase_mm = wheelbase
        self.__encoder_resolution = encoder_resolution
        self.__max_angular_velocity_rpm = max_angular_velocity

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

        self.run_motor_right(scaled_velocity)
        self.run_motor_left(scaled_velocity)
        time.sleep(duration_s)
        self.stop()

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
            angle_deg: (float) Angle in degrees.
            linear_velocity_ms: (float) Linear velocity in m/s.

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

    def convert_encoder_steps_to_distance(self, encoder_steps: Union[int, float]) -> float:
        """
        Method converts steps of encoder to real distance in meters.

        Args:
            encoder_steps: (Union[int, float]) Number of encoder steps.

        Returns: (float) Distance in meters.

        """
        return self.__circumference_m * encoder_steps / self.__encoder_resolution

    def convert_distance_to_encoder_steps(self, distance_m: float) -> Tuple[int, float]:
        """
        Method converts given distance in meters to full encoder steps. After rounding down steps, an underestimation
        may happen. Therefore, apart from result of conversion, distance error in meters is returned.

        Args:
            distance_m: (float) Distance in meters.

        Returns: (Tuple[int, float]) Number of encoder steps and distance error occurring during the rounding process.

        """
        encoder_steps = self.__encoder_resolution * distance_m / self.__circumference_m
        rounded_encoder_steps = int(encoder_steps)
        distance_difference_m = self.convert_encoder_steps_to_distance(encoder_steps - rounded_encoder_steps)
        return rounded_encoder_steps, distance_difference_m

    def convert_encoder_steps_to_angle(self, encoder_steps: Union[int, float]) -> float:
        """
        Method converts steps of encoder to rotation angle in degrees (not radians!).

        Args:
            encoder_steps: (Union[int, float]) Number of encoder steps.

        Returns: Angle in degrees.

        """
        return 360 * encoder_steps / self.__encoder_resolution

    def convert_angle_to_encoder_steps(self, angle_deg: float) -> Tuple[int, float]:
        """
        Method converts given angle in degrees (not radians!) to full encoder steps. After rounding down steps, an
        underestimation may happen. Therefore, apart from result of conversion, angle error in degrees is returned.

        Args:
            angle_deg: (float) Angle in degrees.

        Returns: (Tuple[int, float]) Number of encoder steps and angle error occurring during the rounding process.

        """

        encoder_steps = self.__encoder_resolution * angle_deg / ONE_ROT_DEG
        rounded_encoder_steps = int(encoder_steps)
        angle_difference_deg = self.convert_encoder_steps_to_angle(encoder_steps - rounded_encoder_steps)
        return rounded_encoder_steps, angle_difference_deg

    def convert_angular_to_linear_velocity(self, angular_velocity_rads: float) -> float:
        """
        Method converts angular velocity in radians/s to linear velocity in m/s.

        Args:
            angular_velocity_rads: (float) Angular velocity in radians/s, calculated with formula w=2*PI*f.

        Returns: (float) Linear velocity in m/s, calculated with formula v=2*PI*f*R.

        """
        return angular_velocity_rads * self.__wheel_diameter_mm / (2 * MM_IN_M)

    def convert_linear_to_angular_velocity(self, linear_velocity_ms: float) -> float:
        """
        Method converts linear velocity in m/s to angular velocity in radians/s.

        Args:
            linear_velocity_ms: (float) Linear velocity in m/s, calculated with formula v=2*PI*f*R.

        Returns: (float) Angular velocity in radians/s, calculated with formula w=2*PI*f.

        """
        return linear_velocity_ms * (2 * MM_IN_M) / self.__wheel_diameter_mm


if __name__ == '__main__':
    print('start')
    kit = MotorKit(0x40)
    print(kit.motor1)
    print(type(kit.motor2))
