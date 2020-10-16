import pandas as pd

import config as cfg
from driver import RobotDriver

from encoders import EncoderController
from pid import PIDController


if __name__ == '__main__':
    encoder_controller = EncoderController(left_motor_pin_a=cfg.LEFT_MOTOR_PIN_A,
                                           left_motor_pin_b=cfg.LEFT_MOTOR_PIN_B,
                                           right_motor_pin_a=cfg.RIGHT_MOTOR_PIN_A,
                                           right_motor_pin_b=cfg.RIGHT_MOTOR_PIN_B,
                                           encoder_resolution=cfg.ENCODER_RESOLUTION)

    pid_controller = PIDController(proportional_coef=1,
                                   integral_coef=0,
                                   derivative_coef=0,
                                   set_point=2,
                                   windup_guard=20,
                                   current_time=None)

    rd = RobotDriver(wheel_diameter=cfg.WHEEL_DIAMETER_MM,
                     wheelbase=cfg.WHEELBASE_MM,
                     max_angular_velocity=cfg.MAX_ANGULAR_VELOCITY_RPM,
                     encoder_controller=encoder_controller,
                     pid_controller=pid_controller)

    rd.drive_forward(0.2, 0.5)
    left_velocities = rd.get_encoder_controller().get_left_encoder().get_velocity_records()
    right_velocities = rd.get_encoder_controller().get_right_encoder().get_velocity_records()

    # left_counters = rd.get_encoder_controller().get_left_encoder().get_counter_records()
    # right_counters = rd.get_encoder_controller().get_right_encoder().get_counter_records()

    left_timestamps = rd.get_encoder_controller().get_left_encoder().get_timestamp_velocity_records()
    right_timestamps = rd.get_encoder_controller().get_right_encoder().get_timestamp_velocity_records()

    records_left = pd.DataFrame({'velocity': left_velocities,
                                 'timestamp': left_timestamps})
    records_right = pd.DataFrame({'velocity': right_velocities,
                                  'timestamp': right_timestamps})
    records_left.to_csv('left.csv')
    records_right.to_csv('right.csv')

    a_left = pd.DataFrame({'encoder': rd.get_encoder_controller().get_left_encoder().get_a_records()})
    a_right = pd.DataFrame({'encoder': rd.get_encoder_controller().get_right_encoder().get_a_records()})
    a_left.to_csv('aleft.csv')
    a_right.to_csv('aright.csv')