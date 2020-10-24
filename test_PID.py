import time

import numpy as np
import pandas as pd
from adafruit_motorkit import MotorKit

import config as cfg
from encoders import Encoder
from pid import PIDController


if __name__ == '__main__':
    left_encoder = Encoder(motor_pin_a=cfg.LEFT_MOTOR_PIN_A,
                           motor_pin_b=cfg.LEFT_MOTOR_PIN_B,
                           velocity_averaging_length=3)

    pid_controller = PIDController(proportional_coef=cfg.PID_KP,
                                   integral_coef=cfg.PID_KI,
                                   derivative_coef=cfg.PID_KD,
                                   windup_guard=cfg.PID_WINDUP_GUARD,
                                   current_time=None)

    kit = MotorKit(0x40)
    left_motor = kit.motor1
    velocity_levels = [0, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5,
                       0.55, 0.6, 0.65, 0.7, 0.75, 0.8, 0.85, 0.9, 0.95, 1]
    sleep_time = 2

    for v in velocity_levels:
        left_motor.throttle = v

        start = time.time()
        while time.time() - start < sleep_time:
            left_encoder.update_counter()
    left_motor.throttle = 0

    left_velocities = left_encoder.get_velocity_records()
    left_counters = left_encoder.get_counter_records()
    left_timestamps = left_encoder.get_timestamp_velocity_records()

    records_left = pd.DataFrame({'velocity_steps': left_velocities,
                                 'timestamp': left_timestamps})
    records_left['velocity_ms'] = records_left['velocity_steps'] * cfg.WHEEL_DIAMETER_MM * np.pi / (1000 * cfg.ENCODER_RESOLUTION)
    records_left.set_index('timestamp', drop=True)
    records_left.to_csv('left.csv')

    a_left = pd.DataFrame({'encoder': left_encoder.get_a_records()})
    a_left.to_csv('aleft.csv')
