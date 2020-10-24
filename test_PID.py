import time
from collections import deque

import numpy as np
import pandas as pd
from adafruit_motorkit import MotorKit

import config as cfg
from encoders import Encoder
from pid import PIDController


if __name__ == '__main__':
    left_encoder = Encoder(motor_pin_a=cfg.LEFT_MOTOR_PIN_A,
                           motor_pin_b=cfg.LEFT_MOTOR_PIN_B,
                           velocity_averaging_length=30)

    pid_controller = PIDController(proportional_coef=cfg.PID_KP,
                                   integral_coef=cfg.PID_KI,
                                   derivative_coef=cfg.PID_KD,
                                   windup_guard=cfg.PID_WINDUP_GUARD,
                                   current_time=None)

    max_steps_velocity_sts = cfg.ENCODER_RESOLUTION * cfg.MAX_ANGULAR_VELOCITY_RPM / 60

    kit = MotorKit(0x40)
    left_motor = kit.motor1
    # velocity_levels = [0, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5,
    #                    0.55, 0.6, 0.65, 0.7, 0.75, 0.8, 0.85, 0.9, 0.95, 1, 0]
    velocity_levels = [1000, 2000, 3000, 4000, 5000, 6000, 0]
    sleep_time = 5
    velocities_level_records = deque([])
    for v in velocity_levels:
        pid_controller.set_set_point(v)
        left_motor.throttle = max(min(1, v / max_steps_velocity_sts), 0)

        start = time.time()
        current_time = time.time()
        while current_time - start < sleep_time:

            is_updated = left_encoder.update_counter()

            # PID control
            # measured_steps_velocity_sts = left_encoder.calculate_velocity()
            # new_steps_velocity_sts = pid_controller.update(measured_steps_velocity_sts, current_time)
            # left_motor.throttle = max(min(1, new_steps_velocity_sts / max_steps_velocity_sts), 0)

            if is_updated:
                velocities_level_records.append(v)
            current_time = time.time()

    left_velocities = left_encoder.get_velocity_records()
    left_counters = left_encoder.get_counter_records()
    left_timestamps = left_encoder.get_timestamp_velocity_records()

    print(len(left_velocities))
    print(len(velocities_level_records))

    records_left = pd.DataFrame({'velocity_steps': left_velocities,
                                 'velocity_levels': velocities_level_records,
                                 'timestamp': left_timestamps})
    records_left['velocity_ms'] = records_left['velocity_steps'] * cfg.WHEEL_DIAMETER_MM * np.pi / (1000 * cfg.ENCODER_RESOLUTION)
    records_left.set_index('timestamp', drop=True)
    records_left.to_csv('left.csv')

    a_left = pd.DataFrame({'encoder': left_encoder.get_a_records()})
    a_left.to_csv('aleft.csv')
