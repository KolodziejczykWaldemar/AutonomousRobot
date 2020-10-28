import time
from collections import deque

import numpy as np
import pandas as pd
from adafruit_motorkit import MotorKit

import config as cfg
from encoders import Encoder, EncoderCounter
from pid import PIDController


def test_pid(kp, ki, kd, stime, use_pid=True):
    left_encoder = Encoder(motor_pin_a=cfg.LEFT_MOTOR_PIN_A,
                           motor_pin_b=cfg.LEFT_MOTOR_PIN_B,
                           sampling_time_s=stime)

    left_encoder_counter = EncoderCounter(encoder=left_encoder)
    left_encoder_counter.start()

    pid_controller = PIDController(proportional_coef=kp,
                                   integral_coef=ki,
                                   derivative_coef=kd,
                                   windup_guard=cfg.PID_WINDUP_GUARD,
                                   current_time=None)

    max_steps_velocity_sts = cfg.ENCODER_RESOLUTION * cfg.MAX_ANGULAR_VELOCITY_RPM / 60

    kit = MotorKit(0x40)
    left_motor = kit.motor1

    velocity_levels = [1000, 2000, 3000, 4000, 5000, 6000, 0]
    velocity_levels = [3000, 0]
    sleep_time = 5

    velocities_level_records = deque([])
    velocities_steps_records = deque([])
    pid_velocities_steps_records = deque([])
    timestamps_records = deque([])

    for v in velocity_levels:
        pid_controller.reset()
        pid_controller.set_set_point(v)
        left_motor.throttle = max(min(1, v / max_steps_velocity_sts), 0)

        start = time.time()
        current_time = time.time()
        while current_time - start < sleep_time:

            timestamp, measured_steps_velocity_sts = left_encoder_counter.get_velocity()

            # PID control
            if use_pid:
                new_steps_velocity_sts = pid_controller.update(-measured_steps_velocity_sts, current_time)
                left_motor.throttle = max(min(1, new_steps_velocity_sts / max_steps_velocity_sts), 0)
            else:
                new_steps_velocity_sts = -1

            velocities_level_records.append(v)
            velocities_steps_records.append(-measured_steps_velocity_sts)
            pid_velocities_steps_records.append(new_steps_velocity_sts)
            timestamps_records.append(timestamp)

            current_time = time.time()

    left_motor.throttle = 0
    left_encoder_counter.finish()

    records_left = pd.DataFrame({'velocity_steps': velocities_steps_records,
                                 'velocity_levels': velocities_level_records,
                                 'pid_velocity_steps': pid_velocities_steps_records,
                                 'timestamp': timestamps_records})
    records_left['velocity_ms'] = records_left['velocity_steps'] * cfg.WHEEL_DIAMETER_MM * np.pi / (1000 * cfg.ENCODER_RESOLUTION)
    records_left.set_index('timestamp', drop=True)
    return records_left


if __name__ == '__main__':
    # for kp in [0, 0.2, 0.4, 0.6, 0.8, 1.0]:
    #     for ki in [0, 0.2, 0.4, 0.6, 0.8, 1.0]:
    #         for kd in [0, 0.2, 0.4, 0.6, 0.8, 1.0]:
    #
    #             records = test_pid(kp, ki, kd)
    #             records.to_csv('kp_{}_ki_{}_kd_{}.csv'.format(kp, ki, kd))

    for t in [0.001, 0.01, 0.1]:
        records = test_pid(0, 0, 0, t, False)
        records.to_csv('kp_{}_ki_{}_kd_{}_t_{}.csv'.format(0, 0, 0, t))
