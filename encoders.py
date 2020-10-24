import time

from collections import deque

import RPi.GPIO as GPIO


class Encoder:
    def __init__(self,
                 motor_pin_a: int,
                 motor_pin_b: int,
                 velocity_averaging_length: int = 5) -> None:

        self.__motor_pin_a = motor_pin_a
        self.__motor_pin_b = motor_pin_b
        self.__velocity_averaging_length = velocity_averaging_length

        self.__counter = 0
        self.__timestamp_records = deque([time.time()], maxlen=self.__velocity_averaging_length)
        self.__counter_records = deque([self.__counter], maxlen=self.__velocity_averaging_length)
        # self.__timestamp_records = deque([time.time()])
        # self.__counter_records = deque([self.__counter])

        self.__timestamp_velocity_records = deque([])
        self.__velocity_records = deque([])
        self.__last_a_input = None

        self.__setup()

        self.a_input_records = deque([])

    def __setup(self) -> None:
        GPIO.setup(self.__motor_pin_a, GPIO.IN)
        GPIO.setup(self.__motor_pin_b, GPIO.IN)

    def update_counter(self) -> bool:  # TODO added returning
        is_updated = False
        a_input = GPIO.input(self.__motor_pin_a)

        self.a_input_records.append(a_input)

        if a_input != self.__last_a_input:
            if a_input != GPIO.input(self.__motor_pin_b):
                self.__counter += 1
            else:
                self.__counter -= 1
            current_time = time.time()
            self.__timestamp_records.append(current_time)
            self.__counter_records.append(self.__counter)
            self.__velocity_records.append(self.calculate_velocity())
            self.__timestamp_velocity_records.append(current_time)
            is_updated = True

        self.__last_a_input = a_input
        return is_updated

    def reset_counter(self) -> None:
        self.__counter = 0
        self.__timestamp_records = deque([time.time()], maxlen=self.__velocity_averaging_length)
        self.__counter_records = deque([self.__counter], maxlen=self.__velocity_averaging_length)
        # self.__timestamp_records = deque([time.time()])
        # self.__counter_records = deque([self.__counter])
        self.__timestamp_velocity_records = deque([])
        self.__velocity_records = deque([])
        self.__last_a_input = None

    def calculate_velocity(self):
        return (self.__counter_records[-1] - self.__counter_records[0]) / \
               (self.__timestamp_records[-1] - self.__timestamp_records[0])

    def get_counter(self):
        return self.__counter

    def get_timestamp_records(self):
        return self.__timestamp_records

    def get_timestamp_velocity_records(self):
        return self.__timestamp_velocity_records

    def get_counter_records(self):
        return self.__counter_records

    def get_velocity_records(self):
        return self.__velocity_records

    def get_a_records(self):
        return self.a_input_records


class EncoderController:
    def __init__(self,
                 left_motor_pin_a: int,
                 left_motor_pin_b: int,
                 right_motor_pin_a: int,
                 right_motor_pin_b: int,
                 encoder_resolution: int) -> None:

        self.__setup()

        self.__left_encoder = Encoder(motor_pin_a=left_motor_pin_a,
                                      motor_pin_b=left_motor_pin_b)

        self.__right_encoder = Encoder(motor_pin_a=right_motor_pin_a,
                                       motor_pin_b=right_motor_pin_b)
        # TODO pay attention to reversed mounting of motors

        self.__encoder_resolution = encoder_resolution

    def __setup(self) -> None:
        GPIO.setmode(GPIO.BCM)

    def update_counters(self) -> None:
        self.__left_encoder.update_counter()
        self.__right_encoder.update_counter()

    def reset_counters(self):
        self.__left_encoder.reset_counter()
        self.__right_encoder.reset_counter()

    def get_right_encoder(self) -> Encoder:
        return self.__right_encoder

    def get_left_encoder(self) -> Encoder:
        return self.__left_encoder

    def get_encoder_resolution(self) -> int:
        return self.__encoder_resolution


if __name__ == '__main__':
    lel = deque([2, 3, 4], maxlen=4)
    for i in range(10):
        lel.append(i)
        print(lel)