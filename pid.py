import time

from typing import Optional


class PIDController:
    def __init__(self,
                 proportional_coef: float,
                 integral_coef: float,
                 derivative_coef: float,
                 windup_guard: float,
                 current_time: Optional[float] = None) -> None:
        """PID Controller class for universal control purposes.

        u(t) = K_p e(t) + K_i \int_{0}^{t} e(t)dt + K_d {de}/{dt}

        Args:
            proportional_coef (float): Proportional coefficient.
            integral_coef (float): Integral coefficient.
            derivative_coef (float): Derivative coefficient.
            windup_guard (float): Critical value of integral term in main equation.
            current_time (Optional[float]): First timestamp of control in seconds.
        """
        self.__p_coef = proportional_coef
        self.__i_coef = integral_coef
        self.__d_coef = derivative_coef

        self.__windup_guard = windup_guard
        self.__current_time = current_time if current_time is not None else time.time()

        self.__set_point = None

        self.__last_time = self.__current_time
        self.__last_error = 0.0

        self.__p_term = 0.0
        self.__i_term = 0.0
        self.__d_term = 0.0

    def update(self,
               feedback_value: float,
               current_time: Optional[float]) -> float:
        """Method for updating actuator value.

        Args:
            feedback_value (float): Real target value, measured in previous step.
            current_time (Optional[float]): Current timestamp in seconds.

        Returns:
            float: Actuator value.
        """

        error = self.__set_point - feedback_value
        delta_error = error - self.__last_error

        self.__current_time = current_time if current_time is not None else time.time()
        delta_time = self.__current_time - self.__last_time

        self.__p_term += self.__p_coef * error
        self.__d_term += self.__d_coef * delta_error / delta_time
        self.__i_term += self.__i_term * error * delta_time
        self.__i_term = max(min(self.__windup_guard, self.__i_term), -self.__windup_guard)

        self.__last_time = self.__current_time
        self.__last_error = error

        return self.__p_term + self.__i_term + self.__d_term

    def reset(self) -> None:
        """Method resets all equation terms.

        """
        self.__p_term = 0.0
        self.__i_term = 0.0
        self.__d_term = 0.0
        self.__last_error = 0.0

    def set_set_point(self, set_point: float) -> None:
        """Setter method for new set point.

        Args:
            set_point(float): Point of interest, eg. target velocity.

        Returns:

        """
        self.__set_point = set_point
