import config as cfg
from RobotDriver import RobotDriver


if __name__ == '__main__':
    rd = RobotDriver(wheel_diameter=cfg.WHEEL_DIAMETER_MM,
                     wheelbase=cfg.WHEELBASE_MM,
                     encoder_resolution=cfg.ENCODER_RESOLUTION,
                     max_angular_velocity=cfg.MAX_ANGULAR_VELOCITY_RPM)

    rd.turn(angle_deg=40,
            linear_velocity_ms=0.05)

