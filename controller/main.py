from camera import Camera
from pid import PIDController
from i2c import I2C
import numpy as np
from ik import IK

HERTZ = 60

TARGET_X = 0
TARGET_Y = 0

MAX_MM = 150

MAX_TILT_ANGLE = 25

if __name__ == "__main__":
  # Initializations

  # Camera
  # IK
  # PID(s)
  # I2C interface

  # while True:

  camera = Camera()
  coordinates = camera.compute_x_and_y()

  pid_controller_x = PIDController(1, 0, 0, 1/60)
  pid_controller_y = PIDController(1, 0, 0, 1/60)

  if (coordinates[0] and coordinates[1]):
    x_error_mm = pid_controller_x.compute(coordinates[0])
    y_error_mm = pid_controller_y.compute(coordinates[1])

    # Normalizing error
    normalized_x_error = x_error_mm / MAX_MM
    normalized_y_error = y_error_mm / MAX_MM

    # Scaling angle
    x_tilt = -(normalized_x_error * MAX_TILT_ANGLE)
    y_tilt = -(normalized_y_error * MAX_TILT_ANGLE)

    stewart_platform_ik = IK(105.83, 99.92, 47.5, 190, np.radians(15), 0.146, 0)
    servo_angles = stewart_platform_ik.compute(np.array([x_tilt, y_tilt, 0]))
    i2c_interface = I2C()
    i2c_interface.send_data([int(x) for x in servo_angles.tolist()])
  else:
    print("Camera can't see anything")

