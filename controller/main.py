# from camera import Camera
# from pid import PIDController
from i2c import I2C
import numpy as np
from ik import IK

HERTZ = 60

TARGET_X = 0
TARGET_Y = 0

if __name__ == "__main__":
  # Initializations

  # Camera
  # IK
  # PID(s)
  # I2C interface

  # while True:
  stewart_platform_ik = IK(105.83, 99.92, 47.5, 190, np.radians(15), 0.146, 0)
  servo_angles = stewart_platform_ik.compute(np.array[[0, 0, 0]])
  i2c_interface = I2C()
  i2c_interface.send_data(servo_angles)