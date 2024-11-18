# from camera import Camera
# from pid import PIDController
from i2c import I2C
# from ik import IK

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
  i2c_interface = I2C()
  i2c_interface.send_data([5, 5, 5, 5, 5, 5])