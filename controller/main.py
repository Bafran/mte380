from camera import Camera
from pid import PIDController
from i2c import I2C
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
  #   pass
    # cameraClass.compute_XandY()
    # pidXClass.computeAngle1()
    # pidYClass.computeAngle1()
    # ikClass.computeAngles()
    # i2cClass.sendangles()

  camera = Camera()
  pidx = PIDController(
    kp = 1,
    ki = 0,
    kd = 0,
    dt = 1 / HERTZ,
    integral_window = 0.5, # seconds
  )
  pidy = PIDController(
    kp = 1,
    ki = 0,
    kd = 0,
    dt = 1 / HERTZ,
    integral_window = 0.5, # seconds
  )

  # One iteration for testing
  x, y = camera.compute_x_and_y()

  # Compute error from target
  pidx.compute()

  # One iteration for testing
  x, y = camera.compute_x_and_y()

  # Compute values, error is simply the difference
  pitch = pidx.compute(x - TARGET_X)
  roll = pidy.compute(y - TARGET_Y)
  