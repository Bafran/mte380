class Camera:
  pass
  # Init camera

  # Compute function - acts like a black box

class PID:
  pass
  # Init gains, time step, and window

  # Compute function - acts like a black box

class IK:
  pass
  # Init physical dimenstions (beta, wtv else i dont remember)

  # Compute function - acts like a black box

class I2C:
  pass
  # Init device addresses

  # Send function - acts like a black box
  # may need to add a delay after this if the while loop is too fast

if __name__ == "__main__":
  # Initializations

  # Camera
  # IK
  # PID(s)
  # I2C interface

  while True:
    pass
    # cameraClass.compute_XandY()
    # pidXClass.computeAngle1()
    # pidYClass.computeAngle1()
    # ikClass.computeAngles()
    # i2cClass.sendangles()