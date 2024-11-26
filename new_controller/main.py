# Subsystem controllers
from camera import Camera
from pid import PIDController
from ik import IK
from motor_serial import MotorSerial

import numpy as np
from time import sleep
import math

HERTZ = 60

TARGET_X = 0
TARGET_Y = 0

MAX_MM = 150

MAX_TILT_ANGLE = 20

if __name__ == "__main__":
  # Initializations
  camera = Camera(camera_index=0)
  pid_controller_x = PIDController(0.105, 0.001, 0.054, 0.05, dead_zone=0, edge_threshold=60)
  pid_controller_y = PIDController(0.105, 0.001, 0.054, 0.05, dead_zone=0, edge_threshold=60)
  stewart_platform_ik = IK(105.83, 99.92, 47.5, 190, np.radians(15), 0.146, 0)
  bus = MotorSerial('/dev/tty.usbmodem21301')

  # Trajectory
  followTrajectory = False
  trajectory = [(0, 0), (40, 40), (0, 0), (-40, -40)]
  trajectoryIndex = 0
  inBoundsCycles = 0
  DISTANCE_THRESH = 25 # [mm], this is the abs value of the allowable distance from the target point
  CYCLES_THRESH = 2 # 2 seconds @ 50ms cycle time

  running = True
  while running:
    coordinates = camera.compute_x_and_y()

    if (coordinates[0] and coordinates[1]):
      # To follow a trajectory, we must change the point the PID controller targets
      # This point will be at trajectory[trajectoryIndex] and can be cleared and move on
      # to the next index when the target has been held within a threshold for some number of cycles
      if followTrajectory:
        # Our target is trajectory[trajectoryIndex]
        xtarget, ytarget = trajectory[trajectoryIndex]

        # Check if we're in bounds
        inBounds = True
        if abs(xtarget - coordinates[0]) > DISTANCE_THRESH:
          inBounds = False
        if abs(ytarget - coordinates[1]) > DISTANCE_THRESH:
          inBounds = False

        # If we're within bounds of a target, count up cycles. Otherwise, reset.
        if inBounds:
          inBoundsCycles += 1
        else:
          inBoundsCycles = 0

        # If we hit a certain number of cycles, move on to the next index
        if inBoundsCycles > CYCLES_THRESH:
          inBoundsCycles = 0
          trajectoryIndex  = (trajectoryIndex + 1) % len(trajectory)
      else:
        xtarget, ytarget = (0, 0)

      x_error_mm = pid_controller_x.compute(coordinates[0], target=xtarget)
      y_error_mm = pid_controller_y.compute(coordinates[1], target=ytarget)

      # Normalizing error
      normalized_x_error = x_error_mm / MAX_MM
      normalized_y_error = y_error_mm / MAX_MM

      # Scaling angle
      roll = max(-15, min((normalized_x_error * MAX_TILT_ANGLE), 20))
      pitch = max(-15, min(-(normalized_y_error * MAX_TILT_ANGLE),20))

      servo_angles = stewart_platform_ik.compute(np.array([pitch, roll, 0]))

      bus_angles = []
      for angle in servo_angles.tolist():
        # Clamp motor angles
        if angle > 70:
          angle = 70
        elif angle < -15:
          angle = -15

        bus_angles.append(int(angle))

      try:
        print(f"Ball Coords: {coordinates}")
        print(f"Pitch: {pitch}")
        print(f"Roll: {roll}")
        print(f"Target: {trajectory[trajectoryIndex]}")
        print(f"InBounds: {inBounds}, Cycles: {inBoundsCycles}")
      except:
        pass

      print(bus_angles)
      bus.send(bus_angles)

      sleep(0.03)
    else:
      print("Camera can't see anything")

    running = True
