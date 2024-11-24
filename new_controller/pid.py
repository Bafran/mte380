import collections
import math
import numpy as np


class PIDController:
  def __init__(self, kp, ki, kd, dt, dead_zone=5, edge_threshold=70, integral_window=2):
    self.kp = kp
    self.ki = ki
    self.kd = kd
    self.dt = dt
    self.integral_window = int(integral_window / dt)
    self.error_window = collections.deque(maxlen=self.integral_window)
    self.prev_error = 0
    self.prev_derivative = 0
    self.dead_zone = dead_zone
    self.edge_threshold = edge_threshold

  def compute(self, error):
    gain = 1
    # Ramp up P response near the edges
    if abs(error) > self.edge_threshold:
      gain = 1.2
    # P response in the dead zone
    if abs(error) < self.dead_zone:
      error = 0

    P = self.kp * error * gain
    
    self.error_window.append(error)
    I = self.ki * (self.dt * sum(self.error_window))
    
    derivative = (error - self.prev_error) / self.dt
    D = self.kd * (0.5 * derivative + 0.5 * self.prev_derivative) * gain
    
    self.prev_error = error
    self.prev_derivative = derivative
    
    print(f"P:{P} I:{I} D:{D}")
    return P + I + D
  