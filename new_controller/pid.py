import collections
import math


class PIDController:
  def __init__(self, kp, ki, kd, dt, dead_zone=5, integral_window=2):
    self.kp = kp
    self.ki = ki
    self.kd = kd
    self.dt = dt
    self.integral_window = int(integral_window / dt)
    self.error_window = collections.deque(maxlen=self.integral_window)
    self.prev_error = 0
    self.prev_derivative = 0
    self.dead_zone = dead_zone

  def compute(self, error):

    # error = math.pow(error, 1.1)

    if abs(error) < self.dead_zone:
      error = 0

    print('Error', error)
    P = self.kp * error
    
    self.error_window.append(error)
    I = self.ki * (self.dt * sum(self.error_window))
    
    derivative = (error - self.prev_error) / self.dt
    D = self.kd * (0.8 * derivative + 0.2 * self.prev_derivative)
    
    self.prev_error = error
    self.prev_derivative = derivative
    
    print(f"P:{P} I:{I} D:{D}")
    return P + I + D
  