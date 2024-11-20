import collections


class PIDController:
  def __init__(self, kp, ki, kd, dt, integral_window=0.5):
    self.kp = kp
    self.ki = ki
    self.kd = kd
    self.dt = dt
    self.integral_window = int(integral_window / dt)
    self.error_window = collections.deque(maxlen=self.integral_window)
    self.prev_error = 0
    self.prev_derivative = 0

  def compute(self, error):
    P = self.kp * error
    
    self.error_window.append(error)
    I = self.ki * (self.dt * sum(self.error_window))
    
    derivative = (error - self.prev_error) / self.dt
    D = self.kd * (0.8 * derivative + 0.2 * self.prev_derivative)
    
    self.prev_error = error
    self.prev_derivative = derivative
    
    return P + I + D
  