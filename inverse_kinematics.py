from src.stewart_controller import Stewart_Platform
import numpy as np
import matplotlib.pyplot as plt

class IK():
    def __init__(self, r_B, r_P, lhl, ldl, gamma_B, gamma_P, ref_rotation=0):
        self.stewart_platform = Stewart_Platform(r_B, r_P, lhl, ldl, gamma_B, gamma_P, ref_rotation)
    
    def compute(self, tilt, translation=np.array([0,0,0])):
        servo_angles = self.stewart_platform.calculate(tilt, translation)
        return servo_angles
    
    def plot(self):
        fig, ax = plt.subplots()
        ax = self.stewart_platform.plot_platform()
        plt.draw()
        plt.show()