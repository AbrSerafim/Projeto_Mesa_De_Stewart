""" Motor table model """

import numpy as np
from math import pi

class Motor_table_model():
    """ Class to handle the motor to table model params """
    def __init__(self, lengths: list[float] = [103.03e-3, 27.20e-3, 68.75e-3, 18.00e-3, 65.00e-3, 81.00e-3], ang: float = 1.40/180*pi,  theta_0: float = pi/2, theta_1: float = 0.0, test: bool = False) -> None:

        if len(lengths) != 6:
            raise ValueError("Expected 6 linkage lengths")

        self.l, self.l1, self.l2, self.l3, self.l4, self.l5 = lengths

        if self.l2 == 0:
            raise ValueError("L2 cannot be zero")

        self.test_mode = test

        self.ang = ang # Fixed angle between motor and pivot

        self.theta_0 = theta_0 # Output angle
        self.theta_1 = theta_1 # Input angle

        self.sin = np.sin(ang)
        self.cos = np.cos(ang)

        self.calculate_coefficients()

    def update(self, theta_0: float):
        """ Update input and recalculate coefficients """

        self.theta_0 = theta_0
        self.calculate_coefficients()
        self.theta_1 = self.solve_theta_1()


    def calculate_coefficients(self) -> None:
        """ Calculate theta_0-dependent coefficients """

        delta = self.l3 - self.l5

        self.P = (
            2*self.l1*delta*np.cos(self.theta_0)
            + 2*self.l1*self.l4*np.sin(self.theta_0)
            - 2*self.l*self.l1*self.cos
        )

        self.Q = (
            2*self.l1*delta*np.sin(self.theta_0)
            - 2*self.l1*self.l4*np.cos(self.theta_0)
            - 2*self.l*self.l1*self.sin
        )

        self.R = (
            self.l**2 + self.l1**2 - self.l2**2
            + delta**2 + self.l4**2
            - 2*self.l*delta*np.cos(self.theta_0 - self.ang)
            - 2*self.l*self.l4*np.sin(self.theta_0 - self.ang)
        )

    def solve_theta_1(self, tol: float=1e-9) -> float:
        """ Solve inverse kinematics for valid theta values """

        if self.test_mode:
            return self.theta_1

        d = self.Q**2 + self.P**2 - self.R**2

        scale = self.P**2 + self.Q**2 + self.R**2
        tol = 1e-12 * max(scale, 1.0)

        if d < -tol:
            raise ValueError(f"Impossible to solve. d = {d}")

        d = max(d, 0.0)

        sines = [
            (-self.Q*self.R + self.P*np.sqrt(d))/(self.Q**2+self.P**2),
            (-self.Q*self.R - self.P*np.sqrt(d))/(self.Q**2+self.P**2),
                 ]

        roots = [np.asin(np.clip(sine,-1.0,1.0)) for sine in sines if -1 <= sine <= 1]

        theta = min(roots, key=lambda x: abs(x-self.theta_1))

        return theta


if __name__ == "__main__":
    model = Motor_table_model()
    print(model.solve_theta_1())