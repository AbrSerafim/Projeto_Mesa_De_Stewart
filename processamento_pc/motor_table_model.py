""" Motor table model """

import sympy as sp
import numpy as np

from math import pi

class Motor_table_model():
    """ Class to handle the motor to table model params """
    def __init__(self, lengths: list[float], ang: float,  theta_0: float = pi, theta_1: float = 0.0) -> None:

        if len(lengths) != 6:
            raise ValueError("Expected 6 linkage lengths")

        self.l, self.l1, self.l2, self.l3, self.l4, self.l5 = lengths

        if self.l2 == 0:
            raise ValueError("L2 cannot be zero")

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

        self.P = (
            -2*self.l*self.l1*self.cos + 2*self.l1*self.l3*np.cos(self.theta_0) + 
            +2*self.l1*self.l4*np.sin(self.theta_0) - 2*self.l1*self.l5*np.cos(self.theta_0)
            )
        
        self.Q = (
            -2*self.l*self.l1*self.sin + 2*self.l1*self.l3*np.sin(self.theta_0) + 
            -2*self.l1*self.l4*np.cos(self.theta_0) - 2*self.l1*self.l5*np.sin(self.theta_0)
            )

        self.R = (
            +self.l**2 + self.l1**2 - self.l2**2 + self.l3**2 + self.l4**2 - 2*self.l3*self.l5 +
            -2*self.l*self.l3*self.cos*np.cos(self.theta_0)-2*self.l*self.l4*self.cos*np.sin(self.theta_0) +
            +2*self.l*self.l5*self.cos*np.cos(self.theta_0) - 2*self.l*self.l3*self.sin*np.sin(self.theta_0) +
            +2*self.l*self.l4*self.sin*np.cos(self.theta_0) + 2*self.l*self.l5*self.sin*np.sin(self.theta_0)
        )

    def solve_theta_1(self, tol: float=1e-9) -> list[float]:
        """ Solve inverse kinematics for valid theta values """

        d = self.Q**2 + self.P**2 - self.R**2

        if d < -tol:
            raise ValueError("Impossible to solve, imaginary roots")

        sines = [
            (-self.Q*self.R + self.P*np.sqrt(d))/(self.Q**2+self.P**2),
            (-self.Q*self.R - self.P*np.sqrt(d))/(self.Q**2+self.P**2),
                 ]

        roots = [np.asin(np.clip(sine,-1.0,1.0)) for sine in sines if -1 <= sine <= 1]

        theta = min(roots, key=lambda x: abs(x-self.theta_1))

        return theta


if __name__ == "__main__":
    u, b, c, f, g, h, d = sp.symbols('u b c f g h d')

    expr = b*((1-u**2)/(1+u**2))**2 + c*((2*u)/(1+u**2))**2
    expr += f*((1-u**2)/(1+u**2)) + g*((2*u)/(1+u**2)) + h*((1-u**2)/(1+u**2))*((2*u)/(1+u**2)) + d

    expr = sp.expand(expr)
    expr = sp.simplify(expr)

    print(expr)

    #expr = sp.simplify(expr) * (u**4 + 2*u**2 + 1)