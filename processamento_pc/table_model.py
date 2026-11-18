""" Motor table model """

import sympy as sp
import numpy as np

from math import atan, ceil, floor, pi

class Table_Model():
    """ Class to handle the table model params """
    def __init__(self, lengths: list[float], ang: float,  theta_0: float = 0.0, theta_1: float = 0.0) -> None:

        if len(lengths) != 6:
            raise ValueError("Expected 6 linkage lengths")

        self.l, self.l1, self.l2, self.l3, self.l4, self.l5 = lengths

        if self.l2 == 0:
            raise ValueError("L2 cannot be zero")

        self.ang = ang # Fixed angle between motor and pivot

        self.theta_0 = theta_0 # Output angle
        self.theta_1 = theta_1 # Input angle

        self.calculate_constants()
        self.calculate_coefficients_1()

    def update(self, theta_1: float):
        """ Update input and recalculate coefficients """

        self.theta_1 = theta_1
        self.theta_0 = self.theta_0
        self.calculate_coefficients_0()

    def calculate_constants(self) -> None:
        """ Coefficients to solve for input angle """

        inv_l2 = 1/self.l2**2

        self.A = (self.l**2 + self.l1**2 + self.l4**2) * inv_l2
        self.B = (self.l5 - self.l3)**2 * inv_l2
        self.C = (self.l5 + self.l3)**2 * inv_l2
        self.D = 2 * self.l * self.l1 * inv_l2
        self.E = 2 * self.l * (self.l5 - self.l3) * inv_l2
        self.F = 2 * self.l * (self.l5 + self.l3) * inv_l2
        self.G = 2 * self.l * self.l4 * inv_l2
        self.H = 2 * self.l1 * (self.l5 - self.l3) * inv_l2
        self.I = 2 * self.l1 * (self.l5 + self.l3) * inv_l2
        self.J = 2 * self.l1 * self.l4 * inv_l2
        self.K = 2 * self.l4 * (self.l5 - self.l3) * inv_l2
        self.L = 2 * self.l4 * (self.l5 + self.l3) * inv_l2

        self.cos = np.cos(self.ang)
        self.sin = np.sin(self.ang)

    def calculate_coefficients_0(self) -> None:
        """ Calculate theta_1-dependent quartic coefficients """
        self.b_0 = self.B
        self.c_0 = self.C
        self.f_0 = (self.E - self.G) * self.cos - self.H * np.cos(self.theta_1) + self.J * np.sin(self.theta_1)
        self.g_0 = -self.F * self.sin - self.G * self.cos + self.I * np.sin(self.theta_1) + self.J * np.cos(self.theta_1)
        self.h_0 = self.L - self.K
        self.d_0 = self.A - 1.0 - self.D * np.cos(self.ang - self.theta_1)

    def calculate_coefficients_1(self) -> None:
        """ Calculate theta_1-dependent quartic coefficients """

        self.a_1 = -self.D * self.cos - self.H + self.J * np.sin(self.theta_0)
        self.b_1 = -self.D * self.sin + self.I * np.sin(self.sin) + self.J * np.sin(self.sin)
        self.c_1 = self.A - 1.0 + self.B * (np.cos(self.theta_0))**2 + self.C * (np.sin(self.theta_0))**2 + self.E * self.cos * np.cos(self.theta_0)
        self.c_1 += -self.F * self.sin * np.sin(self.theta_0) - self.G * (np.sin(self.ang + self.theta_0)) + (self.L - self.K) * np.sin(2*self.theta_0) / 2

    def solve_theta_1(self, theta_range=(-pi, pi), tol: float=1e-9) -> list[float]:
        """ Solve inverse kinematics for valid theta values """

        theta_min, theta_max = theta_range

        d = self.a_1**2 + self.b_1**2 - self.c_1**2

        if d < 0:
            raise ValueError("Impossible to solve, imaginary roots")

        sines = [
            (-self.b_1*self.c_1 + self.a_1*(d)**.5)/(self.a_1**2+self.b_1**2),
            (-self.b_1*self.c_1 - self.a_1*(d)**.5)/(self.a_1**2+self.b_1**2),
                 ]

        roots = [np.asin(sine) for sine in sines]

        return roots

    def solve_theta_0(self, theta_range=(-pi, pi), tol: float=1e-9) -> list[float]:
        """ Solve direct kinematics for valid theta values """

        theta_min, theta_max = theta_range

        coeffs = np.array([
            self.b_0 - self.f_0 + self.d_0,
            2*(self.g_0 - self.h_0),
            -2*self.b_0 + 4*self.c_0 + 2*self.d_0,
            2*(self.g_0 + self.h_0),
            self.b_0 + self.f_0 + self.d_0
        ], dtype=float)

        scale = np.max(np.abs(coeffs))

        if not np.isfinite(scale):
            raise ValueError("Non-finite polynomial coefficients")

        if scale == 0:
            raise ValueError("Equation is satisfied for every theta.")

        # Remove numerically negligible leading coefficients
        coeffs[np.abs(coeffs) < tol * scale] = 0

        # Calculate polynomial roots
        roots = np.roots(coeffs)

        theta_solutions = []

        for root in roots:

            if abs(root.imag) > tol * (1 + abs(root.real)):
                continue

            theta = 2 * atan(root.real)

            # Account for periodicity
            k_min = ceil((theta_min - theta) / (2*pi))
            k_max = floor((theta_max - theta) / (2*pi))

            for k in range(k_min, k_max + 1):
                theta_solutions.append(theta + 2*pi*k)

        # Remove duplicate solutions
        theta_solutions = sorted(theta_solutions)

        unique = []

        for theta in theta_solutions:
            if not unique or abs(theta - unique[-1]) > tol:
                unique.append(theta)

        return unique



if __name__ == "__main__":
    u, b, c, f, g, h, d = sp.symbols('u b c f g h d')

    expr = b*((1-u**2)/(1+u**2))**2 + c*((2*u)/(1+u**2))**2
    expr += f*((1-u**2)/(1+u**2)) + g*((2*u)/(1+u**2)) + h*((1-u**2)/(1+u**2))*((2*u)/(1+u**2)) + d

    expr = sp.expand(expr)
    expr = sp.simplify(expr)

    print(expr)

    #expr = sp.simplify(expr) * (u**4 + 2*u**2 + 1)