""" Ball table model """

import numpy as np

class Table_ball_model():
    """ Class to handle the table to motor model params """
    def __init__(self, J: float, m: float,
                 g: float, R:float, Ta:float,
                 ang: float = 0) -> None:
        self.ang = ang
        self.J = J
        self.R2 = R**2
        self.P = m*g
        self.tg = np.tan(ang)
        self.sec = 1/np.cos(ang)
        self.Ta = Ta
        self.A = (ang*J-m*g*R**2*np.tan(ang))/J*Ta
        self.B = J/np.cos(ang)/2*Ta**2
        self.angk = 0
        self.angk_1 = 0
        self.angk_2 = 0
        self.posk = ang
        self.posk_1 = 0
        self.posk_2 = 0

    # Ang(s) = Ang_0/s - PR2*tg/J/s+J*sec/s2 * Pos(s)
    # ang_k-2ang_(k-1)+ang_(k-2) = A(δ_k-δ_(k-1)) + B(pos_(k-1)+pos_(k-2))

    def update(self, pos, k):
        self.posk_2 = self.posk_1
        self.posk_1 = self.posk
        self.posk = pos
        self.angk_2 = self.angk_1
        self.angk_1 = self.ang
        if k in [0, 1]:
            self.angk = self.A*(-1)**k + 2*self.angk_1 - self.angk_2 + self.B*(self.posk_1+self.posk_2)
        else:
            self.angk = 2*self.angk_1 - self.angk_2 + self.B*(self.posk_1+self.posk_2)


if __name__ == "__main__":
    pass