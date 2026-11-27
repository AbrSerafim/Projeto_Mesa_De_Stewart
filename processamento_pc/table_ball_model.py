""" Ball table model """

import numpy as np

class Table_ball_model():
    """ Class to handle the table to motor model params """
    def __init__(self, J: float, m: float,
                 g: float, R:float, Ta:float,
                 ang: float = 0, pos: float = 0) -> None:
        self.A = J/Ta**2/m/g/R**2/np.cos(ang)
        self.B = Ta*(np.tan(ang)+ang)
        self.angk = ang
        self.angk_1 = 0
        self.posk = pos
        self.posk_1 = 0
        self.posk_2 = 0
        self.posk_3 = 0


    def update(self, pos, k):
        self.posk_3 = self.posk_2
        self.posk_2 = self.posk_1
        self.posk_1 = self.posk
        self.posk = pos
        self.angk_1 = self.angk
        self.k = self.angk_1 + self.A*(self.posk - 3*self.posk_1 + 3*self.posk_2 - self.posk_3)
        if k == 1:
            self.angk += self.B 


if __name__ == "__main__":
    pass