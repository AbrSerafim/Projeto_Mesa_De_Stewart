""" Comunicates with the pico """

import serial
import time

class Com():
    """ Class to handle the communication """
    def __init__(self, port="COM5", baudrate=115200, timeout=1) -> None:
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=timeout
            )

        time.sleep(2) # Wait for Pico to initialize

    def send(self, ang_1: float, ang_3: float):
        """ Sends the info through the serial """

        self.ser.write(f"[{ang_1:.6f},{ang_3:.6f}]".encode("ascii"))

    def close(self):
        """ Finishes comunication """
        self.ser.close()