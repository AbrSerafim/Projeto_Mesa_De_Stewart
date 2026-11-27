""" Code to handle all pc/python processes """

from pathlib import Path

import ping_pong_yolo_mod as ppy
from table_ball_model import Table_ball_model
from motor_table_model import Motor_table_model
from com import Com

# ==========================
# Config
# ==========================

MODEL_PATH = Path("yolo26n.pt")

CAMERA_ID = 0

PORT = "COM5"
BAUDRATE = 115200
TIMEOUT = 1

# ==========================
# Functions & classes
# ==========================

def px_to_metric(xc: int, yc: int, R):
    return xc, yc

# ==========================
# Process
# ==========================

# Creates the model and fetches the camera which will be used
ppy.setup(model_path=MODEL_PATH, camera_id=CAMERA_ID)

x_to_theta = Table_ball_model(J=1.390e-3, R=12.46e-2, m=63.95e-3, g=9.81, ang=0, Ta=20e-3)
y_to_psi   = Table_ball_model(J=1.390e-3, R=12.46e-2, m=63.95e-3, g=9.81, ang=0, Ta=20e-3)

theta_to_ang3 = Motor_table_model() 
psi_to_ang1   = Motor_table_model()

#com = Com(port=PORT, baudrate=BAUDRATE, timeout=TIMEOUT)

k = 0
while True:

    # -------------------------------
    # Position aquisition
    # -------------------------------

    # 1. Capture image
    frame = ppy.capture_img()

    # 2. Inference
    results = ppy.inference(frame)

    # 3. Bbox process
    bbox = ppy.extract_result(results)

    # 4. Process results
    _, _, _, _, xc, yc = ppy.process_result(bbox)
    xc, yc = px_to_metric(xc=xc,yc=yc,R=None)

    # -------------------------------
    # Position to table angle
    # -------------------------------

    x_to_theta.update(pos=xc, k=k)
    y_to_psi.update(pos=yc,k=k)

    # -------------------------------
    # Table angle to motor angle
    # -------------------------------

    theta_to_ang3.update(x_to_theta.angk)
    psi_to_ang1.update(y_to_psi.angk)

    # -------------------------------
    # Send motor angles to pico
    # -------------------------------
    #com.send(ang_1=psi_to_ang1.theta_1, ang_3=theta_to_ang3.theta_1)

    k+=1