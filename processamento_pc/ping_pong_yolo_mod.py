""" Library to run yolo detection and extract CG of a object """

from ultralytics import YOLO
import cv2
import numpy as np
from pathlib import Path

# =========================
# Config
# =========================

MODEL_PATH = Path(r"yolo26n.pt")
CONF = .5

CAMERA_ID = 1

# =========================
# Functions & Classes
# =========================

def setup(model_path: Path = MODEL_PATH) -> None:
    if not model_path.exists():
        raise FileNotFoundError("Model path does not exists")

    # Load yolo
    global model
    model = YOLO(str(model_path))

    # Tries to open the cam
    global cap
    cap = cv2.VideoCapture(CAMERA_ID)
    if not cap.isOpened():
        cap.release()
        raise RuntimeError("Failed to open a webcam")

    global last_points
    last_points = [(0, 0), (0, 0)]

def capture_img() -> cv2.typing.MatLike:
    # Capture frame
    ret, frame = cap.read()
    if not ret:
        raise RuntimeError("Not able to capture camera frame")
    return frame

def inference(frame: cv2.typing.MatLike,conf: float = CONF): return model(frame,conf=conf,verbose=False)

def extract_result(results):
    # Check if anything was found
    boxes = results[0].boxes
    if len(boxes) > 0:
        # Transfer all data to CPU
        boxes = boxes.cpu()

        # Finds most confident bbox
        confs = boxes.conf.numpy
        best_idx = np.argmax(confs)
        return boxes.xyxy[best_idx].numpy()

def process_result(bbox):
    if bbox is None:
        return 
    # Extract bbox corners
    x1, y1, x2, y2 = bbox

    # Center
    xc = (x1 + x2) / 2
    yc = (y1 + y2) / 2

    x1, y1, x2, y2, xc, yc = map(int, [x1, y1, x2, y2, xc, yc])

    # Store CG position
    last_points.append((xc, yc))
    last_points.pop(0)

    return x1, y1, x2, y2, xc, yc

def draw(box, frame):
    if box is None:
        cv2.imshow(
        "Deteccao da bolinha",
        frame
    )
    else:
        x1, y1, x2, y2, xc, yc = box

        cv2.rectangle(
            frame,
            (x1, y1),
            (x2, y2),
            (0, 255, 0),
            2
            )

        cv2.circle(
            frame,
            (xc, yc),
            5,
            (0, 0, 255),
            -1
                )

        cv2.putText(
            frame,
            f"Centro: ({xc}, {yc})",
            (xc + 10, yc),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2
        )

        cv2.putText(
        frame,
        f"Anterior: {last_points[0]}",
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (255, 255, 255),
        2
        )

        cv2.putText(
            frame,
            f"Atual: {last_points[1]}",
            (10, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2
        )

        cv2.imshow(
            "Deteccao da bolinha",
            frame
        )

def close():
    cap.release()
    cv2.destroyAllWindows

if __name__ == "__main__":
    pass