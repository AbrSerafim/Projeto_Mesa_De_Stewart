""" Library to run opencv detection and extract CG of a object """

import cv2
import numpy as np

# =========================
# Config
# =========================

CAMERA_ID = 1

# Orange color range (HSV)
LOWER_ORANGE = np.array([0, 160, 160])
UPPER_ORANGE = np.array([30, 255, 255])

# Min área to consider a object
MIN_AREA = 100


# =========================
# Functions & Classes
# =========================

def setup() -> None:
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

def preprocess(frame: cv2.typing.MatLike): return cv2.inRange(cv2.cvtColor(frame,cv2.COLOR_BGR2HSV), LOWER_ORANGE,UPPER_ORANGE)

def filter(mask): # Morphologic filter
    kernel_close = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE,
        (7, 7)
    )

    kernel_open = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE,
        (3, 3)
    )

    # Fill inner failures
    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_CLOSE,
        kernel_close
    )

    # Remove noise
    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_OPEN,
        kernel_open
    )

    return mask

def find_contour(mask):
    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    best_contour = None

    if contours:
        areas = np.fromiter(
            (cv2.contourArea(c) for c in contours),
            dtype=np.float64,
            count=len(contours)
        )

        idx = np.argmax(areas)

        if areas[idx] > MIN_AREA:
            best_contour = contours[idx]

    return best_contour

def calculate_centroid(contour):
    if contour is not None:

        # Contour moments
        M = cv2.moments(
            contour
        )

        # Centroids
        xc = int(M["m10"] / (M["m00"] + 1e-6))
        yc = int(M["m01"] / (M["m00"] + 1e-6))

        cg = (xc, yc)

        last_points.append(cg)
        last_points.pop(0)

        return cg

def draw(cg, contour, mask, frame):
    if cg is None or contour is None or mask is None:
        cv2.imshow(
            "Deteccao da bolinha",
            frame
        )

    else:
        xc, yc = cg

        cv2.drawContours(
            frame,
            [contour],
            -1,
            (0, 255, 0),
            2
        )

        cv2.circle(
            frame,
            (xc, yc),
            6,
            (0, 0, 255),
            -1
        )

        cv2.putText(
            frame,
            f"Centroide: ({xc}, {yc})",
            (xc + 10, yc),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2
        )

        cv2.imshow(
            "Deteccao da bolinha",
            frame
        )

        cv2.imshow(
        "Mascara Laranja",
        mask
        )

def close():
    cap.release()
    cv2.destroyAllWindows

if __name__ == "__main__":
    pass