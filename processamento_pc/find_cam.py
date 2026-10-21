import cv2

for index in range(10):
    cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)  # Windows

    if cap.isOpened():
        ret, frame = cap.read()

        if ret:
            height, width = frame.shape[:2]
            print(f"Camera {index}: detected ({width}x{height})")
        else:
            print(f"Camera {index}: opened, but no frame received")

        cap.release()