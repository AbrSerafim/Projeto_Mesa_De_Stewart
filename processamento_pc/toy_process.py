""" Code to a process time """

from Timer import PipelineTimer
import cv2

import ping_pong_yolo_mod as ppm

timer = PipelineTimer()

ppm.setup()

while True:

    # 1. Capture image
    with timer.measure("Image capture"):
        frame = ppm.capture_img()

    # 2. Inference
    with timer.measure("Inference"):
        results = ppm.inference(frame)

    # 3. Bbox process
    with timer.measure("Bbox process"):
        bbox = ppm.extract_result(results)

    # 4. Process results
    with timer.measure("Process results"):
        box = ppm.process_result(bbox)

    # 5. Draw results
    with timer.measure("Drawing"):
        ppm.draw(box, frame)

    if cv2.waitKey(1) & 0xFF in (27, ord('q'), ord('Q')) :
        break

ppm.close()
timer.report()