""" Code to a process time """

from Timer import PipelineTimer
import cv2

import ping_pong_yolo_mod as ppy
import ping_pong_opencv_mod as ppc

timer = PipelineTimer()

MODE = "opencv" # "yolo" || "opencv"

if MODE == "yolo":
    ppy.setup()

    while True:

        # 1. Capture image
        with timer.measure("Image capture"):
            frame = ppy.capture_img()

        # 2. Inference
        with timer.measure("Inference"):
            results = ppy.inference(frame)

        # 3. Bbox process
        with timer.measure("Bbox process"):
            bbox = ppy.extract_result(results)

        # 4. Process results
        with timer.measure("Process results"):
            box = ppy.process_result(bbox)

        # 5. Draw results
        with timer.measure("Drawing"):
            ppy.draw(box, frame)

        if cv2.waitKey(1) & 0xFF in (27, ord('q'), ord('Q')) :
            break

    ppy.close()
    timer.report()

elif MODE == "opencv":
    ppc.setup()

    while True:

        # 1. Capture image
        with timer.measure("Image capture"):
            frame = ppc.capture_img()

        # 2. Preprocess image
        with timer.measure("Preprocess image"):
            mask = ppc.preprocess(frame)

        # 3. Filter
        with timer.measure("Filter"):
            mask = ppc.filter(mask)

        # 4. Find contour
        with timer.measure("Find contour"):
            contour = ppc.find_contour(mask)

        # 5. Find centroid
        with timer.measure("Find centroid"):
            cg = ppc.calculate_centroid(contour)

        # 6. Draw
        with timer.measure("Drawing"):
            ppc.draw(cg, contour, mask, frame)

        if cv2.waitKey(1) & 0xFF in (27, ord('q'), ord('Q')) :
            break

    ppc.close()
    timer.report()

else:
    raise ValueError("Invalid mode")