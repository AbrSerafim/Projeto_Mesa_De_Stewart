"""Detecção local com uma única imagem pendente."""

import os
from pathlib import Path
from threading import Condition, Thread
from time import monotonic
from typing import NamedTuple

from numpy import ndarray


EXAMPLE_CLASSES = ("sports ball", "apple")
CUSTOM_WEIGHTS = Path(__file__).resolve().with_name("best.pt")


class Detection(NamedTuple):
    bounds: tuple[float, float, float, float]
    confidence: float


class ArucoMarker(NamedTuple):
    marker_id: int
    corners: tuple[tuple[float, float], ...]


class DetectionResult(NamedTuple):
    frame: ndarray
    class_name: str | None
    boxes: tuple[Detection, ...]
    duration_ms: float
    markers: tuple[ArucoMarker, ...] = ()
    aruco_enabled: bool = False
    captured_at: float = 0


def load_model(weights=None):
    from ultralytics import YOLO

    if weights is not None:
        if not weights.is_file():
            raise FileNotFoundError("Coloque best.pt na pasta do projeto.")
        return YOLO(str(weights))
    cache = Path(os.environ.get("LOCALAPPDATA") or Path.home() / ".cache") / "StewartCamera"
    cache.mkdir(parents=True, exist_ok=True)
    return YOLO(str(cache / "yolo26n.pt"))


def detect(model, frame, class_name, class_id, confidence=.20):
    start = monotonic()
    result = model.predict(frame, classes=[class_id], imgsz=640, conf=confidence, save=False, verbose=False)[0]
    boxes = result.boxes.cpu()
    detections = tuple(Detection(tuple(bounds), confidence)
                       for bounds, confidence in zip(boxes.xyxyn.tolist(), boxes.conf.tolist()))
    return DetectionResult(frame, class_name, detections, (monotonic() - start) * 1000)


def make_aruco_detector():
    from cv2 import aruco

    dictionary = aruco.getPredefinedDictionary(aruco.DICT_5X5_1000)
    return aruco.ArucoDetector(dictionary, aruco.DetectorParameters())


def detect_aruco(detector, frame):
    import cv2

    height, width = frame.shape[:2]
    corners, ids, _ = detector.detectMarkers(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
    if ids is None:
        return ()
    return tuple(ArucoMarker(int(marker_id), tuple((float(x) / width, float(y) / height)
                                                   for x, y in points.reshape(4, 2)))
                 for marker_id, points in zip(ids.flat, corners))


class CameraDetector:
    def __init__(self, weights=None):
        self.weights = weights
        self.model_name = weights.name if weights is not None else "YOLO26n"
        self.names = () if weights is not None else EXAMPLE_CLASSES
        self.loaded = False
        self.error = ""
        self.aruco_error = ""
        self.result = None
        self.condition = Condition()
        self.generation = 0
        self.pending = None
        self.closed = False
        self.thread = Thread(target=self.run, name="camera-detection", daemon=True)
        self.thread.start()

    def submit(self, frame, class_name, aruco_enabled=False, confidence=.20, captured_at=None):
        with self.condition:
            self.pending = (self.generation, frame, class_name, aruco_enabled, confidence,
                            monotonic() if captured_at is None else captured_at)
            self.condition.notify()

    def clear(self):
        with self.condition:
            self.generation += 1
            self.pending = self.result = None

    def close(self):
        with self.condition:
            self.closed = True
            self.pending = self.result = None
            self.condition.notify()

    def run(self):
        model = aruco_detector = None
        classes = {}
        while True:
            with self.condition:
                self.condition.wait_for(lambda: self.closed or self.pending is not None)
                if self.closed:
                    return
                generation, frame, class_name, aruco_enabled, confidence, captured_at = self.pending
                self.pending = None
            start = monotonic()
            boxes = markers = ()
            if class_name is not None and not self.error:
                try:
                    if model is None:
                        model = load_model(self.weights)
                        if model.task != "detect":
                            raise ValueError("O arquivo precisa conter um modelo YOLO de detecção.")
                        classes = {name: index for index, name in model.names.items()}
                        if self.weights is not None:
                            if len(classes) != 1:
                                raise ValueError("best.pt precisa conter uma única classe.")
                            self.names = tuple(classes)
                        else:
                            self.names = (*EXAMPLE_CLASSES, *sorted(classes.keys() - set(EXAMPLE_CLASSES)))
                        self.loaded = True
                    model_class = self.names[0] if self.weights is not None else class_name
                    boxes = detect(model, frame, class_name, classes[model_class], confidence).boxes
                except ImportError:
                    self.error = "Instale as dependências de requirements.txt para usar o YOLO."
                except Exception as error:
                    detail = str(error).splitlines()[0] if str(error) else type(error).__name__
                    self.error = f"{self.model_name} indisponível ({detail})."
            if aruco_enabled and not self.aruco_error:
                try:
                    if aruco_detector is None:
                        aruco_detector = make_aruco_detector()
                    markers = detect_aruco(aruco_detector, frame)
                except Exception as error:
                    detail = str(error).splitlines()[0] if str(error) else type(error).__name__
                    self.aruco_error = f"ArUco indisponível ({detail})."
            result = DetectionResult(frame, class_name, boxes, (monotonic() - start) * 1000, markers,
                                     aruco_enabled, captured_at)
            with self.condition:
                if not self.closed and generation == self.generation:
                    self.result = result
