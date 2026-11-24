"""Prévia da câmera e captura local de fotos para o dataset."""

from datetime import datetime
from pathlib import Path
import sys
from tempfile import NamedTemporaryFile
from threading import Event, Thread
from time import monotonic
import tkinter as tk
from tkinter import ttk
from typing import NamedTuple
from uuid import uuid4

from PIL import Image, ImageTk

from ball_control import MAX_AGE, BallControl, ball_center
from yolo_detection import CUSTOM_WEIGHTS, EXAMPLE_CLASSES, CameraDetector


PHOTO_DIRECTORY = Path(__file__).resolve().parent / "fotos yolo"
STANDARD_MODEL = "YOLO26n (COCO)"
CUSTOM_MODEL = "best.pt (bola)"


class CaptureMode(NamedTuple):
    width: int
    height: int
    fourcc: str = ""
    fps: int = 0


def directshow_modes(index):
    from comtypes import CoInitialize, CoUninitialize
    from pygrabber.dshow_graph import FilterGraph

    CoInitialize()
    try:
        graph = FilterGraph()
        try:
            graph.add_video_input_device(index)
            modes = graph.get_input_device().get_formats()
            return {CaptureMode(mode["width"], abs(mode["height"]), mode["media_type_str"].upper())
                    for mode in modes if mode["width"] > 0 and mode["height"] != 0}
        finally:
            graph.remove_filters()
    finally:
        CoUninitialize()


def save_photo(cv2, frame):
    ok, png = cv2.imencode(".png", frame, [cv2.IMWRITE_PNG_COMPRESSION, 1])
    if not ok:
        raise OSError("Falha ao codificar a imagem.")
    PHOTO_DIRECTORY.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    path = PHOTO_DIRECTORY / f"mesa_{timestamp}_{uuid4().hex[:8]}.png"
    temporary = None
    try:
        with NamedTemporaryFile(dir=PHOTO_DIRECTORY, suffix=".part", delete=False) as output:
            temporary = Path(output.name)
            output.write(png.tobytes())
        temporary.rename(path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
    return path


class CameraCapture:
    def __init__(self, index):
        self.index = index
        self.latest_sample = None
        self.error = ""
        self.photo_requested = Event()
        self.photo_result = ""
        self.stop = Event()
        self.thread = Thread(target=self.run, name="camera-preview", daemon=True)
        self.thread.start()

    @property
    def latest(self):
        sample = self.latest_sample
        return sample[0] if sample is not None else None

    def run(self):
        try:
            import cv2
        except ImportError:
            self.error = "OpenCV indisponível. Instale as dependências de requirements.txt."
            return

        stream = self.open_preview(cv2)
        if stream is None:
            if not self.stop.is_set():
                self.error = f"Câmera {self.index} indisponível. Confira o índice e outros aplicativos em uso."
            return
        camera, frame = stream
        photo_mode = False
        try:
            while frame is not None and not self.stop.is_set():
                self.latest_sample = (frame, monotonic())
                if self.photo_requested.is_set():
                    if not photo_mode:
                        height, width = frame.shape[:2]
                        backend, modes = self.photo_modes(cv2, camera.getBackendName())
                        larger_modes = [mode for mode in modes if mode.width * mode.height > width * height]
                        if larger_modes:
                            camera.release()
                            stream = self.open_for_photos(cv2, backend, larger_modes)
                            if stream is None:
                                self.photo_result = "Câmera indisponível para tirar foto."
                                self.photo_requested.clear()
                                break
                            camera, frame = stream
                            self.latest_sample = (frame, monotonic())
                        photo_mode = True
                    if not self.stop.is_set():
                        self.take_photo(cv2, frame)
                frame = self.read_frame(cv2, camera)
        finally:
            camera.release()
        if not self.stop.is_set():
            self.error = "A câmera parou de enviar imagens. Abra novamente para reconectar."

    def read_frame(self, cv2, camera):
        for _ in range(5):
            if self.stop.is_set():
                return None
            try:
                ok, frame = camera.read()
            except cv2.error:
                return None
            if ok and frame is not None and frame.size:
                return frame
            self.stop.wait(.02)
        return None

    def open_stream(self, cv2, backend, mode=None, verify_size=False):
        if self.stop.is_set():
            return None
        camera = cv2.VideoCapture()
        keep = False
        try:
            if not camera.open(self.index, backend):
                return None
            if mode is not None:
                camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                if len(mode.fourcc) == 4:
                    camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*mode.fourcc))
                camera.set(cv2.CAP_PROP_FRAME_WIDTH, mode.width)
                camera.set(cv2.CAP_PROP_FRAME_HEIGHT, mode.height)
                if mode.fps:
                    camera.set(cv2.CAP_PROP_FPS, mode.fps)
            frame = self.read_frame(cv2, camera)
            if frame is None or (verify_size and frame.shape[1] * frame.shape[0] < mode.width * mode.height):
                return None
            keep = True
            return camera, frame
        except cv2.error:
            return None
        finally:
            if not keep:
                camera.release()

    def open_preview(self, cv2):
        backends = (cv2.CAP_DSHOW, cv2.CAP_MSMF, cv2.CAP_ANY) if sys.platform == "win32" else (cv2.CAP_ANY,)
        for backend in backends:
            for mode in (CaptureMode(640, 480, "MJPG", 60), None):
                stream = self.open_stream(cv2, backend, mode)
                if stream is not None:
                    return stream
        return None

    def photo_modes(self, cv2, backend_name):
        modes = set()
        backend = cv2.CAP_ANY
        if backend_name == "DSHOW":
            backend = cv2.CAP_DSHOW
            try:
                modes = directshow_modes(self.index)
            except (ImportError, OSError, RuntimeError, ValueError, IndexError, KeyError, AttributeError):
                pass
        elif backend_name == "MSMF":
            backend = cv2.CAP_MSMF
        if not modes:
            modes = {CaptureMode(w, h, "MJPG") for w, h in (
                (7680, 4320), (4096, 2160), (3840, 2160), (2560, 1920), (2560, 1440),
                (1920, 1080), (1600, 1200), (1280, 960), (1280, 720),
                (1024, 768), (960, 720), (800, 600), (640, 480),
            )}
        ranked = sorted(modes, key=lambda mode: (-mode.width * mode.height, mode.fourcc != "MJPG", mode))
        return backend, ranked

    def open_for_photos(self, cv2, backend, modes):
        for mode in modes:
            if self.stop.is_set():
                return None
            stream = self.open_stream(cv2, backend, mode, verify_size=True)
            if stream is not None:
                return stream
        return self.open_preview(cv2)

    def take_photo(self, cv2, frame):
        try:
            path = save_photo(cv2, frame)
            height, width = frame.shape[:2]
            self.photo_result = f"Salva em fotos yolo/{path.name} · {width} × {height}"
        except (OSError, cv2.error) as error:
            self.photo_result = f"Falha ao salvar a foto ({error})."
        finally:
            self.photo_requested.clear()


class CameraView:
    def __init__(self, parent, visible):
        self.parent = parent
        self.visible = visible
        self.capture = None
        self.detector = None
        self.submitted = None
        self.shown_result = None
        self.rendered = None
        self.rendered_boxes = ()
        self.rendered_markers = ()
        self.photo = None
        self.preview_bounds = None
        self.preview_dimensions = None
        self.crop = None
        self.raw_frame = self.vision_frame = None
        self.setpoint = None
        self.last_device = None
        self.space_down = False
        controls = ttk.Frame(parent, padding=(8, 6))
        controls.pack(fill="x")
        ttk.Label(controls, text="Dispositivo").pack(side="left")
        self.index = ttk.Spinbox(controls, from_=0, to=15, width=4)
        self.index.set("0")
        self.index.pack(side="left", padx=(5, 8))
        self.toggle = ttk.Button(controls, text="Abrir câmera", command=self.toggle_camera)
        self.toggle.pack(side="left")
        self.status = tk.StringVar(value="Câmera fechada")
        ttk.Label(controls, textvariable=self.status).pack(side="left", padx=10)
        self.detecting = tk.BooleanVar(value=True)
        detect_check = ttk.Checkbutton(controls, text="YOLO", variable=self.detecting,
                                       command=self.change_detection)
        detect_check.pack(side="left", padx=(8, 5))
        self.yolo_model = tk.StringVar(value=STANDARD_MODEL)
        model_box = ttk.Combobox(controls, textvariable=self.yolo_model, values=(STANDARD_MODEL, CUSTOM_MODEL),
                                 width=15, state="readonly")
        model_box.pack(side="left", padx=(0, 6))
        model_box.bind("<<ComboboxSelected>>", self.change_model)
        ttk.Label(controls, text="Classe").pack(side="left")
        self.classes = EXAMPLE_CLASSES
        self.selected_class = tk.StringVar(value=EXAMPLE_CLASSES[0])
        self.custom_class = tk.StringVar(value="bola")
        self.class_box = ttk.Combobox(controls, textvariable=self.selected_class, values=self.classes,
                                      width=17, state="readonly")
        self.class_box.pack(side="left", padx=5)
        self.class_box.bind("<<ComboboxSelected>>", self.change_detection)
        self.aruco_enabled = tk.BooleanVar(value=False)
        aruco_toggle = ttk.Checkbutton(controls, text="ArUco 5×5", variable=self.aruco_enabled,
                                       command=self.change_detection)
        aruco_toggle.pack(side="left", padx=(6, 4))
        self.detection_status = tk.StringVar()
        ttk.Label(controls, textvariable=self.detection_status).pack(side="left", padx=5)
        threshold_controls = ttk.Frame(parent, padding=(8, 0, 8, 5))
        threshold_controls.pack(fill="x")
        ttk.Label(threshold_controls, text="Confiança mínima YOLO").pack(side="left")
        self.confidence = tk.DoubleVar(value=20)
        self.confidence_label = tk.StringVar()
        threshold_slider = ttk.Scale(threshold_controls, from_=1, to=99, variable=self.confidence, length=160)
        threshold_slider.pack(side="left", padx=6)
        ttk.Label(threshold_controls, textvariable=self.confidence_label, width=5).pack(side="left")

        setpoint_controls = ttk.Frame(parent, padding=(8, 0, 8, 6))
        setpoint_controls.pack(fill="x")
        self.clear_button = ttk.Button(setpoint_controls, text="Limpar setpoint", command=self.clear_setpoint,
                                       state="disabled")
        self.clear_button.pack(side="left")
        self.setpoint_status = tk.StringVar(value="Clique na imagem para marcar o setpoint")
        ttk.Label(setpoint_controls, textvariable=self.setpoint_status).pack(side="left", padx=10)

        photo_controls = ttk.Frame(parent, padding=(8, 0, 8, 6))
        photo_controls.pack(fill="x")
        self.photo_button = ttk.Button(photo_controls, text="Tirar foto (Espaço)", command=self.request_photo,
                                       state="disabled")
        self.photo_button.pack(side="left")
        self.photo_status = tk.StringVar()
        ttk.Label(photo_controls, textvariable=self.photo_status).pack(side="left", padx=10)

        ball_toolbar = ttk.Frame(parent, padding=(8, 0, 8, 6))
        ball_toolbar.pack(fill="x")
        body = ttk.Frame(parent)
        body.pack(fill="both", expand=True)
        left = ttk.Frame(body, padding=(8, 0, 4, 0))
        right = ttk.Frame(body, padding=(4, 0, 8, 0))
        left.pack(side="left", fill="y")
        right.pack(side="right", fill="y")
        self.ball = BallControl(left, right, ball_toolbar, self.confidence)
        threshold_slider.configure(command=self.change_confidence)
        self.confidence_label.set(f"{self.confidence.get():.0f}%")
        background = ttk.Style(parent).lookup("TFrame", "background") or "#f0f0f0"
        self.canvas = tk.Canvas(body, bg=background, highlightthickness=0, cursor="crosshair")
        self.canvas.pack(fill="both", expand=True)
        self.preview = self.canvas.create_image(0, 0)
        for color, width in (("#151515", 4), ("#ff3030", 2)):
            for _ in range(2):
                self.canvas.create_line(0, 0, 0, 0, fill=color, width=width, tags="crosshair", state="hidden")
        self.canvas.bind("<Configure>", self.resize)
        self.canvas.bind("<Button-1>", self.pick_setpoint)
        root = parent.winfo_toplevel()
        root.bind("<KeyPress-space>", self.photo_shortcut, add="+")
        root.bind("<KeyRelease-space>", self.release_space, add="+")
        root.bind("<FocusOut>", self.release_space, add="+")
        # Run the camera shortcut before controls interpret Space as button activation.
        axis_widgets = tuple(widget for axis in self.ball.panels for widget in axis.frame.winfo_children())
        for widget in (self.toggle, self.index, detect_check, model_box, self.class_box, aruco_toggle, threshold_slider,
                       self.clear_button, self.photo_button, self.ball.button, self.ball.apply_button,
                       self.ball.save_button, *axis_widgets):
            widget.bindtags((str(root), *(tag for tag in widget.bindtags() if tag != str(root))))

    def resize(self, _event):
        self.rendered = None

    def change_confidence(self, value):
        self.confidence_label.set(f"{float(value):.0f}%")
        self.change_detection()

    def ball_class_selected(self):
        return self.detecting.get() and (self.yolo_model.get() == CUSTOM_MODEL
                                         or self.selected_class.get() == "sports ball")

    def prepare_ball(self, check_pico):
        if not self.visible() or self.capture is None or self.capture.stop.is_set():
            return "Abra a câmera primeiro."
        if not self.ball_class_selected():
            return "Ative YOLO com best.pt (bola) ou sports ball."
        if self.setpoint is None:
            return "Marque o setpoint na imagem."
        return check_pico()

    def useful_frame(self, raw):
        if raw is not self.raw_frame:
            width = raw.shape[1]
            if self.crop is None:
                # Only trim contiguous nearly black columns at the two image edges.
                import numpy as np

                columns = np.mean(np.max(raw, axis=2) <= 16, axis=0)
                useful = np.flatnonzero(columns < .98)
                if useful.size and useful[-1] - useful[0] >= width * .60:
                    self.crop = (int(useful[0]), int(useful[-1]) + 1)
            start, end = self.crop or (0, width)
            self.raw_frame = raw
            self.vision_frame = raw if (start, end) == (0, width) else raw[:, start:end]
        return self.vision_frame

    def request_photo(self):
        capture = self.capture
        if capture is None or capture.latest is None or capture.stop.is_set() or capture.photo_requested.is_set():
            return
        capture.photo_result = ""
        self.photo_button.state(["disabled"])
        self.photo_status.set("Preparando e salvando a foto…")
        capture.photo_requested.set()

    def photo_shortcut(self, _event):
        if self.visible() and self.capture is not None:
            if not self.space_down:
                self.space_down = True
                self.request_photo()
            return "break"
        return None

    def release_space(self, _event):
        self.space_down = False

    def toggle_camera(self):
        if self.capture is not None:
            self.capture.stop.set()
            self.toggle.configure(text="Fechando…", state="disabled")
            self.photo_button.state(["disabled"])
            self.status.set("Fechando câmera")
            self.clear_preview()
            self.clear_detection()
            return
        try:
            index = int(self.index.get())
            if not 0 <= index <= 2_147_483_647:
                raise ValueError
        except ValueError:
            self.status.set("Informe um índice de câmera a partir de zero.")
            return
        if self.last_device != index:
            self.clear_setpoint()
        self.last_device = index
        self.space_down = False
        self.photo_status.set("")
        self.capture = CameraCapture(index)
        self.index.configure(state="disabled")
        self.toggle.configure(text="Fechar câmera")
        self.status.set(f"Abrindo câmera {index}…")
        self.parent.after(0, self.tick)

    def clear_preview(self):
        self.canvas.itemconfigure(self.preview, image="")
        self.canvas.itemconfigure("crosshair", state="hidden")
        self.canvas.delete("detection", "aruco", "setpoint", "disparity")
        self.preview_bounds = None
        self.preview_dimensions = None
        self.crop = None
        self.raw_frame = self.vision_frame = None
        self.photo = self.rendered = None

    def clear_detection(self):
        self.ball.stop("Autocorreção parada · detecção interrompida")
        self.ball.frame_at = self.ball.observation = None
        self.submitted = self.shown_result = None
        self.detection_status.set("")
        self.canvas.delete("detection", "aruco", "disparity")
        if self.detector is not None:
            self.detector.clear()

    def change_detection(self, _event=None):
        self.clear_detection()
        self.rendered = None
        if self.detector is not None and ((self.detecting.get() and self.detector.error)
                                         or (self.aruco_enabled.get() and self.detector.aruco_error)):
            self.detector.close()
            self.detector = None

    def change_model(self, _event=None):
        self.clear_detection()
        self.rendered = None
        if self.detector is not None:
            self.detector.close()
            self.detector = None
        if self.yolo_model.get() == CUSTOM_MODEL:
            self.custom_class.set("bola")
            self.class_box.configure(textvariable=self.custom_class, values=(), state="disabled")
            self.classes = ()
        else:
            self.classes = EXAMPLE_CLASSES
            self.class_box.configure(textvariable=self.selected_class, values=self.classes, state="readonly")

    def pick_setpoint(self, event):
        if self.preview_bounds is None:
            return
        left, top, right, bottom = self.preview_bounds
        if not (left <= event.x < right and top <= event.y < bottom):
            return
        self.setpoint = ((event.x - left) / (right - left), (event.y - top) / (bottom - top))
        self.setpoint_status.set(f"Setpoint na imagem · X {self.setpoint[0]:.1%} · Y {self.setpoint[1]:.1%}")
        self.clear_button.configure(state="normal")
        self.draw_setpoint()

    def clear_setpoint(self):
        self.ball.stop("Autocorreção parada · setpoint removido")
        self.setpoint = None
        self.canvas.delete("setpoint", "disparity")
        self.setpoint_status.set("Clique na imagem para marcar o setpoint")
        self.clear_button.configure(state="disabled")

    def image_point(self, u, v):
        left, top, right, bottom = self.preview_bounds
        return left + u * (right - left), top + v * (bottom - top)

    def draw_setpoint(self):
        self.canvas.delete("setpoint", "disparity")
        if self.setpoint is not None and self.preview_bounds is not None:
            x, y = self.image_point(*self.setpoint)
            result = self.shown_result
            if (self.ball_class_selected() and result is not None and result.frame is self.rendered
                    and monotonic() - result.captured_at <= MAX_AGE):
                center = ball_center(result.boxes)
                if center is not None and center != self.setpoint:
                    start_x, start_y = self.image_point(*center)
                    for color, width in (("#111111", 5), ("#ffdb48", 2)):
                        self.canvas.create_line(start_x, start_y, x, y, fill=color, width=width,
                                                arrow="last", arrowshape=(10, 13, 5), tags="disparity")
            self.canvas.create_oval(x - 8, y - 8, x + 8, y + 8, outline="#111111", width=4, tags="setpoint")
            self.canvas.create_oval(x - 8, y - 8, x + 8, y + 8, outline="#51eaff", width=2, tags="setpoint")
            self.canvas.create_oval(x - 2, y - 2, x + 2, y + 2, fill="#51eaff", outline="", tags="setpoint")

    def draw_detections(self, boxes):
        self.canvas.delete("detection")
        for box in boxes:
            u0, v0, u1, v1 = box.bounds
            x0, y0 = self.image_point(u0, v0)
            x1, y1 = self.image_point(u1, v1)
            self.canvas.create_rectangle(x0, y0, x1, y1, outline="#55ff7b", width=2, tags="detection")
            self.canvas.create_text(x0 + 3, y0 + 3, anchor="nw", text=f"{box.confidence:.0%}",
                                    fill="#55ff7b", tags="detection")
            x, y = (x0 + x1) / 2, (y0 + y1) / 2
            self.canvas.create_oval(x - 4, y - 4, x + 4, y + 4, fill="#55ff7b",
                                    outline="#111111", tags="detection")

    def draw_aruco(self, markers):
        self.canvas.delete("aruco")
        for marker in markers:
            points = [self.image_point(u, v) for u, v in marker.corners]
            coords = [value for point in points for value in point]
            self.canvas.create_polygon(*coords, fill="", outline="#ffcb57", width=2, tags="aruco")
            center_x = sum(x for x, _ in points) / len(points)
            center_y = sum(y for _, y in points) / len(points)
            self.canvas.create_oval(center_x - 4, center_y - 4, center_x + 4, center_y + 4,
                                    fill="#ffcb57", outline="#111111", tags="aruco")
            x, y = points[0]
            self.canvas.create_text(x + 4, y + 4, anchor="nw", text=f"ID {marker.marker_id}",
                                    fill="#ffcb57", tags="aruco")

    def render(self, frame, boxes=(), markers=()):
        import cv2

        width, height = self.canvas.winfo_width(), self.canvas.winfo_height()
        if width < 2 or height < 2:
            return
        frame_height, frame_width = frame.shape[:2]
        scale = min(width / frame_width, height / frame_height)
        size = (max(1, round(frame_width * scale)), max(1, round(frame_height * scale)))
        if size != (frame_width, frame_height):
            interpolation = cv2.INTER_AREA if scale < 1 else cv2.INTER_LINEAR
            frame_for_display = cv2.resize(frame, size, interpolation=interpolation)
        else:
            frame_for_display = frame
        rgb = cv2.cvtColor(frame_for_display, cv2.COLOR_BGR2RGB)
        self.photo = ImageTk.PhotoImage(Image.fromarray(rgb), master=self.canvas)
        self.canvas.itemconfigure(self.preview, image=self.photo)

        x, y = width // 2, height // 2
        self.canvas.coords(self.preview, x, y)
        self.preview_bounds = self.canvas.bbox(self.preview)
        arm = min(20, size[0] // 2, size[1] // 2)
        bounds = ((x - arm, y, x + arm, y), (x, y - arm, x, y + arm))
        for index, item in enumerate(self.canvas.find_withtag("crosshair")):
            self.canvas.coords(item, *bounds[index % 2])
        self.canvas.itemconfigure("crosshair", state="normal")
        self.draw_detections(boxes)
        self.draw_aruco(markers)
        self.canvas.tag_raise("crosshair")
        self.rendered = frame
        self.rendered_boxes = boxes
        self.rendered_markers = markers
        self.draw_setpoint()

    def detected_frame(self, latest, captured_at=None):
        custom_model = self.yolo_model.get() == CUSTOM_MODEL
        class_name = None
        if self.detecting.get():
            class_name = "" if custom_model else self.selected_class.get()
        aruco_enabled = self.aruco_enabled.get()
        if class_name is None and not aruco_enabled:
            return latest, (), ()
        if self.detector is None:
            self.detector = CameraDetector(weights=CUSTOM_WEIGHTS if custom_model else None)
        detector = self.detector
        if self.classes != detector.names:
            self.classes = detector.names
            self.class_box.configure(values=self.classes)
            if custom_model and self.classes:
                self.custom_class.set(self.classes[0])
        if latest is not self.submitted:
            detector.submit(latest, class_name, aruco_enabled, self.confidence.get() / 100, captured_at)
            self.submitted = latest
        result = detector.result
        if result is not None and result.class_name == class_name and result.aruco_enabled == aruco_enabled:
            if result is not self.shown_result:
                status = []
                if class_name is not None:
                    status.append(detector.error or f"YOLO {len(result.boxes)}")
                if aruco_enabled:
                    status.append(detector.aruco_error or f"ArUco {len(result.markers)}")
                status.append(f"{result.duration_ms:.0f} ms")
                self.detection_status.set(" · ".join(status))
                self.shown_result = result
            return result.frame, result.boxes, result.markers
        self.detection_status.set(f"Carregando {detector.model_name}…" if class_name is not None and not detector.loaded
                                  else "Processando…")
        return latest, (), ()

    def tick(self):
        capture = self.capture
        if capture is None:
            return
        if not capture.thread.is_alive():
            self.capture = None
            self.clear_preview()
            self.clear_detection()
            self.index.configure(state="normal")
            self.photo_button.state(["disabled"])
            self.toggle.configure(text="Abrir câmera", state="normal")
            self.status.set(capture.error or "Câmera fechada")
            if capture.photo_result:
                self.photo_status.set(capture.photo_result)
            elif capture.photo_requested.is_set():
                self.photo_status.set("Captura da foto interrompida.")
            return
        if not capture.photo_requested.is_set() and capture.photo_result:
            self.photo_status.set(capture.photo_result)
            capture.photo_result = ""
        self.photo_button.state(["!disabled"] if capture.latest is not None and not capture.stop.is_set()
                                and not capture.photo_requested.is_set() else ["disabled"])
        if not capture.stop.is_set() and self.visible():
            sample = capture.latest_sample
            if sample is not None:
                frame, captured_at = sample
                height, width = frame.shape[:2]
                if self.preview_dimensions != (width, height):
                    if self.preview_dimensions is not None:
                        self.clear_detection()
                        self.clear_setpoint()
                    self.crop = None
                    self.raw_frame = None
                    self.preview_dimensions = (width, height)
                    self.status.set(f"Câmera {capture.index} · {width} × {height}")
                frame, boxes, markers = self.detected_frame(self.useful_frame(frame), captured_at)
                if frame is not self.rendered or boxes != self.rendered_boxes or markers != self.rendered_markers:
                    self.render(frame, boxes, markers)
            detector_ok = (self.ball_class_selected() and self.detector is not None and self.detector.loaded
                           and not self.detector.error)
            now = monotonic()
            result = self.shown_result if detector_ok else None
            if (result is None or now - result.captured_at > MAX_AGE) and self.canvas.find_withtag("disparity"):
                self.canvas.delete("disparity")
            self.ball.update(now, result, self.setpoint)
        elif self.submitted is not None:
            self.clear_detection()
            self.rendered = None
        self.parent.after(12, self.tick)

    def close(self):
        self.ball.stop("Autocorreção encerrada")
        if self.capture is not None:
            self.capture.stop.set()
        if self.detector is not None:
            self.detector.close()
            self.detector.thread.join(timeout=1)
