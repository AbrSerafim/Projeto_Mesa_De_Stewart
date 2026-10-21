""" Converts a .pt to .onnx and .engine """

from pathlib import Path
from ultralytics import YOLO

# ============================================================
# CONFIGURATION
# ============================================================

MODEL_PATH = Path("yolo26n.engine")  # Path to your trained YOLO model
IMAGE_SIZE = 640              # Input image size: 640x640
DEVICE = 0                    # First NVIDIA GPU
HALF = True                   # Use FP16 for TensorRT

# ============================================================
# VALIDATION
# ============================================================

if not MODEL_PATH.is_file():
    raise FileNotFoundError(f"Model not found: {MODEL_PATH}")

model = YOLO(str(MODEL_PATH))

# ============================================================
# STEP 1: PyTorch (.pt) -> ONNX (.onnx)
# ============================================================

onnx_path = model.export(
    format="onnx",
    imgsz=IMAGE_SIZE,
    device=DEVICE,
    dynamic=False,
)

print(f"\nONNX model created: {onnx_path}")

# ============================================================
# STEP 2: ONNX -> TensorRT (.engine)
# ============================================================

engine_path = model.export(
    format="engine",
    imgsz=IMAGE_SIZE,
    device=DEVICE,
    dynamic=False,
    workspace=8,
)

print(f"\nTensorRT engine created: {engine_path}")
print("\nExport completed successfully!")