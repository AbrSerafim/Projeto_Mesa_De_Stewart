from ultralytics import YOLO
import cv2
import numpy as np
import time



# =========================
# CONFIGURAÇÕES
# =========================

MODEL_PATH = "yolo26n.pt"

CONF = 0.5

# Carrega YOLO
model = YOLO(MODEL_PATH)

# Abre a webcam
cap = cv2.VideoCapture(0)

# Verifica se a câmera abriu
if not cap.isOpened():
    print("Erro ao abrir a webcam.")
    exit()

# Dois últimos pontos
ultimos_pontos = []




# =========================
# LOOP DA CÂMERA
# =========================

while True:

    # Captura frame
    ret, frame = cap.read()

    if not ret:
        print("Erro ao capturar frame.")
        break

    # =========================
    # YOLO
    # =========================

    results = model(
        frame,
        conf=CONF,
        verbose=False
    )

    # Verifica se encontrou alguma coisa
    if len(results[0].boxes) > 0:

        boxes = results[0].boxes

        # Confianças
        confs = boxes.conf.cpu().numpy()

        # Detecção com maior confiança
        melhor_idx = np.argmax(confs)

        # Coordenadas da bounding box
        box = boxes.xyxy[
            melhor_idx
        ].cpu().numpy()

        x1, y1, x2, y2 = box

        # =========================
        # CENTRO DA BOUNDING BOX
        # =========================

        xc = int((x1 + x2) / 2)
        yc = int((y1 + y2) / 2)
        tempo = time.time()

        ponto_atual = (xc, yc,tempo)

        # Guarda ponto
        ultimos_pontos.append(ponto_atual)

        # Mantém apenas os dois últimos
        if len(ultimos_pontos) > 2:
            ultimos_pontos.pop(0)

        # =========================
        # DESENHO
        # =========================

        cv2.rectangle(
            frame,
            (int(x1), int(y1)),
            (int(x2), int(y2)),
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

    # =========================
    # MOSTRA OS DOIS PONTOS
    # =========================

    if len(ultimos_pontos) == 2:

        anterior = ultimos_pontos[0]
        atual = ultimos_pontos[1]

        cv2.putText(
            frame,
            f"Anterior: {anterior}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2
        )

        cv2.putText(
            frame,
            f"Atual: {atual}",
            (10, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2
        )

    # =========================
    # MOSTRA FRAME
    # =========================

    cv2.imshow(
        "Deteccao da bolinha",
        frame
    )

    # ESC para sair
    if cv2.waitKey(1) & 0xFF == 27:
        break


# =========================
# FINALIZA
# =========================

cap.release()
cv2.destroyAllWindows()