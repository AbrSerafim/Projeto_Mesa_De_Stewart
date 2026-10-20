import cv2
import numpy as np
import time

# ==========================================
# CONFIGURAÇÕES
# ==========================================

CAMERA_ID = 0

# Faixa de cor laranja em HSV
LOWER_ORANGE = np.array([0, 160, 160])
UPPER_ORANGE = np.array([30, 255, 255])

# Área mínima para considerar um objeto
MIN_AREA = 100


# ==========================================
# ABRE A WEBCAM
# ==========================================

cap = cv2.VideoCapture(CAMERA_ID)

if not cap.isOpened():
    print("Erro ao abrir a webcam.")
    exit()


# ==========================================
# ARMAZENA OS DOIS ÚLTIMOS CENTROIDES
# ==========================================

ultimos_pontos = []


# ==========================================
# LOOP
# ==========================================

while True:

    ret, frame = cap.read()

    if not ret:
        print("Erro ao capturar frame.")
        break

    # ======================================
    # BGR -> HSV
    # ======================================

    hsv = cv2.cvtColor(
        frame,
        cv2.COLOR_BGR2HSV
    )

    # ======================================
    # MÁSCARA DA COR LARANJA
    # ======================================

    mask = cv2.inRange(
        hsv,
        LOWER_ORANGE,
        UPPER_ORANGE
    )

    # ======================================
    # FILTRO MORFOLÓGICO
    # ======================================

    # kernel = np.ones((5, 5), np.uint8)

    # mask = cv2.morphologyEx(
    #     mask,
    #     cv2.MORPH_OPEN,
    #     kernel
    # )

    # mask = cv2.morphologyEx(
    #     mask,
    #     cv2.MORPH_CLOSE,
    #     kernel
    # )




# ======================================
# FILTRO MORFOLÓGICO2
# ======================================

    kernel_close = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE,
        (7, 7)
    )

    kernel_open = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE,
        (3, 3)
    )

    # Primeiro, preenche pequenas falhas internas
    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_CLOSE,
        kernel_close
    )

    # Depois, remove pequenos ruídos
    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_OPEN,
        kernel_open
    )





    # ======================================
    # ENCONTRA CONTORNOS
    # ======================================

    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    melhor_contorno = None
    maior_area = 0

    # ======================================
    # SELECIONA O MAIOR CONTORNO
    # ======================================

    for contour in contours:

        area = cv2.contourArea(contour)

        if area > MIN_AREA and area > maior_area:

            maior_area = area
            melhor_contorno = contour

    # ======================================
    # CALCULA CENTROIDE
    # ======================================

    if melhor_contorno is not None:

        # Momentos do contorno
        M = cv2.moments(
            melhor_contorno
        )

        # Evita divisão por zero
        if M["m00"] != 0:

            # ==================================
            # CENTROIDE
            # ==================================

            xc = int(
                M["m10"] / M["m00"]
            )

            yc = int(
                M["m01"] / M["m00"]
            )

            # Tempo da detecção
            timestamp = time.time()

            # Ponto atual
            ponto_atual = (
                xc,
                yc,
                timestamp
            )

            # Adiciona aos pontos
            ultimos_pontos.append(
                ponto_atual
            )

            # Mantém somente os dois últimos
            if len(ultimos_pontos) > 2:
                ultimos_pontos.pop(0)

            # ==================================
            # DESENHA CONTORNO
            # ==================================

            cv2.drawContours(
                frame,
                [melhor_contorno],
                -1,
                (0, 255, 0),
                2
            )

            # ==================================
            # DESENHA CENTROIDE
            # ==================================

            cv2.circle(
                frame,
                (xc, yc),
                6,
                (0, 0, 255),
                -1
            )

            # ==================================
            # MOSTRA COORDENADAS
            # ==================================

            cv2.putText(
                frame,
                f"Centroide: ({xc}, {yc})",
                (xc + 10, yc),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2
            )

    # ======================================
    # CALCULA DESLOCAMENTO E VELOCIDADE
    # ======================================

    if len(ultimos_pontos) == 2:

        x1, y1, t1 = ultimos_pontos[0]
        x2, y2, t2 = ultimos_pontos[1]

        # Intervalo de tempo
        dt = t2 - t1

        if dt > 0:

            # Deslocamento em pixels
            dx = x2 - x1
            dy = y2 - y1

            # Velocidade em pixels/s
            vx = dx / dt
            vy = dy / dt

            # ==================================
            # MOSTRA INFORMAÇÕES
            # ==================================

            cv2.putText(
                frame,
                f"Anterior: ({x1}, {y1})",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2
            )

            cv2.putText(
                frame,
                f"Atual: ({x2}, {y2})",
                (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2
            )

            cv2.putText(
                frame,
                f"dx: {dx} px",
                (10, 90),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2
            )

            cv2.putText(
                frame,
                f"dy: {dy} px",
                (10, 120),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2
            )

            cv2.putText(
                frame,
                f"Vx: {vx:.1f} px/s",
                (10, 150),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2
            )

            cv2.putText(
                frame,
                f"Vy: {vy:.1f} px/s",
                (10, 180),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2
            )

    # ======================================
    # MOSTRA IMAGEM
    # ======================================

    cv2.imshow(
        "Deteccao da Bolinha",
        frame
    )

    # Mostra máscara
    cv2.imshow(
        "Mascara Laranja",
        mask
    )

    # ======================================
    # ESC PARA SAIR
    # ======================================

    if cv2.waitKey(1) & 0xFF == 27:
        break


# ==========================================
# FINALIZA
# ==========================================

cap.release()
cv2.destroyAllWindows()