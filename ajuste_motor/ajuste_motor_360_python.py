import cv2
import numpy as np
import serial, time

# ==============================
# CONFIGURACIÓN GENERAL
# ==============================
CAM_INDEX = 0
TARGET_W, TARGET_H = 1280, 720

# Marca virtual donde debe alinearse el plato
MARK_FRAC = 0.80      # 80% del ancho (derecha)
TOL = 25              # tolerancia en pixeles

# Cooldown para evitar saturar COM
LAST_SEND = 0
SEND_DELAY = 0.10     # 100 ms

# Serial a Arduino
SERIAL_PORT = "COM4"
BAUD = 9600
ser = serial.Serial(SERIAL_PORT, BAUD, timeout=0.05)
time.sleep(2)
ser.reset_input_buffer()
ser.reset_output_buffer()

# Línea de pre-disparo (para detectar M&M)
PRETRIGGER_FRAC = 0.75
COOLDOWN_COLOR = 0.5
LAST_COLOR_T = 0
LAST_COLOR = None

# ==============================
# RANGOS COLOR HSV
# ==============================
COLOR_RANGES = {
    "rojo_1":   (np.array([0,   120, 70]), np.array([10, 255, 255])),
    "rojo_2":   (np.array([170, 120, 70]), np.array([179,255,255])),
    "verde":    (np.array([40,  80, 60]), np.array([80,255,255])),
    "azul":     (np.array([80, 120, 70]), np.array([140,255,255])),
    "amarillo": (np.array([20, 120, 80]), np.array([32,255,255])),
}
ACTIVE_COLS = ["rojo", "verde", "azul", "amarillo"]
AREA_MIN = 800

# Color objetivo actual (se actualiza con el M&M detectado)
TARGET_COLOR = "rojo"

# ==============================
# FUNCIONES
# ==============================

def get_color_mask(hsv, name):
    """Devuelve máscara HSV para un color específico"""
    if name == "rojo":
        m1 = cv2.inRange(hsv, *COLOR_RANGES["rojo_1"])
        m2 = cv2.inRange(hsv, *COLOR_RANGES["rojo_2"])
        mask = cv2.bitwise_or(m1, m2)
    else:
        lo, hi = COLOR_RANGES[name]
        mask = cv2.inRange(hsv, lo, hi)

    k = np.ones((5,5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
    mask = cv2.dilate(mask, k)
    return mask

def find_colored_objects(hsv):
    """Detecta objetos de colores (M&Ms en la banda)"""
    objs = []
    for name in ACTIVE_COLS:
        mask = get_color_mask(hsv, name)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in cnts:
            if cv2.contourArea(c) < AREA_MIN:
                continue
            x,y,w,h = cv2.boundingRect(c)
            M = cv2.moments(c)
            if M["m00"] == 0:
                continue
            cx = int(M["m10"]/M["m00"])
            cy = int(M["m01"]/M["m00"])
            objs.append({"color": name, "cx": cx, "cy": cy, "bbox": (x,y,w,h)})
    return objs

def choose_next_mm(objs, exit_pos):
    """Qué M&M viene más cerca de la salida"""
    best = None
    best_dist = 9999
    for o in objs:
        dist = exit_pos - o["cy"]
        if 0 < dist < best_dist:
            best = o
            best_dist = dist
    return best

def get_plate_color_cx(hsv, color_name):
    """Devuelve el centroide X del color del PLATO"""
    mask = get_color_mask(hsv, color_name)
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if len(cnts)==0:
        return None
    c = max(cnts, key=cv2.contourArea)
    M = cv2.moments(c)
    if M["m00"]==0:
        return None
    return int(M["m10"]/M["m00"])

def send_servo(cmd):
    global LAST_SEND
    now = time.time()
    if now - LAST_SEND < SEND_DELAY:
        return
    try:
        ser.write(cmd.encode())
        LAST_SEND = now
    except:
        print("⚠ Error enviando comando serial (¿Arduino se reinició?)")

def align_plate(cx, mark_x):
    """Alinea el plato según el color objetivo"""
    if cx is None:
        send_servo('S')
        return

    error = cx - mark_x

    if abs(error) < TOL:
        send_servo('S')
    elif error > 0:
        send_servo('L')
    else:
        send_servo('R')

# ==============================
# LOOP PRINCIPAL
# ==============================

cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_DSHOW)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, TARGET_W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, TARGET_H)

while True:
    ok, frame = cap.read()
    if not ok:
        break

    frame = cv2.flip(frame, 1)
    H,W = frame.shape[:2]

    mark_x = int(W * MARK_FRAC)
    exit_y = int(H * PRETRIGGER_FRAC)

    hsv = cv2.cvtColor(cv2.GaussianBlur(frame,(5,5),0), cv2.COLOR_BGR2HSV)

    # 1. Detectar M&Ms en la banda
    objs = find_colored_objects(hsv)
    nxt = choose_next_mm(objs, exit_y)

    # 2. Si un M&M cruzó la línea -> fijar TARGET_COLOR
    global LAST_COLOR, LAST_COLOR_T
    if nxt:
        if nxt["cy"] >= exit_y:
            now = time.time()
            if now - LAST_COLOR_T > COOLDOWN_COLOR:
                TARGET_COLOR = nxt["color"]
                LAST_COLOR_T = now
                LAST_COLOR = TARGET_COLOR
                print("Nuevo M&M detectado → Color objetivo:", TARGET_COLOR)

    # 3. Alinear color del plato con la marca virtual
    cx_plate = get_plate_color_cx(hsv, TARGET_COLOR)
    align_plate(cx_plate, mark_x)

    # 4. Dibujar
    cv2.line(frame,(mark_x,0),(mark_x,H),(0,255,255),2)
    cv2.line(frame,(0,exit_y),(W,exit_y),(255,0,0),2)

    if nxt:
        x,y,w,h = nxt["bbox"]
        cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
        cv2.putText(frame, f"M&M: {nxt['color']}", (x,y-5),
                    cv2.FONT_HERSHEY_SIMPLEX,0.7,(255,255,255),2)

    cv2.putText(frame, 
                f"Objetivo del plato: {TARGET_COLOR}",
                (20,40), cv2.FONT_HERSHEY_SIMPLEX,0.8,(255,255,255),2)

    cv2.imshow("Clasificación completa", frame)
    k = cv2.waitKey(1) & 0xFF
    if k==27:
        break

cap.release()
ser.close()
cv2.destroyAllWindows()
