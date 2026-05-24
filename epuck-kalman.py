"""
INSTRUCCIONES:
  Cambiar la variable NAV_MODE para ejecutar cada
  experimento por separado y comparar resultados.
"""

from controller import Robot
import os

# MODO DE NAVEGACIÓN 
#   "raw"      - decisión basada en lecturas crudas del sensor
#   "filtered" - decisión basada en señal filtrada (promedio móvil)
#   "kalman"   - decisión basada en estimación del filtro de Kalman

NAV_MODE = "kalman"


# CONSTANTES DEL ROBOT E-PUCK

TIME_STEP = 64              # Paso de simulación (ms)
T_S = TIME_STEP / 1000.0    # Paso en segundos
F_S = 1.0 / T_S             # Frecuencia de muestreo (Hz)

WHEEL_RADIUS = 0.0205       # Radio de la rueda (m)
AXLE_LENGTH = 0.052         # Distancia entre ruedas (m)
MAX_SPEED = 6.28            # Velocidad angular máxima (rad/s)

# SENSORES DE PROXIMIDAD

#   ps0: frente-derecha  (10°)    ps7: frente-izquierda (-10°)
#   ps1: diagonal-derecha (45°)   ps6: diagonal-izquierda (-45°)
#   ps2: lateral-derecha  (90°)   ps5: lateral-izquierda (-90°)
#   ps3: atrás-derecha   (150°)   ps4: atrás-izquierda  (-150°)
SENSOR_NAMES = ['ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7']
IDX_FR, IDX_DR, IDX_LR = 0, 1, 2   # ps0, ps1, ps2
IDX_LL, IDX_DL, IDX_FL = 5, 6, 7   # ps5, ps6, ps7

SENSOR_MAX_RANGE = 0.12
SENSOR_MIN_DETECT = 65.0


# NAVEGACIÓN

SAFETY_DISTANCE = 0.06      # Umbral frontal (m)
FORWARD_SPEED = 4.0         # Velocidad base de avance (rad/s)
TURN_SPEED = 1.8            # Velocidad de giro in-situ (rad/s)
TURN_COMMIT_STEPS = 8       # Pasos mínimos de giro

# Atascamiento
STUCK_THRESHOLD = 0.0001
STUCK_COUNT_LIMIT = 20
REVERSE_DURATION = 12
REVERSE_SPEED = 2.0

# FILTROS

FILTER_WINDOW_SIZE = 5

# Kalman
Q_PROCESS = 0.0001
R_MEASUREMENT = 0.0005
INITIAL_P = 0.01
INITIAL_DISTANCE = SENSOR_MAX_RANGE


# SIMULACIÓN

MAX_STEPS = 5000
# Archivo de salida 
LOG_FILE = f"lab2_data_{NAV_MODE}.csv"


# FUNCIONES

class MovingAverageFilter:
    """Filtro de promedio móvil de ventana fija."""
    def __init__(self, window_size=FILTER_WINDOW_SIZE):
        self.window_size = window_size
        self.buffer = []

    def update(self, value):
        self.buffer.append(value)
        if len(self.buffer) > self.window_size:
            self.buffer.pop(0)
        return sum(self.buffer) / len(self.buffer)

    def get_value(self):
        if not self.buffer:
            return 0.0
        return sum(self.buffer) / len(self.buffer)


class KalmanFilter1D:
    """
    Filtro de Kalman escalar (1D) para distancia frontal.

    """
    def __init__(self, initial_estimate, initial_P, Q, R):
        self.d = initial_estimate
        self.P = initial_P
        self.Q = Q
        self.R = R
        self.K = 0.0
        self.d_predicted = initial_estimate

    def predict(self, delta_d):
        """Etapa de PREDICCIÓN con avance de encoders."""
        self.d = max(self.d - delta_d, 0.0)
        self.P = self.P + self.Q
        self.d_predicted = self.d

    def correct(self, z):
        """Etapa de CORRECCIÓN con medición de sensores."""
        self.K = self.P / (self.P + self.R)
        self.d = self.d + self.K * (z - self.d)
        self.P = (1.0 - self.K) * self.P

    def get_estimate(self):
        return self.d
    def get_gain(self):
        return self.K
    def get_prediction(self):
        return self.d_predicted
    def get_covariance(self):
        return self.P


# CONTROLADOR PRINCIPAL

def raw_to_distance(raw_value):
    """Convierte valor crudo del sensor a distancia aproximada (m).
    Relación inversa empírica: valor alto = obstáculo cerca."""
    
    if raw_value < SENSOR_MIN_DETECT:
        return SENSOR_MAX_RANGE
    distance = (1.0 / (0.003 * raw_value + 0.3)) * 0.03
    return max(0.005, min(distance, SENSOR_MAX_RANGE))


def main():
    robot = Robot()
    print("=" * 58)
    print(f"  MODO DE NAVEGACIÓN: {NAV_MODE.upper()}")
    print("=" * 58)
    print(f"  Ts={T_S}s | Fs={F_S}Hz | Umbral={SAFETY_DISTANCE}m")
    print(f"  Archivo de salida: {LOG_FILE}")
    print("=" * 58)

    # --- Motores ---
    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)

    # --- Sensores de proximidad ---
    sensors = []
    for name in SENSOR_NAMES:
        s = robot.getDevice(name)
        s.enable(TIME_STEP)
        sensors.append(s)

    # --- Encoders ---
    left_enc = robot.getDevice('left wheel sensor')
    right_enc = robot.getDevice('right wheel sensor')
    left_enc.enable(TIME_STEP)
    right_enc.enable(TIME_STEP)

    # --- Filtros ---
    filt_fr = MovingAverageFilter(FILTER_WINDOW_SIZE)   # ps0
    filt_fl = MovingAverageFilter(FILTER_WINDOW_SIZE)   # ps7

    kalman = KalmanFilter1D(INITIAL_DISTANCE, INITIAL_P, Q_PROCESS, R_MEASUREMENT)

    # --- Estado ---
    prev_enc_l = None
    prev_enc_r = None
    step = 0
    stuck_counter = 0
    reverse_counter = 0
    reverse_dir = 1
    turn_commit_counter = 0
    turn_commit_dir = 0

    # --- Log ---
    data_log = []
    headers = [
        "step", "time_s", "nav_mode",
        "raw_ps0", "raw_ps7", "raw_ps1", "raw_ps6", "raw_ps2", "raw_ps5",
        "dist_raw_front_R", "dist_raw_front_L", "dist_raw_front_avg",
        "dist_filtered_front_R", "dist_filtered_front_L", "dist_filtered_avg",
        "dist_diag_R", "dist_diag_L", "dist_lat_R", "dist_lat_L",
        "encoder_left", "encoder_right",
        "delta_left", "delta_right", "delta_advance",
        "kalman_prediction", "kalman_estimate", "kalman_gain", "kalman_P",
        "decision_distance",
        "action", "vel_left", "vel_right"
    ]

    print("[INFO] Bucle de control iniciado\n")

    while robot.step(TIME_STEP) != -1 and step < MAX_STEPS:
        t = step * T_S

        # ---- A. LECTURA CRUDA ----
        raw = [s.getValue() for s in sensors]

        # ---- B. CONVERSIÓN A DISTANCIA ----
        d_fr = raw_to_distance(raw[IDX_FR])   # ps0
        d_fl = raw_to_distance(raw[IDX_FL])   # ps7
        d_dr = raw_to_distance(raw[IDX_DR])   # ps1
        d_dl = raw_to_distance(raw[IDX_DL])   # ps6
        d_lr = raw_to_distance(raw[IDX_LR])   # ps2
        d_ll = raw_to_distance(raw[IDX_LL])   # ps5

        # Promedio crudo de frontales (para modo raw)
        raw_front_avg = (d_fr + d_fl) / 2.0

        # ---- C. FILTRO PROMEDIO MÓVIL (frontales) ----
        d_fr_filt = filt_fr.update(d_fr)
        d_fl_filt = filt_fl.update(d_fl)
        filtered_front_avg = (d_fr_filt + d_fl_filt) / 2.0

        # z_k para el Kalman: usa la señal filtrada como medición
        z_k = filtered_front_avg

        # ---- D. ENCODERS → AVANCE (s = r * theta) ----
        el = left_enc.getValue()
        er = right_enc.getValue()
        delta_adv = 0.0
        dl_enc = 0.0
        dr_enc = 0.0

        if prev_enc_l is not None:
            dl_enc = el - prev_enc_l
            dr_enc = er - prev_enc_r
            delta_adv = WHEEL_RADIUS * (dl_enc + dr_enc) / 2.0

        prev_enc_l = el
        prev_enc_r = er

        # ---- E. FILTRO DE KALMAN (siempre se ejecuta para registro) ----
        kalman.predict(delta_adv)        # Predicción
        kalman.correct(z_k)              # Corrección
        est_dist = kalman.get_estimate()
        k_gain = kalman.get_gain()
        k_pred = kalman.get_prediction()
        k_cov = kalman.get_covariance()

        # ---- SELECCIÓN DE DISTANCIA SEGÚN MODO ----
        if NAV_MODE == "raw":
            decision_dist = raw_front_avg
        elif NAV_MODE == "filtered":
            decision_dist = filtered_front_avg
        else:  # "kalman"
            decision_dist = est_dist

        # ---- F. NAVEGACIÓN REACTIVA ----
        lv = 0.0
        rv = 0.0
        action = "STOP"

        right_space = min(d_dr, d_lr)
        left_space = min(d_dl, d_ll)

        # ---- F0: REVERSA ----
        if reverse_counter > 0:
            lv = -REVERSE_SPEED
            rv = -REVERSE_SPEED * 0.3 * reverse_dir
            action = "REVERSE"
            reverse_counter -= 1
            if reverse_counter == 0:
                kalman.d = z_k
                kalman.P = INITIAL_P
                turn_commit_counter = TURN_COMMIT_STEPS
                turn_commit_dir = 1 if left_space >= right_space else -1

        # ---- F1: GIRO ARRIESGADO ----
        elif turn_commit_counter > 0:
            if turn_commit_dir < 0:
                lv = -TURN_SPEED
                rv = TURN_SPEED
                action = "TURN_LEFT"
            else:
                lv = TURN_SPEED
                rv = -TURN_SPEED
                action = "TURN_RIGHT"
            turn_commit_counter -= 1

        # ---- F2: OBSTÁCULO → GIRAR ----
        elif decision_dist <= SAFETY_DISTANCE:
            if left_space > right_space:
                turn_commit_dir = -1
                action = "TURN_LEFT"
            else:
                turn_commit_dir = 1
                action = "TURN_RIGHT"
            turn_commit_counter = TURN_COMMIT_STEPS

            if turn_commit_dir < 0:
                lv = -TURN_SPEED
                rv = TURN_SPEED
            else:
                lv = TURN_SPEED
                rv = -TURN_SPEED

        # ---- F3: LIBRE → AVANZAR con dirección proporcional ----
        else:
            lv = FORWARD_SPEED
            rv = FORWARD_SPEED
            action = "FORWARD"

            # Curvas suaves con sensores diagonales
            if d_dr < SENSOR_MAX_RANGE:
                influence_r = 1.0 - (d_dr / SENSOR_MAX_RANGE)
                rv = FORWARD_SPEED * (1.0 - 0.6 * influence_r)
                if influence_r > 0.2:
                    action = "FORWARD_CURVE_L"

            if d_dl < SENSOR_MAX_RANGE:
                influence_l = 1.0 - (d_dl / SENSOR_MAX_RANGE)
                lv = FORWARD_SPEED * (1.0 - 0.6 * influence_l)
                if influence_l > 0.2:
                    action = "FORWARD_CURVE_R"

        # ---- DETECCIÓN DE ATASCAMIENTO ----
        if reverse_counter == 0 and step > 10:
            if abs(delta_adv) < STUCK_THRESHOLD and \
               abs(dl_enc) < 0.001 and abs(dr_enc) < 0.001:
                stuck_counter += 1
            else:
                stuck_counter = 0

            if stuck_counter >= STUCK_COUNT_LIMIT:
                reverse_counter = REVERSE_DURATION
                stuck_counter = 0
                reverse_dir *= -1
                action = "STUCK"
                print(f"  [!] Atascamiento en step {step}")

        # Limitar y aplicar
        lv = max(-MAX_SPEED, min(lv, MAX_SPEED))
        rv = max(-MAX_SPEED, min(rv, MAX_SPEED))
        left_motor.setVelocity(lv)
        right_motor.setVelocity(rv)

        # ---- G. LOG ----
        data_log.append([
            step, round(t, 4), NAV_MODE,
            round(raw[IDX_FR], 2), round(raw[IDX_FL], 2),
            round(raw[IDX_DR], 2), round(raw[IDX_DL], 2),
            round(raw[IDX_LR], 2), round(raw[IDX_LL], 2),
            round(d_fr, 6), round(d_fl, 6), round(raw_front_avg, 6),
            round(d_fr_filt, 6), round(d_fl_filt, 6), round(filtered_front_avg, 6),
            round(d_dr, 6), round(d_dl, 6),
            round(d_lr, 6), round(d_ll, 6),
            round(el, 4), round(er, 4),
            round(dl_enc, 6), round(dr_enc, 6),
            round(delta_adv, 6),
            round(k_pred, 6), round(est_dist, 6),
            round(k_gain, 6), round(k_cov, 8),
            round(decision_dist, 6),
            action, round(lv, 2), round(rv, 2)
        ])

        if step % 50 == 0:
            print(f"[{step:4d}|{t:6.2f}s] mode={NAV_MODE} "
                  f"raw={raw_front_avg:.4f} filt={filtered_front_avg:.4f} "
                  f"kal={est_dist:.4f} dec={decision_dist:.4f} → {action}")

        step += 1

    # ---- GUARDAR CSV ----
    print(f"\n[INFO] Fin: {step} pasos, {step*T_S:.2f}s")
    csv_path = os.path.join(os.getcwd(), LOG_FILE)
    try:
        with open(csv_path, 'w', newline='') as f:
            f.write(','.join(headers) + '\n')
            for row in data_log:
                f.write(','.join(str(v) for v in row) + '\n')
        print(f"[INFO] CSV → {csv_path}")
    except Exception as e:
        print(f"[ERROR] CSV: {e}")

    print("[INFO] Controlador terminado.")


if __name__ == "__main__":
    main()