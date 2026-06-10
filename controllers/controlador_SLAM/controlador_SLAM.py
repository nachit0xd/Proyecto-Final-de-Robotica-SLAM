"""
controlador_SLAM.py

Línea B: SLAM / Mapeo Autónomo Simplificado

Hasta ahora, este controlador implementa:
  Fase 1:
    1. Configuración de motores diferenciales (velocidad continua).
    2. Lectura de encoders de rueda.
    3. Cálculo de odometría (x, y, theta) usando el modelo cinemático diferencial.
  Fase 2:
    4. Activación y lectura de los 8 sensores infrarrojos (ps0–ps7).
    5. Filtro de media móvil para estabilizar lecturas de sensores.
    6. Conversión de valores crudos a distancia en metros.
    7. Transformación de detecciones del marco local al marco global.
  Fase 3:
    8. Grilla de ocupación 2D con modelo log-odds.
    9. Trazado de rayos con algoritmo de Bresenham.
    10. Visualización del mapa generado (en formato PPM).
    11. Navegación reactiva básica para cubrir el entorno.
"""

import math
import os
from controller import Robot

# ============================================================================
# Constantes del e-puck
# ============================================================================
WHEEL_RADIUS = 0.0205        # radio de cada rueda [m]
AXLE_LENGTH = 0.052          # distancia entre ruedas (eje a eje) [m]
MAX_SPEED = 6.28             # velocidad angular máxima de cada motor [rad/s]

# Constantes de los sensores infrarrojos
SENSOR_NOMBRES = ['ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7']

# Ángulos de cada sensor respecto al frente del robot [rad]
#   ps0 ≈  10° (frontal derecho)     ps7 ≈ 150° (frontal izquierdo)
#   ps1 ≈  45°                       ps6 ≈ 210°
#   ps2 ≈  90° (lateral derecho)     ps5 ≈ 270° (lateral izquierdo / trasero)
#   ps3 ≈ 330° (trasero derecho)     ps4 ≈ 300° (trasero izquierdo)
SENSOR_ANGULOS = [1.27, 0.77, 0.00, 5.21, 4.21, 3.14, 2.37, 1.87]  # [rad]

SENSOR_UMBRAL_DETECCION = 80    # valor mínimo para considerar detección
SENSOR_MAX_VALUE = 4095.0       # valor máximo del sensor
SENSOR_MAX_RANGE = 0.04         # alcance efectivo máximo [m] (aprox. 4 cm)

FILTRO_VENTANA = 5              # tamaño de la ventana del filtro de media móvil

# Constantes de la grilla de ocupación 
GRID_RESOLUCION = 0.01          # 1 cm por celda [m/celda]
ARENA_ANCHO = 1.0               # ancho de la arena [m]
ARENA_ALTO = 1.0                # alto de la arena [m]
GRID_ANCHO = int(ARENA_ANCHO / GRID_RESOLUCION)   # 100 columnas
GRID_ALTO = int(ARENA_ALTO / GRID_RESOLUCION)      # 100 filas

# Offset: el punto origen de Webots (0,0) está en el centro del arena.
# Desplazamos las coordenadas para que la esquina inferior-izquierda del arena corresponda a la celda (0, 0) de la grilla.
GRID_OFFSET_X = ARENA_ANCHO / 2.0   # 0.5 m
GRID_OFFSET_Y = ARENA_ALTO / 2.0    # 0.5 m

# Parámetros del modelo log-odds
LOG_ODD_OCUPADO = 0.85       # incremento al detectar obstáculo
LOG_ODD_LIBRE = -0.40        # decremento para celdas libres
LOG_ODD_MAX = 5.0            # límite superior (alta confianza ocupado)
LOG_ODD_MIN = -5.0           # límite inferior (alta confianza libre)

# Intervalo para guardar el mapa [segundos de simulación]
MAPA_INTERVALO_GUARDADO = 5.0

# ============================================================================
# Inicialización del robot
# ============================================================================
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Inicializamos los motores diferenciales
motor_izq = robot.getDevice('left wheel motor')
motor_der = robot.getDevice('right wheel motor')

# Configuramos los motores para control por velocidad continua
motor_izq.setPosition(float('inf'))
motor_der.setPosition(float('inf'))

# Velocidad inicial detenida
motor_izq.setVelocity(0.0)
motor_der.setVelocity(0.0)

# Encoders (sensores de posición de las ruedas)
encoder_izq = robot.getDevice('left wheel sensor')
encoder_der = robot.getDevice('right wheel sensor')
encoder_izq.enable(timestep)
encoder_der.enable(timestep)

# Sensores infrarrojos de proximidad
sensores_ir = []
for nombre in SENSOR_NOMBRES:
    sensor = robot.getDevice(nombre)
    sensor.enable(timestep)
    sensores_ir.append(sensor)

# ============================================================================
# Variables de odometría
# ============================================================================
# Posición y orientación estimadas del robot en el marco global
x = 0.0       # posición X [m]
y = 0.0       # posición Y [m]
theta = 0.0   # orientación [rad]

# Valores previos de los encoders (inicializados en la primera lectura)
prev_encoder_izq = 0.0
prev_encoder_der = 0.0
primera_lectura = True

# ============================================================================
# Variables de percepción
# ============================================================================
# Historial de lecturas para el filtro de media móvil (una lista por sensor)
historial_sensores = [[] for _ in range(8)]

# ============================================================================
# Variables de la grilla de ocupación
# ============================================================================
# Grilla de ocupación: matriz 2D inicializada en 0.0 (incertidumbre total, p=0.5)
grilla = [[0.0] * GRID_ANCHO for _ in range(GRID_ALTO)]

# Ruta donde se guardará el mapa (en la carpeta del controlador)
RUTA_MAPA = os.path.join(os.path.dirname(__file__), "mapa_ocupacion.ppm")

# ============================================================================
# Funciones auxiliares de Odometría
# ============================================================================

def actualizar_odometria(enc_izq_actual, enc_der_actual):
    """
    Función que calcula el desplazamiento incremental del robot a partir de las
    lecturas actuales de los encoders y actualiza la posición global
    (x, y, theta).

    Ecuaciones del modelo cinemático diferencial:
        Δs_l = r * Δθ_l       (avance rueda izquierda)
        Δs_r = r * Δθ_r       (avance rueda derecha)
        Δs   = (Δs_r + Δs_l) / 2    (avance lineal del centro)
        Δφ   = (Δs_r - Δs_l) / L    (cambio de orientación)
        x_k  = x_{k-1} + Δs * cos(θ_{k-1} + Δφ/2)
        y_k  = y_{k-1} + Δs * sin(θ_{k-1} + Δφ/2)
        θ_k  = θ_{k-1} + Δφ
    """
    global x, y, theta, prev_encoder_izq, prev_encoder_der

    # Diferencia angular de cada rueda desde el último paso
    delta_enc_izq = enc_izq_actual - prev_encoder_izq
    delta_enc_der = enc_der_actual - prev_encoder_der

    # Distancia lineal recorrida por cada rueda
    delta_s_izq = WHEEL_RADIUS * delta_enc_izq
    delta_s_der = WHEEL_RADIUS * delta_enc_der

    # Avance lineal del centro del robot y cambio de orientación
    delta_s = (delta_s_der + delta_s_izq) / 2.0
    delta_theta = (delta_s_der - delta_s_izq) / AXLE_LENGTH

    # Actualizamos la posición global usando la orientación intermedia
    x += delta_s * math.cos(theta + delta_theta / 2.0)
    y += delta_s * math.sin(theta + delta_theta / 2.0)
    theta += delta_theta

    # Normalizamos theta al rango [-π, π]
    theta = math.atan2(math.sin(theta), math.cos(theta))

    # Guardamos las lecturas actuales para el siguiente paso
    prev_encoder_izq = enc_izq_actual
    prev_encoder_der = enc_der_actual


def set_velocidades(vel_izq, vel_der):
    """
    Función que establece las velocidades angulares de los motores izquierdo y derecho.
    Los valores se limitan al rango [-MAX_SPEED, MAX_SPEED].
    """
    vel_izq = max(-MAX_SPEED, min(MAX_SPEED, vel_izq))
    vel_der = max(-MAX_SPEED, min(MAX_SPEED, vel_der))
    motor_izq.setVelocity(vel_izq)
    motor_der.setVelocity(vel_der)


# ============================================================================
# Funciones auxiliares de Percepción
# ============================================================================

def filtrar_lectura(indice_sensor, valor_nuevo):
    """
    Función que aplica un filtro de media móvil al sensor indicado.
    Mantiene las últimas FILTRO_VENTANA lecturas y retorna su promedio.
    Esto permite la suavización del ruido de las mediciones infrarrojas.
    """
    historial = historial_sensores[indice_sensor]
    historial.append(valor_nuevo)
    if len(historial) > FILTRO_VENTANA:
        historial.pop(0)
    return sum(historial) / len(historial)


def valor_a_distancia(valor_crudo):
    """
    Función que convierte una lectura cruda del sensor IR a distancia en metros:
    - Si el valor está por debajo del umbral de detección, retorna None
      (no hay obstáculo dentro del rango).
    - Si hay detección, mapea linealmente el rango del sensor a distancia.
      Valor alto → obstáculo cerca, valor bajo → obstáculo lejos.
    """
    if valor_crudo < SENSOR_UMBRAL_DETECCION:
        return None  # sin detección
    distancia = SENSOR_MAX_RANGE * (1.0 - (valor_crudo / SENSOR_MAX_VALUE))
    return max(distancia, 0.005)  # mínimo 5 mm para evitar valores negativos


def obtener_puntos_obstaculos():
    """
    Función que lee todos los sensores IR, filtra las lecturas, convierte a distancia
    y transforma las detecciones del marco local del robot al marco global.

    Para cada sensor que detecta un obstáculo a distancia d, la posición
    global del obstáculo se calcula como:
        x_obs = x_robot + d * cos(θ_robot + α_sensor)
        y_obs = y_robot + d * sin(θ_robot + α_sensor)

    Retorna una lista de tuplas (x_obs, y_obs) para cada detección válida.
    """
    puntos = []
    for i in range(8):
        valor_crudo = sensores_ir[i].getValue()
        valor_filtrado = filtrar_lectura(i, valor_crudo)
        distancia = valor_a_distancia(valor_filtrado)

        if distancia is not None:
            angulo_global = theta + SENSOR_ANGULOS[i]
            x_obs = x + distancia * math.cos(angulo_global)
            y_obs = y + distancia * math.sin(angulo_global)
            puntos.append((x_obs, y_obs))

    return puntos


# ============================================================================
# Funciones auxiliares de Mapeo
# ============================================================================

def mundo_a_grilla(x_mundo, y_mundo):
    """
    Función que convierte las coordenadas del mundo de Webots (metros) a índices
    (fila, columna) de la grilla de ocupación.

    El punto origen de Webots (0, 0) está en el centro del arena, por lo que
    se aplica un offset para que la esquina inferior-izquierda del arena
    corresponda a la celda (0, 0).
    """
    col = int((x_mundo + GRID_OFFSET_X) / GRID_RESOLUCION)
    fila = int((y_mundo + GRID_OFFSET_Y) / GRID_RESOLUCION)
    # Limitamos a los bordes de la grilla
    col = max(0, min(GRID_ANCHO - 1, col))
    fila = max(0, min(GRID_ALTO - 1, fila))
    return fila, col


def bresenham(fila0, col0, fila1, col1):
    """
    Función que genera la lista de celdas (fila, col) sobre la línea recta entre
    (fila0, col0) y (fila1, col1) usando el algoritmo de Bresenham.

    El algoritmo de Bresenham traza eficientemente una línea discreta en la grilla,
    permitiendo marcar todas las celdas que el rayo del sensor atraviesa entre el robot y el punto de obstáculo detectado.
    """
    celdas = []
    df = abs(fila1 - fila0)
    dc = abs(col1 - col0)
    sf = 1 if fila1 > fila0 else -1
    sc = 1 if col1 > col0 else -1
    err = df - dc

    f, c = fila0, col0

    while True:
        celdas.append((f, c))
        if f == fila1 and c == col1:
            break
        e2 = 2 * err
        if e2 > -dc:
            err -= dc
            f += sf
        if e2 < df:
            err += df
            c += sc

    return celdas


def actualizar_grilla(x_robot, y_robot, x_obs, y_obs):
    """
    Función que actualiza la grilla de ocupación a partir de una detección de obstáculo.

    Dado el punto del robot (x_robot, y_robot) y un punto de obstáculo
    detectado (x_obs, y_obs):
      1. Traza una línea con Bresenham del robot al obstáculo.
      2. Marca las celdas intermedias como LIBRES (log-odds negativo).
      3. Marca la celda final como OCUPADA (log-odds positivo).

    Usa el modelo log-odds para actualizar probabilísticamente cada celda, con límites para evitar saturación excesiva.
    """
    fila_r, col_r = mundo_a_grilla(x_robot, y_robot)
    fila_o, col_o = mundo_a_grilla(x_obs, y_obs)

    celdas_linea = bresenham(fila_r, col_r, fila_o, col_o)

    # Todas las celdas intermedias son libres, excepto la última que es ocupada
    for fila, col in celdas_linea[:-1]:
        grilla[fila][col] = max(LOG_ODD_MIN,
                                grilla[fila][col] + LOG_ODD_LIBRE)

    # La última celda es donde está el obstáculo
    if celdas_linea:
        fila_f, col_f = celdas_linea[-1]
        grilla[fila_f][col_f] = min(LOG_ODD_MAX,
                                     grilla[fila_f][col_f] + LOG_ODD_OCUPADO)


def guardar_mapa(nombre_archivo=None):
    """
    Función que guarda la grilla de ocupación como imagen PPM (Portable PixMap).
    No requiere matplotlib ni ninguna librería externa.

    Escala de colores:
      - Negro  = ocupado (log-odds alto, probabilidad alta)
      - Blanco  = libre  (log-odds bajo, probabilidad baja)
      - Gris   = desconocido (log-odds ≈ 0, probabilidad ≈ 0.5)
    """
    if nombre_archivo is None:
        nombre_archivo = RUTA_MAPA

    with open(nombre_archivo, 'w') as f:
        f.write(f"P3\n{GRID_ANCHO} {GRID_ALTO}\n255\n")
        # Invertimos las filas para que Y crezca hacia arriba en la imagen
        for fila in reversed(range(GRID_ALTO)):
            for col in range(GRID_ANCHO):
                # Convertir log-odds a probabilidad de ocupación [0, 1]
                log_odd = grilla[fila][col]
                prob_ocupado = 1.0 / (1.0 + math.exp(-log_odd))
                # Invertir: prob alta (ocupado) → oscuro, prob baja (libre) → claro
                valor = int((1.0 - prob_ocupado) * 255)
                valor = max(0, min(255, valor))
                f.write(f"{valor} {valor} {valor} ")
            f.write("\n")


# ============================================================================
# Rutina de prueba - Fase 3: Navegación reactiva para mapeo
# ============================================================================
# El robot navega de forma reactiva por el entorno: avanza mientras no detecte obstáculos cercanos, y al detectar uno gira para cambiar de dirección.
# Mientras tanto, la grilla de ocupación se actualiza con cada lectura de los sensores y se guarda periódicamente como imagen.

VELOCIDAD_AVANCE = 2.0       # velocidad al avanzar recto [rad/s]
VELOCIDAD_GIRO = 1.5         # velocidad de giro al esquivar [rad/s]

# Umbrales de los sensores para la navegación reactiva
# Recordar que los sensores del e-puck en Webots devuelven valores bajos (~60-150) incluso a distancias cortas. Umbrales bajos = reacción más temprana.

UMBRAL_PARED = 80            # valor del sensor para considerar pared cercana
UMBRAL_PARED_MUY_CERCA = 150   # valor para pared muy cerca (giro fuerte)

tiempo_simulacion = 0.0
paso_seg = timestep / 1000.0   # convertir timestep de ms a segundos

print("=" * 60)
print("  FASE 3 - Mapeo con Grilla de Ocupación")
print("=" * 60)
print(f"  Grilla:       {GRID_ANCHO}x{GRID_ALTO} celdas")
print(f"  Resolución:   {GRID_RESOLUCION*100:.0f} cm/celda")
print(f"  Arena:        {ARENA_ANCHO}x{ARENA_ALTO} m")
print(f"  Modelo:       Log-odds (occ={LOG_ODD_OCUPADO}, "
      f"libre={LOG_ODD_LIBRE})")
print(f"  Mapa guardado en: {RUTA_MAPA}")
print("=" * 60)
print("  El robot explorará el entorno reactivamente...")
print("=" * 60)

# ============================================================================
# Bucle principal de simulación
# ============================================================================
while robot.step(timestep) != -1:
    # Lectura de encoders
    enc_izq = encoder_izq.getValue()
    enc_der = encoder_der.getValue()

    # En el primer paso, inicializamos los valores previos
    if primera_lectura:
        prev_encoder_izq = enc_izq
        prev_encoder_der = enc_der
        primera_lectura = False

    # Cálculo de odometría (Fase 1)
    actualizar_odometria(enc_izq, enc_der)

    # Lectura de sensores y obtención de puntos de obstáculos (Fase 2)
    puntos_detectados = obtener_puntos_obstaculos()

    # Actualización de la grilla de ocupación (Fase 3)
    for (px, py) in puntos_detectados:
        actualizar_grilla(x, y, px, py)

    # Navegación reactiva (Fase 3)
    # Leemos los sensores frontales y laterales para decidir el movimiento
    valores_sensores = [sensores_ir[i].getValue() for i in range(8)]

    # Sensores frontales: ps0 (frontal derecho) y ps7 (frontal izquierdo)
    # Sensores diagonales: ps1 (diagonal derecho) y ps6 (diagonal izquierdo)
    frontal_der = valores_sensores[0]
    diagonal_der = valores_sensores[1]
    frontal_izq = valores_sensores[7]
    diagonal_izq = valores_sensores[6]

    # Lógica de navegación reactiva
    obstaculo_frente = (frontal_der > UMBRAL_PARED or
                        frontal_izq > UMBRAL_PARED)
    obstaculo_derecha = diagonal_der > UMBRAL_PARED
    obstaculo_izquierda = diagonal_izq > UMBRAL_PARED

    if frontal_der > UMBRAL_PARED_MUY_CERCA or frontal_izq > UMBRAL_PARED_MUY_CERCA:
        # Muy cerca de pared frontal: girar fuerte a la izquierda
        set_velocidades(-VELOCIDAD_GIRO, VELOCIDAD_GIRO)
        fase_actual = "GIRO FUERTE IZQ"
    elif obstaculo_frente and obstaculo_derecha:
        # Obstáculo al frente y a la derecha: girar a la izquierda
        set_velocidades(-VELOCIDAD_GIRO * 0.5, VELOCIDAD_GIRO)
        fase_actual = "GIRO IZQUIERDA"
    elif obstaculo_frente and obstaculo_izquierda:
        # Obstáculo al frente y a la izquierda: girar a la derecha
        set_velocidades(VELOCIDAD_GIRO, -VELOCIDAD_GIRO * 0.5)
        fase_actual = "GIRO DERECHA"
    elif obstaculo_frente:
        # Obstáculo solo al frente: girar a la izquierda por defecto
        set_velocidades(-VELOCIDAD_GIRO, VELOCIDAD_GIRO)
        fase_actual = "GIRO (FRENTE)"
    elif obstaculo_derecha:
        # Obstáculo a la derecha: curva suave a la izquierda
        set_velocidades(VELOCIDAD_AVANCE * 0.5, VELOCIDAD_AVANCE)
        fase_actual = "CURVA IZQ"
    elif obstaculo_izquierda:
        # Obstáculo a la izquierda: curva suave a la derecha
        set_velocidades(VELOCIDAD_AVANCE, VELOCIDAD_AVANCE * 0.5)
        fase_actual = "CURVA DER"
    else:
        # Camino libre: avanzar recto
        set_velocidades(VELOCIDAD_AVANCE, VELOCIDAD_AVANCE)
        fase_actual = "AVANZANDO"

    # Guardamos el mapa periódicamente
    if int(tiempo_simulacion * 1000) % int(MAPA_INTERVALO_GUARDADO * 1000) < timestep:
        guardar_mapa()
        print(f"  [MAPA] Grilla guardada → {os.path.basename(RUTA_MAPA)}")

    # Imprimimos el estado cada ~1 segundo
    if int(tiempo_simulacion * 1000) % 1000 < timestep:
        theta_deg = math.degrees(theta)
        print(
            f"[{tiempo_simulacion:6.1f}s] {fase_actual:18s} | "
            f"X={x:+7.4f} m  Y={y:+7.4f} m  θ={theta_deg:+7.1f}° | "
            f"Det: {len(puntos_detectados)}"
        )
        # Debug: imprimir valores crudos de sensores frontales y diagonales
        print(
            f"         Sensores → ps0={valores_sensores[0]:6.1f}  "
            f"ps1={valores_sensores[1]:6.1f}  "
            f"ps6={valores_sensores[6]:6.1f}  "
            f"ps7={valores_sensores[7]:6.1f}"
        )

    tiempo_simulacion += paso_seg
