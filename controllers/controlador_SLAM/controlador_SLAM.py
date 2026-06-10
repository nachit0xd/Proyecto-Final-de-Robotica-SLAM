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
"""

import math
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
# Rutina de prueba
# ============================================================================
# El robot avanzará lentamente en línea recta hacia la pared del arena y a medida que se acerque, los sensores frontales comenzarán a detectar
# el obstáculo y se imprimirán las coordenadas globales de los puntos detectados para validar la transformación de coordenadas.

VELOCIDAD_PRUEBA = 1.5   # velocidad lenta para acercarse a la pared [rad/s]

tiempo_simulacion = 0.0
paso_seg = timestep / 1000.0   # convertir timestep de ms a segundos

print("=" * 60)
print("  FASE 2 - Prueba de Percepción y Transformación")
print("=" * 60)
print(f"  Sensores IR:          {len(sensores_ir)} (ps0–ps7)")
print(f"  Filtro:               Media móvil (ventana={FILTRO_VENTANA})")
print(f"  Alcance efectivo:     {SENSOR_MAX_RANGE*100:.1f} cm")
print(f"  Umbral de detección:  {SENSOR_UMBRAL_DETECCION}")
print("=" * 60)
print("  El robot avanzará hacia la pared del arena...")
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

    # Calculo de odometría 
    actualizar_odometria(enc_izq, enc_der)

    # Leemos los sensores y obtenemos los puntos de obstáculos
    puntos_detectados = obtener_puntos_obstaculos()

    # Rutina de movimiento de prueba:
    # Avanzar recto; si un sensor frontal detecta algo muy cerca, detenerse
    sensor_frontal_izq = sensores_ir[7].getValue()
    sensor_frontal_der = sensores_ir[0].getValue()
    obstaculo_cerca = sensor_frontal_izq > 1000 or sensor_frontal_der > 1000

    if obstaculo_cerca:
        set_velocidades(0.0, 0.0)
        fase_actual = "DETENIDO (PARED)"
    else:
        set_velocidades(VELOCIDAD_PRUEBA, VELOCIDAD_PRUEBA)
        fase_actual = "AVANZANDO"

    # Imprime el estado cada ~500 ms
    if int(tiempo_simulacion * 1000) % 500 < timestep:
        theta_deg = math.degrees(theta)
        print(
            f"[{tiempo_simulacion:6.2f}s] {fase_actual:18s} | "
            f"X={x:+7.4f} m  Y={y:+7.4f} m  θ={theta_deg:+8.2f}°"
        )
        print(
            f"         Detecciones: {len(puntos_detectados)} puntos"
        )
        for px, py in puntos_detectados:
            print(f"           → ({px:+.4f}, {py:+.4f})")

    tiempo_simulacion += paso_seg
