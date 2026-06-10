"""
controlador_SLAM.py

Línea B: SLAM / Mapeo Autónomo Simplificado

Este controlador implementa:
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
  Fase 4:
    11. Máquina de estados (AVANZAR → REBOTAR).
    12. Exploración reactiva por deambulación y rebotes en el sitio (Roomba-style).
    13. Pesos Braitenberg para esquive suave y proporcional a media distancia.
    14. Métrica de cobertura del mapa explorado.
  Fase 5:
    15. Registro de métricas de odometría en tiempo real (distancia recorrida y velocidad promedio).
    16. Evaluación cuantitativa y comparativa entre escenarios de prueba.
  Fase 6:
    17. Integración de sensor LiDAR de 360 grados y largo rango (1.2m).
    18. Modo dual configurable de mapeo (LiDAR vs Infrarrojo).
    19. Mapeo de alta resolución optimizado por submuestreo de barrido.
"""

import math
import os
import random
from controller import Robot

# Configuración del Modo de Mapeo
USAR_LIDAR = True              # True: mapea usando LiDAR de 360°. False: mapea usando los 8 sensores IR

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

# Inicialización de LiDAR
lidar = None
if USAR_LIDAR:
    lidar = robot.getDevice('lidar')
    if lidar is not None:
        lidar.enable(timestep)
    else:
        print("[ADVERTENCIA] El dispositivo 'lidar' no fue encontrado en el robot. Usando sensores IR.")
        USAR_LIDAR = False

# ============================================================================
# Variables de odometría
# ============================================================================
# Posición y orientación estimadas del robot en el marco global
x = 0.0       # posición X [m]
y = 0.0       # posición Y [m]
theta = 0.0   # orientación [rad]
distancia_recorrida = 0.0     # distancia lineal acumulada recorrida [m]

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
    global x, y, theta, prev_encoder_izq, prev_encoder_der, distancia_recorrida

    # Diferencia angular de cada rueda desde el último paso
    delta_enc_izq = enc_izq_actual - prev_encoder_izq
    delta_enc_der = enc_der_actual - prev_encoder_der

    # Distancia lineal recorrida por cada rueda
    delta_s_izq = WHEEL_RADIUS * delta_enc_izq
    delta_s_der = WHEEL_RADIUS * delta_enc_der

    # Avance lineal del centro del robot y cambio de orientación
    delta_s = (delta_s_der + delta_s_izq) / 2.0
    delta_theta = (delta_s_der - delta_s_izq) / AXLE_LENGTH

    # Acumulamos la distancia lineal recorrida
    distancia_recorrida += abs(delta_s)

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
# Funciones auxiliares de Navegación 
# ============================================================================

# Estados de la máquina de estados por Rebote Reactivo
ESTADO_AVANZAR = 0          # avanzar recto y esquive suave Braitenberg
ESTADO_REBOTAR = 1          # girar en el lugar para cambiar de rumbo

NOMBRES_ESTADO = {
    ESTADO_AVANZAR: "AVANZAR",
    ESTADO_REBOTAR: "REBOTAR",
}

# Parámetros de navegación
VELOCIDAD_AVANCE = 2.5      # velocidad base [rad/s]
VELOCIDAD_GIRO = 2.0        # velocidad de giro en el lugar [rad/s]
UMBRAL_DETECCION = 100      # valor de sensor para iniciar esquive suave
UMBRAL_CRITICO = 350        # valor de sensor para activar rebote (giro en sitio)

# Pesos Braitenberg para esquive suave (Fase 4): cada sensor contribuye con un peso a cada motor.
# Sensores del lado derecho (ps0, ps1, ps2) hacen girar a la izquierda.
# Sensores del lado izquierdo (ps5, ps6, ps7) hacen girar a la derecha.
#             ps0   ps1   ps2   ps3   ps4   ps5   ps6   ps7
PESOS_IZQ = [-1.0, -0.5, -0.2,  0.0,  0.0,  0.2,  0.5,  1.0]
PESOS_DER = [ 1.0,  0.5,  0.2,  0.0,  0.0, -0.2, -0.5, -1.0]
FACTOR_BRAITENBERG = 0.01       # factor de escala para las influencias


def calcular_braitenberg(valores_sensores):
    """
    Función que calcula las velocidades de los motores usando pesos Braitenberg.

    Cada sensor contribuye proporcionalmente a cada motor según su peso.
    Esto produce un giro suave y continuo que es proporcional a la
    cercanía de los obstáculos, en vez de giros bruscos tipo on/off.
    """
    influencia_izq = sum(v * p for v, p in zip(valores_sensores, PESOS_IZQ))
    influencia_der = sum(v * p for v, p in zip(valores_sensores, PESOS_DER))
    vel_izq = VELOCIDAD_AVANCE + influencia_izq * FACTOR_BRAITENBERG
    vel_der = VELOCIDAD_AVANCE + influencia_der * FACTOR_BRAITENBERG
    return vel_izq, vel_der


def calcular_cobertura():
    """
    Función que calcula el porcentaje de celdas de la grilla que han sido exploradas.
    Una celda se considera explorada si su valor log-odds se ha movido significativamente del valor inicial (0), es decir, si el robot
    ya pasó por ahí y detectó que era libre u ocupada.
    """
    exploradas = 0
    total = GRID_ANCHO * GRID_ALTO
    for fila in range(GRID_ALTO):
        for col in range(GRID_ANCHO):
            if abs(grilla[fila][col]) > 0.1:
                exploradas += 1
    return (exploradas / total) * 100.0


# ============================================================================
# Variables de estado de la máquina de estados
# ============================================================================
estado_actual = ESTADO_AVANZAR
tiempo_giro_restante = 0.0
direccion_giro = 1.0                # 1.0 para izquierda, -1.0 para derecha

tiempo_simulacion = 0.0
paso_seg = timestep / 1000.0        # convertir timestep de ms a segundos

print("=" * 60)
print("  FASE 4 - Exploración Autónoma Inteligente (Rebote Reactivo)")
print("=" * 60)
print(f"  Grilla:       {GRID_ANCHO}x{GRID_ALTO} celdas "
      f"({GRID_RESOLUCION*100:.0f} cm/celda)")
print(f"  Estrategia:   Rebote Reactivo + Braitenberg (Roomba-style)")
print(f"  Mapa:         {RUTA_MAPA}")
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

    # Percepción y Mapeo
    puntos_detectados = []
    if USAR_LIDAR:
        # Obtenemos el barrido del LiDAR (lista de 360 distancias en metros)
        lecturas_lidar = lidar.getRangeImage()
        
        # Submuestreo (1 de cada 5 grados para trazar 72 rayos y no sobrecargar la grilla)
        paso_angular = (2 * math.pi) / 360.0
        for i in range(0, 360, 5):
            distancia = lecturas_lidar[i]
            
            # Filtramos las lecturas fuera de rango o infinitas/NaN
            if 0.02 < distancia < 1.2 and not math.isinf(distancia) and not math.isnan(distancia):
                # El ángulo del rayo en el marco global es la orientación del robot más el ángulo local del rayo.
                # Webots LiDAR convención: el ángulo del rayo i es (i * paso_angular) relativo al frente y en sentido antihorario.
                angulo_global = theta + (i * paso_angular)
                
                x_obs = x + distancia * math.cos(angulo_global)
                y_obs = y + distancia * math.sin(angulo_global)
                puntos_detectados.append((x_obs, y_obs))
    else:
        # Modo normal: usar los 8 sensores infrarrojos
        puntos_detectados = obtener_puntos_obstaculos()

    # Actualización de la grilla de ocupación (Fase 3)
    for (px, py) in puntos_detectados:
        actualizar_grilla(x, y, px, py)

    # Lectura de valores crudos de sensores para navegación
    valores_sensores = [sensores_ir[i].getValue() for i in range(8)]

    # Sensores clave para detección frontal/diagonal:
    frontal_der = valores_sensores[0]
    diagonal_der = valores_sensores[1]
    diagonal_izq = valores_sensores[6]
    frontal_izq = valores_sensores[7]
    
    # Máximo valor detectado en el arco frontal
    max_frente = max(frontal_der, frontal_izq, diagonal_der, diagonal_izq)

    # ===================================================================
    # MÁQUINA DE ESTADOS (Wander y Rebote Reactivo)
    # ===================================================================

    if estado_actual == ESTADO_AVANZAR:
        if max_frente > UMBRAL_CRITICO:
            # Obstáculo muy cerca: iniciar Rebote (giro en el sitio)
            estado_actual = ESTADO_REBOTAR
            
            # Decidir dirección del giro: girar hacia el lado con menos obstáculo
            peso_derecho = frontal_der + diagonal_der
            peso_izquierdo = frontal_izq + diagonal_izq
            if peso_derecho > peso_izquierdo:
                direccion_giro = 1.0   # girar a la izquierda (antihorario)
            else:
                direccion_giro = -1.0  # girar a la derecha (horario)
            
            # Duración aleatoria para romper ciclos, entre 0.6 y 1.2 segundos
            tiempo_giro_restante = random.uniform(0.6, 1.2)
        else:
            # Avanzar: si hay obstáculos a media distancia, aplicar Braitenberg para esquive suave
            if max_frente > UMBRAL_DETECCION:
                vel_izq, vel_der = calcular_braitenberg(valores_sensores)
            else:
                vel_izq, vel_der = VELOCIDAD_AVANCE, VELOCIDAD_AVANCE
            set_velocidades(vel_izq, vel_der)

    elif estado_actual == ESTADO_REBOTAR:
        # Girar en el sitio
        set_velocidades(-direccion_giro * VELOCIDAD_GIRO, direccion_giro * VELOCIDAD_GIRO)
        tiempo_giro_restante -= paso_seg
        
        if tiempo_giro_restante <= 0.0:
            # Cuando termina el giro, volver a avanzar
            estado_actual = ESTADO_AVANZAR

    # Guardamos el mapa periódicamente
    if int(tiempo_simulacion * 1000) % int(MAPA_INTERVALO_GUARDADO * 1000) < timestep:
        cobertura = calcular_cobertura()
        guardar_mapa()
        vel_promedio = distancia_recorrida / tiempo_simulacion if tiempo_simulacion > 0 else 0.0
        print(f"  [MAPA] Guardado | Cobertura: {cobertura:.1f}%")
        print(f"  [MÉTRICAS] Tiempo: {tiempo_simulacion:.1f}s | Distancia: {distancia_recorrida:.2f}m | Vel Promedio: {vel_promedio:.2f} m/s")

    # Imprimimos el estado cada ~2 segundos para no saturar la consola
    if int(tiempo_simulacion * 1000) % 2000 < timestep:
        theta_deg = math.degrees(theta)
        nombre_estado = NOMBRES_ESTADO.get(estado_actual, "?")
        print(
            f"[{tiempo_simulacion:6.1f}s] {nombre_estado:14s} | "
            f"X={x:+7.4f}  Y={y:+7.4f}  θ={theta_deg:+7.1f}° | "
            f"Det: {len(puntos_detectados)}"
        )

    tiempo_simulacion += paso_seg
