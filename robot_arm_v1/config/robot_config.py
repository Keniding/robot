# Configuración de la simulación
SIM_CONFIG = {
    'TIME_STEP': 1./240.,
    'GRAVITY': -9.81
}

# Configuración de la cámara
CAMERA_CONFIG = {
    'DISTANCE': 1.5,
    'YAW': 45,
    'PITCH': -30,
    'TARGET_POS': [0, 0, 0.5]
}

# Configuración del robot
ROBOT_CONFIG = {
    'START_POS': [0, 0, 0],
    'START_ORI': [0, 0, 0, 1],
    'FIXED_BASE': True
}

# Posiciones predefinidas
POSITIONS = {
    'HOME': [0, 0, 0, 0, 0, 0],
    'READY': [0, -0.5, 1.0, -0.5, 0, 0]
}
