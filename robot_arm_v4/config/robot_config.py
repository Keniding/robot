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

# config/robot_config.py
JOINT_SPEED = {
    'POSITION': 0.02,  # radianes por paso (reducido para más control)
    'VELOCITY': 0.5    # radianes por segundo
}

CONTROL_MODES = {
    'home': [0, 0, 0, 0, 0, 0],
    'grab': [0, -0.5, 1.0, -0.5, 0, 1.0],
    'park': [0, -1.57, 0, 0, 0, 0]
}

# Configuración para el procesamiento de señales BCI
BCI_CONFIG = {
    'SAMPLE_RATE': 250,  # Hz
    'WINDOW_SIZE': 1000, # muestras
    'CHANNELS': ['C3', 'C4', 'Cz'],  # Canales EEG estándar
    'FREQUENCY_BANDS': {
        'alpha': (8, 13),
        'beta': (13, 30),
        'gamma': (30, 50)
    }
}
