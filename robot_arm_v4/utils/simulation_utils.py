import pybullet as p
import pybullet_data
from config.robot_config import SIM_CONFIG, CAMERA_CONFIG

def init_simulation():
    """Inicializa el entorno de simulación"""
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, SIM_CONFIG['GRAVITY'])
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
    set_camera()
    return physicsClient

def set_camera():
    """Configura la cámara de la simulación"""
    p.resetDebugVisualizerCamera(
        cameraDistance=CAMERA_CONFIG['DISTANCE'],
        cameraYaw=CAMERA_CONFIG['YAW'],
        cameraPitch=CAMERA_CONFIG['PITCH'],
        cameraTargetPosition=CAMERA_CONFIG['TARGET_POS']
    )

def load_ground():
    """Carga el plano base"""
    return p.loadURDF("plane.urdf")
