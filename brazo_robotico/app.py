import pybullet as p
import time
import pybullet_data
import math

# Inicializar PyBullet
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

# Cargar el plano y el robot
planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("robot_arm.urdf", [0, 0, 0], useFixedBase=True)

# Definir los IDs de las articulaciones
WAIST = 0
SHOULDER = 1
ELBOW = 2
GRIPPER = 3

# Función para mover una articulación a un ángulo específico
def set_joint_angle(joint_id, angle):
    p.setJointMotorControl2(robotId, 
                           joint_id, 
                           p.POSITION_CONTROL,
                           targetPosition=angle,
                           force=500)

# Función para mover el brazo a una posición específica
def move_arm(waist_angle, shoulder_angle, elbow_angle, gripper_angle):
    set_joint_angle(WAIST, waist_angle)
    set_joint_angle(SHOULDER, shoulder_angle)
    set_joint_angle(ELBOW, elbow_angle)
    set_joint_angle(GRIPPER, gripper_angle)

# Configurar la interfaz de depuración
p.addUserDebugParameter("Waist", -3.14, 3.14, 0)
p.addUserDebugParameter("Shoulder", -1.57, 1.57, 0)
p.addUserDebugParameter("Elbow", -1.57, 1.57, 0)
p.addUserDebugParameter("Gripper", -1.57, 1.57, 0)

# Bucle principal
try:
    while True:
        # Leer los valores de los sliders
        waist = p.readUserDebugParameter(0)
        shoulder = p.readUserDebugParameter(1)
        elbow = p.readUserDebugParameter(2)
        gripper = p.readUserDebugParameter(3)
        
        # Mover el brazo
        move_arm(waist, shoulder, elbow, gripper)
        
        # Actualizar la simulación
        p.stepSimulation()
        time.sleep(1./240.)

except KeyboardInterrupt:
    p.disconnect()
