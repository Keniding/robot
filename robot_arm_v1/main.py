import pybullet as p
import time
from config.robot_config import SIM_CONFIG, ROBOT_CONFIG, POSITIONS
from utils.simulation_utils import init_simulation, load_ground
from controllers.joint_controller import JointController
from visualization.debug_visualizer import DebugVisualizer

def main():
    # Inicializar simulación
    physicsClient = init_simulation()
    
    # Cargar el suelo
    ground_id = load_ground()
    
    # Cargar el robot
    robot_id = p.loadURDF(
        "models/modern_arm.urdf",
        ROBOT_CONFIG['START_POS'],
        useFixedBase=ROBOT_CONFIG['FIXED_BASE']
    )
    
    # Inicializar controladores
    joint_controller = JointController(robot_id)
    visualizer = DebugVisualizer(robot_id)
    
    try:
        while True:
            # Leer valores de los sliders y aplicar control
            joint_positions = visualizer.get_slider_values()
            for joint, position in enumerate(joint_positions):
                joint_controller.set_joint_position(joint, position)
            
            # Actualizar simulación
            p.stepSimulation()
            time.sleep(SIM_CONFIG['TIME_STEP'])
            
    except KeyboardInterrupt:
        p.disconnect()

if __name__ == "__main__":
    main()
