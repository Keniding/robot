import pybullet as p
import time
import numpy as np
from utils.simulation_utils import init_simulation, load_ground
from controllers.keyboard_controller import KeyboardController
from controllers.bci_controller import BCIController
from config.robot_config import BCI_CONFIG
from controllers.arduino_controller import ArduinoController

def main():
    # Configuración de hardware
    USE_REAL_HARDWARE = False
    is_connected = False
    
    try:
        # Inicializar simulación
        physicsClient = init_simulation()
        is_connected = True
        ground_id = load_ground()
        
        # Cargar el robot
        robot_id = p.loadURDF("models/modern_arm.urdf", [0, 0, 0], useFixedBase=True)
        
        # Inicializar controladores
        keyboard_ctrl = KeyboardController(robot_id)
        bci_ctrl = BCIController(robot_id)
        arduino = ArduinoController(simulate=not USE_REAL_HARDWARE)
        
        # Modo de control actual
        control_mode = "keyboard"
        
        # Obtener información de las articulaciones
        num_joints = p.getNumJoints(robot_id)
        last_joint_states = [0] * num_joints
        
        while True:
            try:
                # Procesar control según el modo
                if control_mode == "keyboard":
                    keyboard_ctrl.process_keyboard_events()
                else:
                    simulated_eeg = np.random.randn(BCI_CONFIG['WINDOW_SIZE'])
                    bci_ctrl.update_robot_control(simulated_eeg)
                
                # Sincronizar con Arduino si está habilitado
                if USE_REAL_HARDWARE:
                    for joint_id in range(num_joints):
                        joint_state = p.getJointState(robot_id, joint_id)
                        current_angle = np.degrees(joint_state[0])
                        
                        if abs(current_angle - last_joint_states[joint_id]) > 1.0:
                            arduino.move_joint(joint_id + 1, int(current_angle))
                            last_joint_states[joint_id] = current_angle
                
                # Cambiar modo de control con la tecla 'b'
                keys = p.getKeyboardEvents()
                if ord('b') in keys and keys[ord('b')] & p.KEY_WAS_TRIGGERED:
                    control_mode = "bci" if control_mode == "keyboard" else "keyboard"
                    print(f"Modo de control cambiado a: {control_mode}")
                
                # Cambiar modo hardware/simulación con la tecla 'h'
                if ord('h') in keys and keys[ord('h')] & p.KEY_WAS_TRIGGERED:
                    USE_REAL_HARDWARE = not USE_REAL_HARDWARE
                    print(f"Modo hardware: {'Activado' if USE_REAL_HARDWARE else 'Desactivado'}")
                    arduino.close()
                    arduino = ArduinoController(simulate=not USE_REAL_HARDWARE)
                
                p.stepSimulation()
                time.sleep(1./240.)
                
            except p.error:
                break
                
    except KeyboardInterrupt:
        print("\nCerrando simulación...")
    finally:
        try:
            arduino.close()
        except:
            pass
            
        try:
            if is_connected:
                p.disconnect()
        except:
            pass
            
        print("Simulación terminada")

if __name__ == "__main__":
    main()
