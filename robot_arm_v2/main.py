# main.py
import pybullet as p
import time
import numpy as np
from utils.simulation_utils import init_simulation, load_ground
from controllers.keyboard_controller import KeyboardController
from controllers.bci_controller import BCIController
from config.robot_config import BCI_CONFIG

def main():
    # Inicializar simulación
    physicsClient = init_simulation()
    ground_id = load_ground()
    
    # Cargar el robot
    robot_id = p.loadURDF("models/modern_arm.urdf", [0, 0, 0], useFixedBase=True)
    
    # Inicializar controladores
    keyboard_ctrl = KeyboardController(robot_id)
    bci_ctrl = BCIController(robot_id)
    
    # Modo de control actual
    control_mode = "keyboard"  # "keyboard" o "bci"
    
    try:
        while True:
            if control_mode == "keyboard":
                keyboard_ctrl.process_keyboard_events()
            else:
                # Simular señal EEG (en una implementación real, esto vendría del hardware)
                simulated_eeg = np.random.randn(BCI_CONFIG['WINDOW_SIZE'])
                bci_ctrl.update_robot_control(simulated_eeg)
            
            # Cambiar modo de control con la tecla 'b'
            keys = p.getKeyboardEvents()
            if ord('b') in keys and keys[ord('b')] & p.KEY_WAS_TRIGGERED:
                control_mode = "bci" if control_mode == "keyboard" else "keyboard"
                print(f"Modo de control cambiado a: {control_mode}")
            
            p.stepSimulation()
            time.sleep(1./240.)
            
    except KeyboardInterrupt:
        p.disconnect()

if __name__ == "__main__":
    main()
