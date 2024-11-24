import pybullet as p
import numpy as np
from config.robot_config import CONTROL_MODES, JOINT_SPEED
import time

class KeyboardController:
    def __init__(self, robot_id):
        self.robot_id = robot_id
        self.active_joint = 0
        self.num_joints = p.getNumJoints(robot_id)
        self.control_mode = "POSITION"
        self.last_movement_time = time.time()
        self.movement_interval = 0.01  # Intervalo entre movimientos en segundos
        self.initialize_keyboard_controls()

    def initialize_keyboard_controls(self):
        """Define los controles del teclado"""
        self.controls = {
            ord('1'): lambda: self.set_active_joint(0),
            ord('2'): lambda: self.set_active_joint(1),
            ord('3'): lambda: self.set_active_joint(2),
            ord('4'): lambda: self.set_active_joint(3),
            ord('5'): lambda: self.set_active_joint(4),
            ord('6'): lambda: self.set_active_joint(5),
            ord('m'): self.toggle_control_mode,
            ord('r'): self.reset_position,
            ord('h'): lambda: self.move_to_preset("home"),
            ord('g'): lambda: self.move_to_preset("grab"),
            ord('p'): lambda: self.move_to_preset("park")
        }

    def move_joint(self, direction):
        """Mueve la articulación activa en la dirección especificada"""
        current_time = time.time()
        
        # Verificar si ha pasado suficiente tiempo desde el último movimiento
        if current_time - self.last_movement_time < self.movement_interval:
            return

        if self.control_mode == "POSITION":
            current_pos = p.getJointState(self.robot_id, self.active_joint)[0]
            # Obtener los límites de la articulación
            joint_info = p.getJointInfo(self.robot_id, self.active_joint)
            lower_limit = joint_info[8]
            upper_limit = joint_info[9]
            
            # Calcular nueva posición
            target_pos = current_pos + direction * JOINT_SPEED['POSITION']
            
            # Aplicar límites
            if lower_limit < upper_limit:
                target_pos = max(lower_limit, min(upper_limit, target_pos))
            
            p.setJointMotorControl2(
                self.robot_id,
                self.active_joint,
                p.POSITION_CONTROL,
                targetPosition=target_pos,
                maxVelocity=JOINT_SPEED['VELOCITY']
            )
        else:  # Modo VELOCITY
            p.setJointMotorControl2(
                self.robot_id,
                self.active_joint,
                p.VELOCITY_CONTROL,
                targetVelocity=direction * JOINT_SPEED['VELOCITY']
            )
        
        self.last_movement_time = current_time

    def process_keyboard_events(self):
        """Procesa los eventos del teclado"""
        keys = p.getKeyboardEvents()
        
        # Procesar teclas que requieren una sola pulsación
        for key, state in keys.items():
            if state & p.KEY_WAS_TRIGGERED:
                if key in self.controls:
                    self.controls[key]()

        # Procesar teclas de movimiento (flechas) de manera continua
        if p.B3G_LEFT_ARROW in keys and keys[p.B3G_LEFT_ARROW] & p.KEY_IS_DOWN:
            self.move_joint(-1)
        if p.B3G_RIGHT_ARROW in keys and keys[p.B3G_RIGHT_ARROW] & p.KEY_IS_DOWN:
            self.move_joint(1)

    def set_active_joint(self, joint_idx):
        """Cambia la articulación activa"""
        if 0 <= joint_idx < self.num_joints:
            self.active_joint = joint_idx
            joint_info = p.getJointInfo(self.robot_id, joint_idx)
            joint_name = joint_info[1].decode('utf-8')
            current_pos = p.getJointState(self.robot_id, joint_idx)[0]
            print(f"Articulación activa: {joint_idx} ({joint_name}) - Posición actual: {current_pos:.2f}")

    def toggle_control_mode(self):
        """Alterna entre control de posición y velocidad"""
        self.control_mode = "VELOCITY" if self.control_mode == "POSITION" else "POSITION"
        print(f"Modo de control: {self.control_mode}")

    def reset_position(self):
        """Resetea todas las articulaciones a su posición inicial"""
        for joint in range(self.num_joints):
            p.setJointMotorControl2(
                self.robot_id,
                joint,
                p.POSITION_CONTROL,
                targetPosition=0
            )

    def move_to_preset(self, preset_name):
        """Mueve el brazo a una posición predefinida"""
        positions = CONTROL_MODES[preset_name]
        for joint, pos in enumerate(positions):
            p.setJointMotorControl2(
                self.robot_id,
                joint,
                p.POSITION_CONTROL,
                targetPosition=pos
            )
