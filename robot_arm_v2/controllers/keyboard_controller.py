import pybullet as p
import numpy as np
from config.robot_config import CONTROL_MODES, JOINT_SPEED

class KeyboardController:
    def __init__(self, robot_id):
        self.robot_id = robot_id
        self.active_joint = 0  # Articulación actualmente seleccionada
        self.num_joints = p.getNumJoints(robot_id)
        self.control_mode = "POSITION"  # POSITION o VELOCITY
        self.initialize_keyboard_controls()

    def initialize_keyboard_controls(self):
        """Define los controles del teclado"""
        self.controls = {
            ord('1'): lambda: self.set_active_joint(0),  # Base
            ord('2'): lambda: self.set_active_joint(1),  # Hombro
            ord('3'): lambda: self.set_active_joint(2),  # Codo
            ord('4'): lambda: self.set_active_joint(3),  # Muñeca 1
            ord('5'): lambda: self.set_active_joint(4),  # Muñeca 2
            ord('6'): lambda: self.set_active_joint(5),  # Muñeca 3
            ord('m'): self.toggle_control_mode,          # Cambiar modo
            ord('r'): self.reset_position,              # Reset
            p.B3G_LEFT_ARROW: lambda: self.move_joint(-1),
            p.B3G_RIGHT_ARROW: lambda: self.move_joint(1),
            # Teclas para movimientos predefinidos
            ord('h'): lambda: self.move_to_preset("home"),
            ord('g'): lambda: self.move_to_preset("grab"),
            ord('p'): lambda: self.move_to_preset("park")
        }

    def set_active_joint(self, joint_idx):
        """Cambia la articulación activa"""
        if 0 <= joint_idx < self.num_joints:
            self.active_joint = joint_idx
            print(f"Articulación activa: {joint_idx}")

    def toggle_control_mode(self):
        """Alterna entre control de posición y velocidad"""
        self.control_mode = "VELOCITY" if self.control_mode == "POSITION" else "POSITION"
        print(f"Modo de control: {self.control_mode}")

    def move_joint(self, direction):
        """Mueve la articulación activa en la dirección especificada"""
        if self.control_mode == "POSITION":
            current_pos = p.getJointState(self.robot_id, self.active_joint)[0]
            target_pos = current_pos + direction * JOINT_SPEED['POSITION']
            p.setJointMotorControl2(
                self.robot_id,
                self.active_joint,
                p.POSITION_CONTROL,
                targetPosition=target_pos
            )
        else:
            p.setJointMotorControl2(
                self.robot_id,
                self.active_joint,
                p.VELOCITY_CONTROL,
                targetVelocity=direction * JOINT_SPEED['VELOCITY']
            )

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

    def process_keyboard_events(self):
        """Procesa los eventos del teclado"""
        keys = p.getKeyboardEvents()
        for key, state in keys.items():
            if state & p.KEY_WAS_TRIGGERED:
                if key in self.controls:
                    self.controls[key]()
