# controllers/hand_controller.py
import pybullet as p

class HandController:
    def __init__(self, robot_id):
        self.robot_id = robot_id
        # Mapeo de articulaciones de la mano
        self.hand_joints = {
            'thumb': {
                'base': 'thumb_base',         # Nombre de la articulación en el URDF
                'phalange': 'thumb_joint_2'
            },
            'index': {
                'base': 'finger_1_base',
                'phalange': 'finger_1_joint_2'
            },
            'middle': {
                'base': 'finger_2_base',
                'phalange': 'finger_2_joint_2'
            },
            'ring': {
                'base': 'finger_3_base',
                'phalange': 'finger_3_joint_2'
            }
        }
        
        # Obtener los IDs de las articulaciones
        self.joint_name_to_id = {}
        for i in range(p.getNumJoints(robot_id)):
            joint_info = p.getJointInfo(robot_id, i)
            self.joint_name_to_id[joint_info[1].decode('utf-8')] = i

    def move_finger(self, finger_name, close_amount):
        """
        Mueve un dedo específico
        :param finger_name: 'thumb', 'index', 'middle', 'ring'
        :param close_amount: 0 (abierto) a 1 (cerrado)
        """
        if finger_name in self.hand_joints:
            # Mover la base del dedo
            base_joint = self.hand_joints[finger_name]['base']
            phalange_joint = self.hand_joints[finger_name]['phalange']
            
            base_id = self.joint_name_to_id[base_joint]
            phalange_id = self.joint_name_to_id[phalange_joint]
            
            # Ajustar los límites según el dedo
            if finger_name == 'thumb':
                base_target = -1.57 * close_amount
                phalange_target = -1.57 * close_amount
            else:
                base_target = -1.57 * close_amount
                phalange_target = 1.57 * close_amount
            
            p.setJointMotorControl2(
                self.robot_id,
                base_id,
                p.POSITION_CONTROL,
                targetPosition=base_target,
                force=10
            )
            p.setJointMotorControl2(
                self.robot_id,
                phalange_id,
                p.POSITION_CONTROL,
                targetPosition=phalange_target,
                force=10
            )

    def close_hand(self):
        """Cierra todos los dedos"""
        for finger in self.hand_joints.keys():
            self.move_finger(finger, 1.0)

    def open_hand(self):
        """Abre todos los dedos"""
        for finger in self.hand_joints.keys():
            self.move_finger(finger, 0.0)

    def set_finger_position(self, finger_name, position):
        """
        Establece la posición de un dedo específico
        :param finger_name: nombre del dedo ('thumb', 'index', 'middle', 'ring')
        :param position: posición normalizada (0 = abierto, 1 = cerrado)
        """
        if finger_name in self.hand_joints:
            self.move_finger(finger_name, position)
