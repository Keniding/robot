import pybullet as p

class JointController:
    def __init__(self, robot_id):
        self.robot_id = robot_id
        self.num_joints = p.getNumJoints(robot_id)
        self.joint_indices = range(self.num_joints)

    def set_joint_position(self, joint_id, position):
        """Controla la posición de una articulación específica"""
        p.setJointMotorControl2(
            self.robot_id,
            joint_id,
            p.POSITION_CONTROL,
            targetPosition=position
        )

    def get_joint_states(self):
        """Obtiene el estado actual de todas las articulaciones"""
        return [p.getJointState(self.robot_id, joint)[0] 
                for joint in self.joint_indices]

    def move_to_position(self, positions, steps=100):
        """Mueve el brazo a una posición específica de forma suave"""
        current_positions = self.get_joint_states()
        for step in range(steps):
            for joint, (start, end) in enumerate(zip(current_positions, positions)):
                position = start + (end - start) * (step + 1) / steps
                self.set_joint_position(joint, position)
            p.stepSimulation()
