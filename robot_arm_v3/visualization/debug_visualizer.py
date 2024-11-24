import pybullet as p

class DebugVisualizer:
    def __init__(self, robot_id):
        self.robot_id = robot_id
        self.sliders = []
        self.create_joint_sliders()

    def create_joint_sliders(self):
        """Crea sliders para el control de las articulaciones"""
        num_joints = p.getNumJoints(self.robot_id)
        for joint in range(num_joints):
            joint_info = p.getJointInfo(self.robot_id, joint)
            joint_name = joint_info[1].decode('utf-8')
            joint_lower = joint_info[8]
            joint_upper = joint_info[9]
            slider = p.addUserDebugParameter(
                joint_name, joint_lower, joint_upper, 0)
            self.sliders.append(slider)

    def get_slider_values(self):
        """Lee los valores actuales de los sliders"""
        return [p.readUserDebugParameter(slider) 
                for slider in self.sliders]
