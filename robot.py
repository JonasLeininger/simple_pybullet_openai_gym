import pybullet as p
import pybullet_data


class Robot:
    """
    Robot arm with 4 dof.
    """

    def __init__(self, urdf_root_path=pybullet_data.getDataPath()):
        self.max_velocity = 0.4
        self.max_force = 150.
        self.max_force_hip = 200.
        self.kp = 0.1
        self.kd = 10.4
        self.n_joints = 0

        self.urdf_root_path = urdf_root_path
        cube_start_pos = [0, 0, 0]
        cube_start_orientation = p.getQuaternionFromEuler([0, 0, 0])
        self.robot = p.loadURDF("robot.urdf", cube_start_pos, cube_start_orientation)
        self.joint_dict = self.get_joint_names()
        self.reset()

    def reset(self):
        p.resetBasePositionAndOrientation(self.robot, [0.0, 0.0, 0.00],
                                          [0.000000, 0.000000, 0.000000, 1.000000])

        self.jointPositions = [
            0.0, 0.0, 0.0, 0.0, 0.0
        ]
        for joint in range(self.n_joints):
            p.resetJointState(self.robot, joint, self.jointPositions[joint])
            p.setJointMotorControl2(self.robot,
                                    joint,
                                    p.POSITION_CONTROL,
                                    targetPosition=self.jointPositions[joint],
                                    force=self.max_force)

    def get_observation(self):
        observation = []
        torso_state = p.getLinkState(self.robot, 0)
        upper_arm_state = p.getLinkState(self.robot, 1)
        lower_arm_state = p.getLinkState(self.robot, 2)
        hand_state = p.getLinkState(self.robot, 3)
        observation.append(torso_state)
        observation.append(upper_arm_state)
        observation.append(lower_arm_state)
        observation.append(hand_state)
        return observation

    def get_joint_names(self):
        self.n_joints = p.getNumJoints(self.robot)
        print('num joints: ', self.n_joints)
        joint_name_to_id = {}

        for i in range(self.n_joints):
            joint_info = p.getJointInfo(self.robot, i)
            joint_name_to_id[joint_info[1].decode('UTF-8')] = joint_info[0]
        return joint_name_to_id

    def applyAction(self, actions):
        """
        Execude the 'actions', motor commands.
        Each action resembles one joint motor command.
        :param action: List of motor commands
        :return: None
        """
        for joint_index in range(1,self.n_joints):
            action_index = joint_index - 1
            self.motor_control(actions[action_index], self.max_force, joint_index)

    def motor_control(self, action, force, index):
        if index == 1:
            force = self.max_force_hip
        
        p.setJointMotorControl2(bodyIndex=self.robot,
                                jointIndex=index,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=action,
                                positionGain=self.kp,
                                velocityGain=self.kd,
                                force=force)