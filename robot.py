import pybullet as p
import pybullet_data


class Robot:
    """
    Robot arm with 4 dof.
    """

    def __init__(self, urdf_root_path=pybullet_data.getDataPath()):
        self.max_velocity = 0.4
        self.max_force = 200.
        self.n_joints = 0

        self.urdf_root_path = urdf_root_path
        cube_start_pos = [0, 0, 0]
        cube_start_orientation = p.getQuaternionFromEuler([0, 0, 0])
        self.robot = p.loadURDF("robot.urdf", cube_start_pos, cube_start_orientation)
        self.joint_dict = self.getJointNames()
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

    def getObservation(self):
        observation = []
        torsoState = p.getLinkState(self.robot, 0)
        upper_armState = p.getLinkState(self.robot, 1)
        lower_armState = p.getLinkState(self.robot, 2)
        handState = p.getLinkState(self.robot, 3)
        observation.append(torsoState)
        observation.append(upper_armState)
        observation.append(lower_armState)
        observation.append(handState)
        return observation

    def getJointNames(self):
        self.n_joints = p.getNumJoints(self.robot)
        print('num joints: ', self.n_joints)
        joint_name_to_id = {}

        for i in range(self.n_joints):
            joint_info = p.getJointInfo(self.robot, i)
            joint_name_to_id[joint_info[1].decode('UTF-8')] = joint_info[0]
        return joint_name_to_id
