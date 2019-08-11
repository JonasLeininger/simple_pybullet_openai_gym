import pybullet as p
import numpy as np

from robotEnv import RobotEnv

def main():
    robotEnv = RobotEnv()
    print(RobotEnv.action_space)
    for i in range(10000):
        random_action = np.random.random(size=robotEnv.robot.n_joints - 1)
        robotEnv.step(random_action)
        obs = robotEnv.robot.get_observation()
        if robotEnv.terminated == 1:
            break
        p.stepSimulation()


if __name__ == "__main__":
    main()
