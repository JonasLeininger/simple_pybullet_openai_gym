import pybullet as p
import numpy as np

from robotEnv import RobotEnv

def main():
    robotEnv = RobotEnv()
    print(RobotEnv.action_space)
    for i in range(10000):
        random_action = np.random.random(size=1)
        robotEnv.step(random_action)
        obs = robotEnv.robot.getObservation()
        print(obs)
        p.stepSimulation()


if __name__ == "__main__":
    main()
