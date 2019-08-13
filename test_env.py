import pybullet as p
import numpy as np

from robotEnv import RobotEnv

def main():
    robotEnv = RobotEnv()
    print(RobotEnv.action_space)
    for i in range(100):
        robotEnv.reset()
        while True:
            random_action = np.random.random(size=4)
            robotEnv.step(random_action)
            obs = robotEnv.robot.get_observation()
            print(robotEnv.reward())
            if robotEnv.terminated == 1:
                print('========================================')
                print('terminated')
                print('========================================')
                robotEnv.box_pos = [0, 0, 0]
                robotEnv.angle = 0.
                break
            p.stepSimulation()


if __name__ == "__main__":
    main()
