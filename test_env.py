import pybullet as p
import numpy as np

from robotEnv import RobotEnv

def main():
    robotEnv = RobotEnv()
    print(RobotEnv.action_space)
    for i in range(100):
        robotEnv.reset()
        run_steps(robotEnv)

def run_steps(robot_env):
    while True:
        random_action = np.random.random(size=robot_env.robot.n_joints - 1)
        robot_env.step(random_action)
        obs = robot_env.robot.get_observation()
        if robot_env.terminated == 1:
            break
        p.stepSimulation()


if __name__ == "__main__":
    main()
