import gym
from gym import spaces
from gym.utils import seeding
import numpy as np
import pybullet as p
import pybullet_data
import time

from robot import Robot


class RobotEnv(gym.Env):
    """
    Openai Gym Environment for the Kuka UR10 arm with pybullet physics.
    Follows the openai gym environment template.
    """

    def __init__(self, isDiscrete=False, renders=True, maxSteps=1000):
        self._p = p
        self._action_bound = 1
        self._action_dim = 3
        self.terminated = 0
        self._timeSleep = 1. / 240.
        # self._p = p
        self._isDiscrete = isDiscrete
        self._renders = renders
        self._maxtSteps = maxSteps
        self._envStepCounter = 0

        self.set_render(p)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.reset()

        self.seed()
        self.get_action_space()

    def reset(self):
        """
        Reset the environment to initial state.

        :return: None
        """
        self.terminated = 0
        p.resetSimulation()
        self.plane = p.loadURDF("plane.urdf")
        p.setGravity(0, 0, -9.82)
        self.robot = Robot()
        self._envStepCounter = 0
        p.stepSimulation()

    def reward(self):
        return -1.0

    def render(self, mode='human'):
        """Renders the environment.

        The set of supported modes varies per environment. (And some
        environments do not support rendering at all.) By convention,
        if mode is:

        - human: render to the current display or terminal and
          return nothing. Usually for human consumption.
        - rgb_array: Return an numpy.ndarray with shape (x, y, 3),
          representing RGB values for an x-by-y pixel image, suitable
          for turning into a video.
        - ansi: Return a string (str) or StringIO.StringIO containing a
          terminal-style text representation. The text can include newlines
          and ANSI escape sequences (e.g. for colors).

        Note:
            Make sure that your class's metadata 'render.modes' key includes
              the list of supported modes. It's recommended to call super()
              in implementations to use the functionality of this method.

        Args:
            mode (str): the mode to render with
        """
        return None

    def set_render(self, p):
        """
        Set the render moder for the p - bullet engine.

        :param p: bullet enginge
        :return:
        """
        if self._renders:
            # TODO: check different p.connect() options
            # cid = p.connect(p.SHARED_MEMORY)
            cid = p.connect(p.GUI)
            if (cid < 0):
                cid = p.connect(p.GUI)
        else:
            cid = p.connect(p.DIRECT)

    def step(self, action_hip):
        self.robot.applyAction(action_hip)
        self._termination()
        time.sleep(self._timeSleep)


    def get_action_space(self):
        """
        Get the action space as a gym.spaces.Box.

        :return: gym.spaces.Box
        """
        if self._isDiscrete:
            self.action_space = spaces.Discrete(7)
        else:
            self._action_bound = 1
            action_high = np.array([self._action_bound] * self._action_dim)
            self.action_space = spaces.Box(-action_high, action_high)

    def _termination(self):
        # robot index 4 indicates the hand. Check if hand is near the ground
        closest_point = p.getClosestPoints(self.plane, self.robot.robot, 0.005, -1, 4)
        print(closest_point)
        if len(closest_point):
            self.terminated = 1

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)

    def __del__(self):
        p.disconnect()
