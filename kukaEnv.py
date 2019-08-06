import gym
from gym import spaces
from gym.utils import seeding
import numpy as np
import pybullet as p
import pybullet_data

class KukaEnv(gym.Env):
    """
    Openai Gym Environment for the Kuka UR10 arm with pybullet physics.

    """

    def __init__(self, isDiscrete=False, renders=False):
        self._p = p
        self._action_bound = 1
        self._action_dim = 3
        self.terminated = 0
        self._p = p

        self._renders = renders
        self._isDiscrete = isDiscrete

        if self._renders:
            cid = p.connect(p.SHARED_MEMORY)
            if (cid < 0):
                cid = p.connect(p.GUI)
        else:
            p.connect(p.DIRECT)

        self.seed()
        self.get_action_space()

    def reset(self):
        """
        Reset the environment to initial state.

        :return: None
        """
        p.setGravity(0, 0, -9.82)

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
        return False

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)

    def __del__(self):
        p.disconnect()
