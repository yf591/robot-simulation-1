"""
rl_env.py
強化学習用Gym環境
"""

import gym
import numpy as np
from gripper import SoftGripper
import pybullet as p
import pybullet_data
from utils import calculate_reward, load_food_model

class FoodGripperEnv(gym.Env):
    """
    ソフトグリッパーと食品モデルの強化学習環境
    """
    def __init__(self, gui=False):
        super().__init__()
        self.gui = gui
        self.gripper = SoftGripper()
        self.action_space = gym.spaces.Box(low=-1, high=1, shape=(8,), dtype=np.float32)  # 4本指+アーム3軸+1予備
        self.observation_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(12,), dtype=np.float32)
        self.state = None
        self.step_count = 0
        self.max_steps = 200
        self.physics_client = None
        self.food_id = None
        self.food_type = "apple"
        self.food_pos = [0.1, 0, 0.1]
        self.food_ori = [0, 0, 0, 1]
        self.contact_points = 0
        self.force = 0
        self.ground_contact = False
        self.distance_delta = 0
        self.last_food_pos = None
        self._init_sim()

    def _init_sim(self):
        if self.physics_client is not None:
            p.disconnect(self.physics_client)
        if self.gui:
            self.physics_client = p.connect(p.GUI)
        else:
            self.physics_client = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.8)
        p.loadURDF("plane.urdf")
        self.gripper = SoftGripper()
        self.food_id = load_food_model(self.food_type)
        self.last_food_pos = self.food_pos.copy()

    def reset(self):
        self._init_sim()
        self.step_count = 0
        self.contact_points = 0
        self.force = 0
        self.ground_contact = False
        self.distance_delta = 0
        obs = self._get_obs()
        self.state = obs
        return obs

    def _get_obs(self):
        # グリッパーのノード位置（4点）、食品位置、接触力
        gripper_pos = self.gripper.get_finger_positions()
        gripper_forces = self.gripper.get_finger_forces()
        food_pos, food_ori = self.food_pos, self.food_ori
        obs = np.array(gripper_pos + gripper_forces + list(food_pos), dtype=np.float32)
        return obs[:12]

    def step(self, action):
        self.gripper.apply_action(action[:4])
        # アームの関節制御（仮: action[4:7]）
        self.gripper.set_arm_joint_positions(action[4:7])
        p.stepSimulation()
        # 食品の現在位置取得
        food_pos, _ = p.getBasePositionAndOrientation(self.food_id)
        self.food_pos = food_pos
        # 接触判定
        contacts = p.getContactPoints(bodyA=self.gripper.gripper_id, bodyB=self.food_id)
        self.contact_points = len(contacts)
        self.force = sum([c[9] for c in contacts]) if contacts else 0.0
        # 地面接触判定
        ground_contacts = p.getContactPoints(bodyA=self.food_id, bodyB=0)
        self.ground_contact = len(ground_contacts) > 0
        # 移動距離
        if self.last_food_pos is not None:
            self.distance_delta = np.linalg.norm(np.array(food_pos) - np.array(self.last_food_pos))
        self.last_food_pos = food_pos
        obs = self._get_obs()
        reward = calculate_reward(contact_points=self.contact_points, force=self.force, ground_contact=self.ground_contact, distance_delta=self.distance_delta)
        self.step_count += 1
        done = self.step_count >= self.max_steps
        info = {}
        self.state = obs
        return obs, reward, done, info

    def render(self, mode="human"):
        """可視化"""
        # GUIの場合は何もしない（ウィンドウ維持）
        pass
