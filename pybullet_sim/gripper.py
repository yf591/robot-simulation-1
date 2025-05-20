"""
gripper.py
ソフトグリッパーの物理モデル・制御クラス
"""

import pybullet as p
import pybullet_data
import os

class SoftGripper:
    """
    ソフトグリッパーの物理モデルと制御を担当するクラス
    """
    def __init__(self, base_position=(0,0,0.2)):
        self.base_position = base_position
        self.gripper_id = None
        self.finger_links = []  # 指のリンクIDリスト
        self._load_gripper()

    def _load_gripper(self):
        """グリッパーのソフトボディモデルをPyBulletにロード"""
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        # 仮: gripper.objをソフトボディとしてロード
        obj_path = os.path.join(os.path.dirname(__file__), '../ros_ws/src/soft_gripper/meshes/gripper.obj')
        if os.path.exists(obj_path):
            self.gripper_id = p.loadSoftBody(obj_path, basePosition=self.base_position, scale=1.0, mass=0.2, useNeoHookean=1, NeoHookeanMu=180, NeoHookeanLambda=600, NeoHookeanDamping=0.01, collisionMargin=0.001, useSelfCollision=1, frictionCoeff=0.5, repulsionStiffness=800)
        else:
            self.gripper_id = None
        # 指のリンクIDは仮に[0,1,2,3]とする
        self.finger_links = [0,1,2,3]

    def reset(self):
        """グリッパーの初期化"""
        if self.gripper_id is not None:
            p.resetBasePositionAndOrientation(self.gripper_id, self.base_position, [0,0,0,1])
        # 追加のリセット処理があればここに

    def apply_action(self, action):
        """アクション（4本指の力）を適用: actionは長さ4のリスト(-1~1)"""
        if self.gripper_id is None:
            return
        for i, force in enumerate(action):
            # 仮: 各指リンクに外力を加える（実際はノードや頂点単位で調整が必要）
            p.applyExternalForce(self.gripper_id, self.finger_links[i], [0, 0, float(force)], [0,0,0], p.LINK_FRAME)

    def get_finger_positions(self):
        """指のノード位置を取得（例: 4ノードの座標）"""
        if self.gripper_id is None:
            return [0.0, 0.0, 0.0, 0.0]
        # 仮: 先頭4ノードのZ座標を取得
        positions = []
        for i in range(4):
            pos, _ = p.getSoftBodyData(self.gripper_id, i)
            positions.append(pos[2])
        return positions

    def get_finger_forces(self):
        """指の力を取得（仮: 4ノードの合力）"""
        if self.gripper_id is None:
            return [0.0, 0.0, 0.0, 0.0]
        # 仮: 先頭4ノードの力（未実装）
        return [0.0, 0.0, 0.0, 0.0]

    def set_arm_joint_positions(self, joint_positions):
        """アームの関節角度を設定（仮）"""
        # 実際はアームのURDFをPyBulletにロードし、joint_positionsを適用
        pass

    def get_arm_joint_positions(self):
        """アームの関節角度を取得（仮）"""
        # 実際はPyBulletから取得
        return [0.0, 0.0, 0.0]
