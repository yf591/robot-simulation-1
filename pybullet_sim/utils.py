"""
utils.py
共通ユーティリティ関数
"""

import pybullet as p

def load_food_model(name, base_position=[0.1,0,0.1]):
    """食品モデルのロード（球体や楕円体など）"""
    if name == "apple":
        # 球体: リンゴ
        visual = p.createVisualShape(p.GEOM_SPHERE, radius=0.03, rgbaColor=[1,0,0,1])
        collision = p.createCollisionShape(p.GEOM_SPHERE, radius=0.03)
        food_id = p.createMultiBody(baseMass=0.2, baseCollisionShapeIndex=collision, baseVisualShapeIndex=visual, basePosition=base_position)
    elif name == "tomato":
        visual = p.createVisualShape(p.GEOM_SPHERE, radius=0.03, rgbaColor=[1,0.3,0.3,1])
        collision = p.createCollisionShape(p.GEOM_SPHERE, radius=0.03)
        food_id = p.createMultiBody(baseMass=0.15, baseCollisionShapeIndex=collision, baseVisualShapeIndex=visual, basePosition=base_position)
    elif name == "egg":
        visual = p.createVisualShape(p.GEOM_ELLIPSOID, radius=[0.025,0.018,0.018], rgbaColor=[1,1,0.8,1])
        collision = p.createCollisionShape(p.GEOM_ELLIPSOID, radius=[0.025,0.018,0.018])
        food_id = p.createMultiBody(baseMass=0.05, baseCollisionShapeIndex=collision, baseVisualShapeIndex=visual, basePosition=base_position)
    elif name == "bread":
        visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.03,0.03,0.015], rgbaColor=[1,0.9,0.7,1])
        collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.03,0.03,0.015])
        food_id = p.createMultiBody(baseMass=0.1, baseCollisionShapeIndex=collision, baseVisualShapeIndex=visual, basePosition=base_position)
    else:
        food_id = None
    return food_id

def calculate_reward(contact_points=0, force=0, ground_contact=False, distance_delta=0):
    """報酬計算のユーティリティ（設計書仕様の雛形）"""
    reward = 0
    if contact_points >= 3 and force <= 1.0:
        reward += 100  # 掴む成功
    reward += 10 * distance_delta  # 移動距離
    if force > 2.0:
        reward -= 200  # 破損
    if ground_contact:
        reward -= 50  # 衛生
    reward -= 0.05 * abs(force)  # エネルギー効率
    return reward
