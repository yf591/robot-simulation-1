# RobotSimulation1設計書（ドラフト版）
食品産業向けソフトグリッパーロボットシミュレーション（ROS・Gazebo対応）を想定

## 1. プロジェクト概要

### 1.1 目的
食品産業（選別、包装、加工）向けに、柔軟なソフトグリッパーを用いたロボットをROSとGazebo（またはGenesis）でシミュレーション。強化学習（RL）でグリッパーの変形と力加減を最適化し、形状・硬さ・重量の異なる食品（リンゴ、トマト、卵、パン）を傷つけずハンドリング。1〜5分の動画を生成しシミュレーションを外部向けに可視化。

### 1.2 実社会での用途
- **選別**: 果物・野菜（例: トマト）を潰さず品質別に仕分け。
- **包装**: 壊れやすい食品（例: 卵）をトレイに配置。
- **加工**: 柔らかいパンをコンベア間で移動。
- **価値**
  - 品質向上（傷ゼロで商品価値維持）。
  - 労働力不足解消（自動化で人件費削減）。
  - 衛生性（非接触に近いハンドリング）。

### 1.3 技術的特徴
- **ソフトボディ**: Gazeboのソフトボディプラグイン（またはPyBullet互換）で柔軟なグリッパーを実現。
- **強化学習**: Stable-Baselines3のPPOで適応型制御を学習。
- **ゼロコスト**: ROS、Gazebo、Stable-Baselines3、OpenCV、Blender（無料）を活用。
- **環境**: RLトレーニングはGoogle Colab Pro（A100 GPU）、シミュレーションはローカルまたはクラウドでROS+Gazebo。
- **動画**: 1〜5分、MP4形式、マルチアングルで食品ハンドリングを強調。

### 1.4 制約
- **ツール**: ROS（Noetic推奨）、Gazebo、Genesis（またはPyBullet互換シミュレータ）。
- **動画**: 1〜5分、視覚的インパクト重視。
- **ファイル形式**
  - RLトレーニング: `.ipynb`（Colab、A100）。
  - その他（シミュレーション、動画生成）: `.py`。
- **環境**: ゼロコスト（無料ソフトのみ）。

### 1.5 リポジトリ構成（案）
```
robot-simulation-1/
├── .venv/                        # 仮想環境（.gitignoreで無視）
├── ros_ws/                       # ROSワークスペース
│   ├── src/
│   │   ├── soft_gripper/         # ROSパッケージ
│   │   │   ├── launch/           # 起動ファイル
│   │   │   │   ├── simulation.launch      # Gazeboシミュレーション起動
│   │   │   │   └── pybullet_bridge.launch # PyBullet-ROS連携
│   │   │   ├── urdf/             # ロボットモデル
│   │   │   │   ├── arm.urdf      # ロボットアーム
│   │   │   │   └── gripper.urdf  # グリッパー
│   │   │   ├── meshes/           # 3Dモデル
│   │   │   │   └── gripper.obj   # ソフトグリッパーメッシュ
│   │   │   ├── worlds/           # Gazebo環境
│   │   │   │   ├── food_sorting.world  # 選別環境
│   │   │   │   └── packaging.world     # 包装環境
│   │   │   ├── scripts/          # ROSノード
│   │   │   │   ├── control_node.py   # コントローラー
│   │   │   │   └── simulation_node.py # シミュレーション管理
│   │   │   ├── config/           # パラメータ
│   │   │   └── CMakeLists.txt    # ビルド設定
│   │   │   └── package.xml       # パッケージ情報
│   │   └── soft_gripper_msgs/    # メッセージ定義
├── pybullet_sim/                 # PyBullet実装
│   ├── gripper.py                # ソフトグリッパー実装
│   ├── rl_env.py                 # 強化学習環境
│   ├── utils.py                  # ユーティリティ
│   └── food_models/              # 食品モデル
├── notebooks/                    # Jupyter/Colab
│   ├── train_ppo.ipynb           # 強化学習トレーニング
│   └── visualization.ipynb       # 結果可視化
├── video/                        # 動画生成
│   ├── record.py                 # 録画スクリプト
│   └── editing.py                # 編集スクリプト
├── models/                       # 保存モデル
│   └── ppo_gripper.zip           # 学習済みPPOモデル
├── requirements.txt              # 依存ライブラリ
├── setup.py                      # パッケージ設定
└── RobotSimulation1_Design.md    # 設計書
```

## 2. 技術スタック詳細

| カテゴリ | 技術 | バージョン | 用途 |
|---------|------|------------|------|
| **OS/プラットフォーム** | Ubuntu | 20.04 | ROSとGazebo実行環境 |
| | Google Colab Pro | A100 GPU | 強化学習トレーニング |
| **ロボティクス** | ROS | Noetic | ロボット制御フレームワーク |
| | Gazebo | 11.x | 物理シミュレータ |
| | PyBullet | 3.2.5 | 代替シミュレータ |
| | MoveIt | 1.1.11 | モーションプランニング |
| **強化学習** | Stable-Baselines3 | 2.0.0 | PPO実装 |
| | Gym | 0.21.0 | 強化学習環境 |
| | PyTorch | 2.0.0+ | ディープラーニング |
| **視覚化/動画** | OpenCV | 4.7.0+ | 動画生成・編集 |
| | Matplotlib | 3.5.0+ | グラフ作成 |
| | FFmpeg | 4.4.2 | 動画エンコード |
| **3Dモデリング** | Blender | 3.3.0+ | グリッパー・食品モデル作成 |
| **その他** | NumPy | 1.22.0+ | 数値計算 |
| | SciPy | 1.8.0+ | 科学計算 |

## 3. 技術スタック連携図

```mermaid
graph TD
    Development["開発環境"] --> Local["ローカル開発<br>VSCode"]
    Development --> Colab["Colab<br>A100 GPU"]
    Development --> Drive["Google Drive<br>ストレージ"]
    
    Local <--> Colab
    Colab <--> Drive
    
    Local --> SimEnv["シミュレーション環境"]
    Colab --> RLEnv["強化学習環境"]
    
    subgraph SimEnv["シミュレーション環境"]
        ROS["ROS"] --> Gazebo
        ROS --> MoveIt
        Gazebo <--> PyBullet
    end
    
    subgraph RLEnv["強化学習環境"]
        Gym --> PyBullet2["PyBullet"]
        Gym --> SB3["Stable-Baselines3"]
    end
    
    SimEnv <--> RLEnv
    
    ModelDef["モデル定義"] --> Blender
    Blender --> URDF["URDF/SDF"]
    
    RLEnv --> Results["結果分析"]
    Results --> Matplotlib
    Results --> OpenCV
    
    SimEnv --> VideoGen["動画生成"]
    VideoGen --> OpenCV2["OpenCV"]
    
    Matplotlib --> Output["最終成果物"]
    OpenCV --> Output
    OpenCV2 --> Output
    
    Output --> Model["学習済み<br>モデル"]
    Output --> Video["MP4動画"]
```

## 4. データフロー

1. **モデリングとシミュレーション準備**
   - Blenderでグリッパーメッシュ作成 → URDF/SDF変換 → Gazebo/PyBulletでロード
   - 食品モデルの物理パラメータ設定 → シミュレーション環境に配置

2. **シミュレーション環境と学習環境の連携**
   - シミュレーション状態 → 観測データに変換 → Gym環境に提供
   - 学習モデルからのアクション → シミュレータに適用

3. **学習プロセス**
   - PyBullet環境でトレーニング → モデルをGoogleドライブに保存
   - 学習済みモデルをローカル環境で読み込み → ROS/Gazeboに接続

4. **結果検証と動画生成**
   - シミュレーション結果を複数アングルから録画
   - 物理データとパフォーマンス指標を可視化
   - 動画編集（字幕、グラフ挿入）→ 最終MP4出力

## 5. システム構成

### 5.1 シミュレーション環境
- **プラットフォーム**: ROS（Noetic）、Gazebo（11.x、ソフトボディ対応）。
- **代替シミュレータ**: Genesis（未対応の場合、PyBulletをGazebo代替としてROS統合）。
- **トレーニング環境**: Google Colab Pro（A100 GPU、RL専用）。
- **ライブラリ**
  - `ros-noetic-desktop-full`: ROS基盤。
  - `gazebo_ros`: Gazebo-ROS統合。
  - `pybullet`（代替シミュレータの場合）。
  - `stable-baselines3`: RL（PPO）。
  - `gym`: RL環境定義。
  - `opencv-python`: 動画生成。
  - `numpy`: データ処理。
- **ストレージ**: Google Drive（Colab用）、ローカル（ROS/Gazebo用）。

### 5.2 ロボット設計
- **ソフトグリッパー**
  - **形状**: 4本の指状メッシュ（食品を包み込む）。
  - **実装**: GazeboのSDF（SoftBodyプラグイン）またはPyBulletの`createSoftBody`（ROS統合）。
  - **制御**: ROSトピック（`/gripper_force`）でノードに力（収縮・膨張）を適用。
  - **パラメータ**
    - 線形剛性: `kLST=0.08`（PyBullet）またはGazebo相当。
    - ダンピング: `kDP=0.05`。
    - 摩擦: `kCHR=0.5`。
- **ロボットアーム**
  - 3自由度アーム（URDF、FANUCを模擬）。
  - ROS MoveItで動作計画、`/joint_states`で制御。
- **食品モデル**
  - **リンゴ**: 球（質量=0.2kg、摩擦=0.8）。
  - **トマト**: 柔らかい球（SDF/SoftBody、質量=0.15kg、摩擦=0.6）。
  - **卵**: 楕円体（質量=0.05kg、摩擦=0.3）。
  - **パン**: 柔らかい立方体（SoftBody、質量=0.1kg、摩擦=0.5）。
  - BlenderでSDF/URDF自作、またはGazeboモデルリポジトリ使用。

### 5.3 シミュレーションシーン
- **選別**: コンベア（Gazebo SDFで動く平面）にリンゴ・トマトを配置、別コンベアに仕分け。
- **包装**: 卵をトレイ（固定立方体）に配置。
- **加工**: パンをコンベアから加工台に移動。
- **物理設定**
  - 重力: `-9.8 m/s²`。
  - タイムステップ: `0.004秒`（ソフトボディ安定化）。
  - コンベア: ROSトピック（`/conveyor_velocity`）で制御。


## 6. 強化学習（RL）設計

### 6.1 Gym環境
- **状態空間**（12次元）
  - グリッパーのノード位置（GazeboセンサーまたはPyBulletの`getSoftBodyData`）。
  - 食品の位置・姿勢（`/food_pose`）。
  - 接触力と滑り（`/contact_force`）。
- **行動空間**（8次元）
  - 4本指の力（収縮・膨張、-1～1）。
  - アームの関節速度（-0.5～0.5）。
- **報酬関数**
  - 掴む成功（接触点≥3、力≤閾値）: +100。
  - コンベア移動距離: +10 * 距離減少。
  - 破損（過剰力、落下、滑り）: -200。
  - エネルギー効率: -0.05 * 力の総和。
  - 衛生（地面接触）: -50。
- **終了条件**
  - コンベア到達、落下、破損、200ステップ。

### 6.2 RLアルゴリズム
- **アルゴリズム**: PPO（Stable-Baselines3）。
- **ハイパーパラメータ**
  - `n_steps=2048`。
  - `batch_size=2048`。
  - `num_envs=16`（A100並列化）。
  - 総ステップ: 100万（約45分～1時間）。
- **汎化**
  - 食品ランダム化（形状、質量=0.05～0.2kg、摩擦=0.3～0.8）。
  - Domain Randomizationで未知食品に対応。

### 6.3 ファイル形式
- **トレーニング**: `.ipynb`（Colab、A100）。
- **環境・制御**: `.py`（ROSノード、Gazebo統合）。


## 7. 動画生成

### 7.1 撮影内容（1〜5分）
- **選別**: リンゴ・トマトを別コンベアに仕分け（1分）。
- **包装**: 卵をトレイに配置（1分）。
- **加工**: パンを潰さず移動（1分）。
- **学習過程**: 初期（落下）→最終（スムーズ、1分）。
- **概要**: 字幕と報酬曲線で技術説明（30秒）。

### 7.2 撮影方法
- **解像度**: 1280x720（Gazeboカメラプラグイン）。
- **出力**: MP4（OpenCV、`cv2.VideoWriter`）。
- **アングル**:
  - 俯瞰: コンベア全体。
  - グリッパー視点: 食品を包む瞬間。
  - 側面: 変形のダイナミクス。
- **演出**:
  - スローモーション: トマト・卵の掴む瞬間。
  - 背景: 清潔な白いコンベア。
  - 字幕: 「トマト選別: 傷ゼロ」。
  - 報酬曲線: Matplotlibで挿入。

### 7.3 保存
- Google Drive（Colab）：`/content/drive/MyDrive/`。
- ローカル（ROS）：`/ros_ws/videos/`。


## 8. 実装順序（スケジュール）

### 8.1 環境構築と基本実装 (1-5日目)

1. **開発環境設定**
   - 仮想環境構築 (Python 3.8)
   - 依存パッケージのインストール (ROS Noetic, Gazebo, PyBullet)
   - リポジトリ構造の作成

2. **基本モデルの作成**
   - BlenderでソフトグリッパーメッシュOBJファイル作成
   - URDF/SDFによるロボットアーム定義
   - 食品モデル (リンゴ、トマト、卵、パン) 作成

3. **PyBullet環境実装**
   - `gripper.py` ソフトボディ実装
   - ロボットアームとグリッパー統合
   - 食品モデルのインポートとパラメータ調整

### 8.2 Gym環境とRL実装 (6-10日目)

4. **強化学習環境実装**
   - `rl_env.py` Gym環境定義
   - 状態空間と行動空間の設計
   - 報酬関数の実装
   - テスト用の単純な制御ロジック

5. **ROS連携準備**
   - ROSパッケージ構造の作成
   - ROSノード（グリッパー制御、環境管理）実装
   - トピックとサービス定義

6. **Colab学習環境準備**
   - `train_ppo.ipynb` の作成
   - ColabとGoogle Drive連携設定
   - 環境のテスト実行

### 8.3 学習と動画生成 (11-14日目)

7. **強化学習実行**
   - PPOモデルのトレーニング (Colab A100)
   - モデル保存と評価
   - ハイパーパラメータの調整

8. **シミュレーション実行**
   - 学習済みモデルの読み込み
   - 各シナリオ（選別、包装、加工）のシミュレーション実行
   - パフォーマンス評価

9. **動画生成**
   - 各シナリオの録画
   - 動画編集（字幕、マルチアングル、スローモーション）
   - 最終出力（MP4, 1280x720, 30fps）


## 9. 実用性とアピールポイント（仮）

### 9.1 食品産業への貢献
- **課題解決**
  - 労働不足: 作業員1/3削減。
  - 品質: 廃棄ロス50%減。
  - 生産性: 24時間稼働。
- **優位性**
  - 硬いグリッパーに比べ多様性対応。
  - RLで新商品に適応。
- **応用例**
  - トマト選別: 熟度別仕分け。
  - 卵包装: 1分50個。
  - パン移動: 焼きたてを潰さず。

### 9.2 シミュレーションの価値
- 低コストプロトタイピング。
- 動画で食品メーカーへの説得力強化。


## 10. 実行可能性と不具合リスク

### 10.1 ファイル形式の実行可能性
- **RLトレーニング（`.ipynb`, Colab A100）**
  - Stable-Baselines3とGymをColabで実行可能。
  - A100で高速（100万ステップ≈45分）。
  - モデルを`.zip`で保存、`.py`環境に統合。
- **シミュレーション・制御（`.py`, ROS/Gazebo）**
  - ROSノードとしてGazeboを制御可能。
  - ソフトボディはGazeboプラグイン（またはPyBullet）で実装。
  - 動画生成はOpenCVで`.py`スクリプト化。
- **統合**
  - Colabで学習したPPOモデルを`stable_baselines3.PPO.load()`で`.py`環境にインポート。
  - ROSトピック（`/gripper_force`, `/food_pose`）でデータ共有。

### 10.2 不具合リスクと対策
1. **ROS-GazeboとColabの互換性**
   - **リスク**: Gazeboはローカル実行、Colabはクラウドで分離。モデルやデータの受け渡しでエラー（例: バージョン不一致）。
   - **対策**
     - モデルをGoogle Drive経由で共有。
     - Stable-Baselines3のバージョンを統一（例: `2.0.0`）。
     - ROS（Noetic）とColabのPython（3.8）を揃える。
   - **確率**: 低（Drive経由で統合実績あり）。
1. **Gazeboのソフトボディ実装**
   - **リスク**: Gazeboのソフトボディプラグインが不安定、または未対応（Genesis依存）。
   - **対策**
     - PyBulletを代替シミュレータとし、ROS Bridgeで統合（`/pybullet_state`トピック）。
     - GazeboのSDFで簡易ソフトボディ（弾性モデル）を構築。
   - **確率**: 中（プラグイン依存）。
1. **動画生成の負荷**
   - **リスク**: 1〜5分の高解像度動画（1280x720）がColabやローカルで処理遅延。
   - **対策**
     - フレームレートを30fpsに制限。
     - OpenCVで軽量エンコード（H.264）。
     - ローカルPCで最終編集（FFmpeg）。
   - **確率**: 低（Colab Proで十分）。
1. **RLモデルの汎化**
   - **リスク**: Colabで学習したモデルがGazebo環境で性能低下（シミュレーションギャップ）。
   - **対策**
     - Domain Randomizationを強化（食品パラメータを広範囲に）。
     - GazeboとPyBulletの物理パラメータを近づける（例: 摩擦、重力）。
   - **確率**: 中（シミュレータ差異による）。

### 10.3 総合評価
- **実行可能性**: 高。ROS+Gazeboはロボットシミュレーションの標準、Colab A100はRLに最適。PyBullet代替で柔軟性確保。
- **不具合確率**: 低〜中。主なリスク（ソフトボディ、統合）は対策で軽減。
- **推奨**
  - Gazeboのソフトボディを事前確認（プラグイン有無）。
  - 初期テストで`.ipynb`と`.py`のデータ受け渡しを検証。


## 11. Pythonコードのドラフト（案なので未定）

### 11.1 PyBulletコード（`gripper.py`）
````python
# gripper.py: ソフトグリッパーと食品モデル
import pybullet as p
import pybullet_data

class SoftGripper:
    def __init__(self):
        p.connect(p.DIRECT)  # ROS統合時はp.SHARED_MEMORY
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.8)
        self.plane = p.loadURDF("plane.urdf")
        self.conveyor = p.createMultiBody(...)  # コンベアSDF
        self.gripper = self._create_gripper()
        self.food = self._create_food()

    def _create_gripper(self):
        # 4本指グリッパー（BlenderでOBJ作成）
        gripper = p.createSoftBody(
            fileName="gripper.obj",
            mass=0.5,
            collisionMargin=0.001
        )
        p.setSoftBodyParameters(
            gripper,
            kLST=0.08,  # 線形剛性
            kDP=0.05,   # ダンピング
            kCHR=0.5    # 摩擦
        )
        return gripper

    def _create_food(self):
        # ランダム食品（例: リンゴ）
        food = p.createMultiBody(
            baseMass=0.2,
            basePosition=[0, 0, 0.1],
            baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_SPHERE, radius=0.05)
        )
        p.changeDynamics(food, -1, lateralFriction=0.8)
        return food

    def apply_force(self, forces):
        # 4本指に力適用
        for i, node in enumerate([0, 10, 20, 30]):  # 仮ノード
            p.addForce(self.gripper, node, forces[i*2:i*2+2])
````

- **備考**: Gazebo使用時はSDFモデルに置き換え、ROSトピック（/gripper_force）で制御。
    

### 11.2 RL環境の実装（rl_env.py）
````python
# rl_env.py: Gym環境
import gym
import numpy as np
import pybullet as p
from gripper import SoftGripper

class FoodGripperEnv(gym.Env):
    def __init__(self):
        super().__init__()
        self.action_space = gym.spaces.Box(low=-1, high=1, shape=(8,), dtype=np.float32)
        self.observation_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(12,), dtype=np.float32)
        self.gripper = SoftGripper()
        self.step_count = 0
        self.max_steps = 200

    def reset(self):
        self.gripper = SoftGripper()
        self.step_count = 0
        return self._get_obs()

    def step(self, action):
        self.gripper.apply_force(action[:4])
        # アーム制御（仮）
        p.setJointMotorControlArray(...)
        p.stepSimulation()
        self.step_count += 1
        obs = self._get_obs()
        reward = self._compute_reward()
        done = self._is_done()
        return obs, reward, done, {}

    def _get_obs(self):
        gripper_pos = p.getSoftBodyData(self.gripper.gripper, attributes=[p.SOFT_BODY_NODE_POSITION])
        food_pos = p.getBasePositionAndOrientation(self.gripper.food)[0]
        contact = p.getContactPoints(self.gripper.gripper, self.gripper.food)
        contact_force = sum([c[9] for c in contact]) if contact else 0
        return np.array([*gripper_pos[:4], *food_pos, contact_force, 0, 0, 0], dtype=np.float32)

    def _compute_reward(self):
        contact = p.getContactPoints(self.gripper.gripper, self.gripper.food)
        contact_count = len(contact)
        contact_force = sum([c[9] for c in contact]) if contact else 0
        food_pos = p.getBasePositionAndOrientation(self.gripper.food)[0]
        target_pos = [1, 0, 0.1]  # コンベア目標
        distance = np.linalg.norm(np.array(food_pos) - np.array(target_pos))
        reward = 0
        if contact_count >= 3 and contact_force < 10:
            reward += 100
        reward += 10 * (1 - distance)
        if contact_force > 20 or food_pos[2] < 0:
            reward -= 200
        reward -= 0.05 * np.sum(np.abs(action))
        if p.getContactPoints(self.gripper.gripper, self.gripper.plane):
            reward -= 50
        return reward

    def _is_done(self):
        food_pos = p.getBasePositionAndOrientation(self.gripper.food)[0]
        target_pos = [1, 0, 0.1]
        return ( Ascendingly close food_pos[2] < 0 or self.step_count >= self.max_steps
````

- **備考**: ROS統合時は/food_pose、/contact_forceを購読。
    

### 11.3 Colabセットアップ手順（train.ipynb）
````python
# train.ipynb: RLトレーニング
%%capture
!pip install pybullet stable-baselines3 gym opencv-python
!apt-get install -y ros-noetic-ros-base
from google.colab import drive
drive.mount('/content/drive')

import gym
from stable_baselines3 import PPO
from rl_env import FoodGripperEnv
import matplotlib.pyplot as plt

env = FoodGripperEnv()
model = PPO("MlpPolicy", env, verbose=1, n_steps=2048, batch_size=2048, device="cuda")
model.learn(total_timesteps=1_000_000, log_interval=10)

model.save("/content/drive/MyDrive/food_gripper_model")
rewards = model.logger.get_log_dict()['rollout/ep_rew_mean']
plt.plot(rewards)
plt.xlabel("Episode")
plt.ylabel("Mean Reward")
plt.savefig("/content/drive/MyDrive/reward_curve.png")
````

- **手順**
    1. ColabでA100 GPUを選択。
    2. 依存ライブラリをインストール。
    3. Google Driveをマウント。
    4. rl_env.pyとgripper.pyを/content/にアップロード。
    5. トレーニング実行、モデルと報酬曲線を保存。

### 11.4 動画編集スクリプト（video.py）
````python
# video.py: 動画生成
import pybullet as p
import cv2
import numpy as np
from rl_env import FoodGripperEnv
from stable_baselines3 import PPO

def record_video(model, filename="/ros_ws/videos/food_gripper.mp4"):
    env = FoodGripperEnv()
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    out = cv2.VideoWriter(filename, fourcc, 30.0, (1280, 720))
    obs = env.reset()
    for _ in range(3000):  # ~5分
        action, _ = model.predict(obs)
        obs, _, done, _ = env.step(action)
        img = p.getCameraImage(1280, 720, renderer=p.ER_BULLET_HARDWARE_OPENGL)[2]
        img = cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)
        cv2.putText(img, "Tomato Sorting: Zero Damage", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        out.write(img)
        if done:
            obs = env.reset()
    out.release()

if __name__ == "__main__":
    model = PPO.load("/content/drive/MyDrive/food_gripper_model")
    record_video(model) 
````

- 備考: Gazebo使用時はカメラプラグイン（/camera/image_raw）で撮影。
    

## 12. 次のステップ

- **優先**
    - 食品: トマト選別（需要高）、卵包装
    - シーン: 選別（2分）、包装（1.5分）、学習過程（1.5分。これはいらないかも?）
        
- **推奨**
    - Gazeboのソフトボディ対応を事前確認。
    - ROS BridgeでPyBullet統合をテスト。
    - 動画を5分以内に収め、字幕で用途を強調。
