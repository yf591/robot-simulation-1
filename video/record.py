"""
record.py
シミュレーション動画の録画スクリプト雛形
"""

import cv2
import numpy as np

def record_simulation(env, filename="output.mp4", duration=5, fps=30):
    """シミュレーション環境を録画してMP4出力（雛形）"""
    width, height = 640, 480
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(filename, fourcc, fps, (width, height))
    for _ in range(duration * fps):
        # 仮: env.render()で画像取得
        frame = np.zeros((height, width, 3), dtype=np.uint8)
        out.write(frame)
    out.release()

if __name__ == "__main__":
    record_simulation()
