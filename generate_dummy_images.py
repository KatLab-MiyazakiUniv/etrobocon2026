#!/usr/bin/env python3
import os
import cv2
import numpy as np
import time

def main():
    out_dir = "datafiles/line_trace"
    os.makedirs(out_dir, exist_ok=True)
    
    print(f"ダミー画像を生成中: {out_dir}")
    
    # 30フレーム生成する
    for i in range(30):
        # 800x600の画像を作成
        img = np.zeros((600, 800, 3), dtype=np.uint8)
        
        # 背景をグレーにする
        img.fill(220)
        
        # 青い円が左から右に動くアニメーション
        cx = 100 + i * 20
        cy = 300
        cv2.circle(img, (cx, cy), 50, (255, 0, 0), -1)
        
        # ROI設定 (円の周辺に設定)
        rx = cx - 60
        ry = cy - 60
        rw = 120
        rh = 120
        
        # タイムスタンプ（ミリ秒）
        timestamp = int(time.time() * 1000) + i
        filename = f"roi_x{rx}_y{ry}_w{rw}_h{rh}_{timestamp}.JPEG"
        filepath = os.path.join(out_dir, filename)
        
        cv2.imwrite(filepath, img)
        
    print("ダミー画像30枚の生成が完了しました。")

if __name__ == "__main__":
    main()
