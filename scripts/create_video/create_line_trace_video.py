"""
@file   create_line_trace_video.py
@brief  保存された連続JPEG画像から、BoundingBox・文字を描画しつつ、FFmpegを利用し高速に動画を作成する
@author sadomiya-sousi
"""

import os
import sys
import re
import glob
import argparse
import subprocess
import cv2
import numpy as np


def parse_args():
  """
  @brief コマンドライン引数をパースする
  @return パースされたコマンドライン引数のオブジェクト
  """
  parser = argparse.ArgumentParser(description="画像動画化ツール (Python + FFmpeg)")
  parser.add_argument("-i", "--input-dir", default="datafiles/line_trace",
                      help="入力JPEG画像のディレクトリのパス")
  parser.add_argument("-o", "--output", default="line_trace.mp4",
                      help="出力動画のファイルパス")
  parser.add_argument("-r", "--fps", type=int, default=30,
                      help="出力動画のFPS")
  parser.add_argument("-s", "--scale", type=float, default=0.5,
                      help="画像の縮小比率 (0.1 〜 1.0)")
  parser.add_argument("-g", "--gpu", action="store_true", default=False,
                      help="NVIDIA GPUエンコーダー (h264_nvenc) を使用するかどうか")
  parser.add_argument("-c", "--clean", action="store_true", default=False,
                      help="動画作成完了後に、入力画像ディレクトリ内のファイルをすべて削除するかどうか")
  return parser.parse_args()


def collect_and_sort_images(input_dir):
  """
  @brief ディレクトリ内の 'det_' で始まり、拡張子がJPEG/jpg/png等のファイルを収集し、
         ファイル名末尾のタイムスタンプ数値順（昇順）にソートして、
         (ファイルパス, was_detected, tlx, tly, trx, try_val, blx, bly, brx, bry) のリストを返す。
         ファイル名形式: det_d{wasDetected}_tlx{topLeft.x}_tly{topLeft.y}_trx{topRight.x}_try{topRight.y}_blx{bottomLeft.x}_bly{bottomLeft.y}_brx{bottomRight.x}_bry{bottomRight.y}_{timestamp}.JPEG
  @param input_dir 入力JPEG画像のディレクトリのパス
  @return ソート後の (ファイルパス, was_detected, tlx, tly, trx, try_val, blx, bly, brx, bry) のリスト
  """
  pattern = os.path.join(input_dir, "det_*")
  files = glob.glob(pattern)

  # 正規表現でファイル名から座標とタイムスタンプを抽出
  regex = re.compile(
      r'det_d(\d+)_tlx(\d+)_tly(\d+)_trx(\d+)_try(\d+)_blx(\d+)_bly(\d+)_brx(\d+)_bry(\d+)_(\d+)\.(?:[jJ][pP][eE]?[gG]|[pP][nN][gG])'
  )

  image_list = []
  for filepath in files:
    basename = os.path.basename(filepath)
    match = regex.match(basename)
    if match:
      was_detected = int(match.group(1))
      tlx = int(match.group(2))
      tly = int(match.group(3))
      trx = int(match.group(4))
      try_val = int(match.group(5))
      blx = int(match.group(6))
      bly = int(match.group(7))
      brx = int(match.group(8))
      bry = int(match.group(9))
      timestamp = int(match.group(10))
      image_list.append((
          filepath, timestamp, was_detected, tlx, tly, trx, try_val, blx, bly,
          brx, bry
      ))

  # タイムスタンプ順で昇順ソート
  image_list.sort(key=lambda item: item[1])

  # ソート後はタイムスタンプを除外したリストを返す
  return [
      (item[0], item[2], item[3], item[4], item[5], item[6], item[7], item[8],
       item[9], item[10])
      for item in image_list
  ]


def main():
  """
  @brief 画像をつなげて動画化
  """
  args = parse_args()

  if not os.path.exists(args.input_dir):
    print(f"ERROR: 入力ディレクトリ '{args.input_dir}' が存在しません。", file=sys.stderr)
    sys.exit(1)

  print(f"INFO画像ファイルをスキャン中: {args.input_dir}")
  images = collect_and_sort_images(args.input_dir)
  total_images = len(images)

  if total_images == 0:
    print("WARNING: 処理対象の画像ファイルが見つかりませんでした。", file=sys.stderr)
    sys.exit(0)

  print(f"INFO:合計 {total_images} 枚の画像を見つけました。")

  # 最初の有効な画像を読み込んでサイズを決定する
  first_img = None
  first_idx = 0
  for idx, (path, *_) in enumerate(images):
    first_img = cv2.imread(path)
    if first_img is not None:
      first_idx = idx
      break

  if first_img is None:
    print("ERROR: すべての画像の読み込みに失敗しました。", file=sys.stderr)
    sys.exit(1)

  # リサイズ後のサイズを計算
  orig_h, orig_w = first_img.shape[:2]
  width = int(orig_w * args.scale)
  height = int(orig_h * args.scale)

  # 横幅と縦幅はFFmpegの要件として偶数である必要がある (yuv420p用)
  if width % 2 != 0:
    width += 1
  if height % 2 != 0:
    height += 1

  print(f"INFO:動画出力サイズ: {width}x{height} (縮小率: {args.scale})")

  # FFmpegコマンドの設定
  vcodec = "h264_nvenc" if args.gpu else "libx264"
  ffmpeg_cmd = [
      "ffmpeg",
      "-y",               # 既存ファイルの強制上書き
      "-f", "rawvideo",   # 生画像入力
      "-vcodec", "rawvideo",
      "-s", f"{width}x{height}",
      "-pix_fmt", "bgr24",
      "-r", str(args.fps),
      "-i", "-",          # 標準入力から受け取る
      "-c:v", vcodec,
      "-pix_fmt", "yuv420p",
      args.output
  ]

  print(f"INFO:FFmpegを起動中: {' '.join(ffmpeg_cmd)}")
  try:
    proc = subprocess.Popen(
        ffmpeg_cmd,
        stdin=subprocess.PIPE,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE
    )
  except FileNotFoundError:
    print("ERROR: ffmpeg コマンドが見つかりませんでした。PATHに登録されているか確認してください。",
          file=sys.stderr)
    sys.exit(1)

  success_count = 0
  processed_files = []

  try:
    for idx, (path, was_detected, tlx, tly, trx, try_val, blx, bly, brx, bry) in enumerate(images):
      # 最初の有効画像は再読込せず流用する
      if idx == first_idx and success_count == 0:
        img = first_img
      else:
        img = cv2.imread(path)

      if img is None:
        print(f"WARNING: 画像の読み込みに失敗しました。スキップします: {path}", file=sys.stderr)
        continue

      # 画像のリサイズ
      resized = cv2.resize(img, (width, height))

      if was_detected == 1:
        # BoundingBoxの4つの頂点
        rtlx = int(tlx * args.scale)
        rtly = int(tly * args.scale)
        rtrx = int(trx * args.scale)
        rtry = int(try_val * args.scale)
        rblx = int(blx * args.scale)
        rbly = int(bly * args.scale)
        rbrx = int(brx * args.scale)
        rbry = int(bry * args.scale)

        # 頂点を結びポリゴンを描画（太さ2, BGR赤: (0, 0, 255)）
        pts = np.array([[rtlx, rtly], [rtrx, rtry], [rbrx, rbry], [rblx, rbly]], np.int32)
        pts = pts.reshape((-1, 1, 2))
        cv2.polylines(resized, [pts], isClosed=True, color=(0, 0, 255), thickness=2)

        # テキスト情報の重畳 (検出結果の座標を表示)
        text = f"Detected: TL({tlx},{tly}), BR({brx},{bry})"
      else:
        # テキスト情報の重畳 (未検出を表示)
        text = "Detected: False"

      # フォントサイズや位置を適宜調整
      font_scale = 0.5 * (width / 400.0) # 画像幅に応じた自動調整
      font_scale = max(0.4, min(font_scale, 1.0))
      cv2.putText(
          resized, text, (10, int(30 * font_scale * 1.5)),
          cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 0, 255), 1, cv2.LINE_AA
      )

      # FFmpegの標準入力に生画像バイナリを書き込む
      proc.stdin.write(resized.tobytes())

      success_count += 1
      processed_files.append(path)

      # 定期的な進捗出力（50枚ごと）
      if success_count % 50 == 0 or success_count == total_images:
        print(f"INFO: {success_count} / {total_images} 枚処理完了")

  except IOError as e:
    print(f"ERROR: FFmpegへの書き込み中にパイプエラーが発生しました: {e}", file=sys.stderr)
  finally:
    if proc.stdin:
      proc.stdin.close()
    stderr_data = b""
    if proc.stderr:
      stderr_data = proc.stderr.read()
    return_code = proc.wait()

  if return_code != 0:
    print(f"ERROR: FFmpegが異常終了しました (終了コード: {return_code})", file=sys.stderr)
    if stderr_data:
      print(stderr_data.decode('utf-8', errors='ignore'), file=sys.stderr)
    sys.exit(1)

  print(
      f"INFO:動画の作成が完了しました: {args.output} "
      f"(正常処理: {success_count}枚 / 総数: {total_images}枚)"
  )

  # クリーンアップ処理
  if args.clean and success_count > 0:
    print("INFO:入力画像ファイルのクリーンアップ中...")
    deleted_count = 0
    for path in processed_files:
      try:
        if os.path.exists(path):
          os.remove(path)
          deleted_count += 1
      except Exception as e:
        print(f"INFO: ファイルの削除に失敗しました: {path} ({e})", file=sys.stderr)
    print(f"INFO:{deleted_count} 枚の入力画像を削除しました。")


if __name__ == "__main__":
  main()
