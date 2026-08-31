"""
@file   createVideo.py
@brief  走行ログ可視化動画を作成する
@author sadomiya-sousi
"""

import os
import sys
import re
import glob
import argparse
import subprocess
import time
import cv2
import numpy as np
from concurrent.futures import ProcessPoolExecutor

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
  parser.add_argument("-r", "--fps", type=int, default= 15,
                      help="出力動画のFPS")
  parser.add_argument("-s", "--scale", type=float, default=0.1,
                      help="画像の縮小比率 (0.1 〜 1.0)")
  parser.add_argument("-g", "--gpu", action="store_true", default=False,
                      help="NVIDIA GPUエンコーダー (h264_nvenc) を使用するかどうか")
  parser.add_argument("-c", "--clean", action="store_true", default=True,
                      help="動画作成完了後に、入力画像ディレクトリ内のファイルをすべて削除するかどうか")
  parser.add_argument("-w", "--num-workers", type=int, default=0,
                      help="並列処理のプロセス数 (0の場合は自動設定)")
  return parser.parse_args()


def collect_and_sort_images(input_dir):
  """
  @brief ディレクトリ内の 'det_' で始まり、拡張子がJPEG/jpg/png等のファイルを収集し、
         ファイル名末尾のタイムスタンプ数値順（昇順）にソートして、
         (ファイルパス, was_detected, tlx, tly, trx, try_val, blx, bly, brx, bry, rx, ry, rw, rh) のリストを返す。
         ファイル名形式: det_d{wasDetected}_tlx{topLeft.x}_tly{topLeft.y}_trx{topRight.x}_try{topRight.y}_blx{bottomLeft.x}_bly{bottomLeft.y}_brx{bottomRight.x}_bry{bottomRight.y}_rx{roi.x}_ry{roi.y}_rw{roi.width}_rh{roi.height}_{timestamp}.JPEG
  @param input_dir 入力JPEG画像のディレクトリのパス
  @return ソート後の (ファイルパス, was_detected, tlx, tly, trx, try_val, blx, bly, brx, bry, rx, ry, rw, rh) のリスト
  """
  pattern = os.path.join(input_dir, "det_*")
  files = glob.glob(pattern)

  # 正規表現でファイル名から座標とタイムスタンプを抽出
  regex = re.compile(
      r'det_d(\d+)_tlx(\d+)_tly(\d+)_trx(\d+)_try(\d+)_blx(\d+)_bly(\d+)_brx(\d+)_bry(\d+)_rx(\d+)_ry(\d+)_rw(\d+)_rh(\d+)_(\d+)\.(?:[jJ][pP][eE]?[gG]|[pP][nN][gG])'
  )

  image_list = []
  for filepath in files:
    basename = os.path.basename(filepath)
    match = regex.match(basename)
    if match:
      was_detected = int(match.group(1))
      tl_x = int(match.group(2))
      tl_y = int(match.group(3))
      tr_x = int(match.group(4))
      tr_y = int(match.group(5))
      bl_x = int(match.group(6))
      bl_y = int(match.group(7))
      br_x = int(match.group(8))
      br_y = int(match.group(9))
      r_x = int(match.group(10))
      r_y = int(match.group(11))
      r_w = int(match.group(12))
      r_h = int(match.group(13))
      timestamp = int(match.group(14))
      image_list.append((
          filepath, timestamp, was_detected, tl_x, tl_y, tr_x, tr_y, bl_x, bl_y,
          br_x, br_y, r_x, r_y, r_w, r_h
      ))

  # タイムスタンプ順で昇順ソート
  image_list.sort(key=lambda item: item[1])

  # ソート後はタイムスタンプを除外したリストを返す
  return [
      (item[0], item[2], item[3], item[4], item[5], item[6], item[7], item[8],
       item[9], item[10], item[11], item[12], item[13], item[14])
      for item in image_list
  ]


def scale_coords(scale, *coords):
  """
  @brief 複数の座標値をスケール比率に合わせて整数型に変換する
  """
  return [int(c * scale) for c in coords]


def draw_outlined_text(img, text, pos, font_scale, color):
  """
  @brief テキストを描画
  """
  cv2.putText(
      img, text, pos,
      cv2.FONT_HERSHEY_SIMPLEX, font_scale, color, 1, cv2.LINE_AA
  )

def get_text_color(text):
  """
  @brief テキストの内容に応じて色を選択する (BGR)
  """
  if "Detected: False" in text:
    return (128, 128, 255)  # 薄い赤/ピンク
  if "Detected:" in text:
    return (0, 0, 255)      # 赤
  if "ROI:" in text:
    return (0, 255, 0)      # 緑
  return (255, 255, 255)    # 白

def draw_frame_annotations(img, item, scale, width):
  """
  @brief 画像に対してROI枠、検出BoundingBox、および関連テキストを描画する
  @param img アノテーションを描画する画像オブジェクト (cv::Mat)
  @param item 描画に必要なアノテーション情報タプル
  @param scale 画像の縮小比率
  @param width 画像の横幅 (フォントサイズ調整用)
  """
  (_, was_detected, tlx, tly, trx, try_val, blx, bly, brx, bry, rx, ry, rw, rh) = item

  # ROIの描画 (緑色: (0, 255, 0), 太さ2)
  rrx, rry, rrw, rrh = scale_coords(scale, rx, ry, rw, rh)
  cv2.rectangle(img, (rrx, rry), (rrx + rrw, rry + rrh), (0, 255, 0), 2)

  if was_detected == 1:
    # BoundingBoxの頂点をスケーリング
    rtlx, rtly, rtrx, rtry, rblx, rbly, rbrx, rbry = scale_coords(
        scale, tlx, tly, trx, try_val, blx, bly, brx, bry
    )

    # 頂点を結びポリゴンを描画（太さ1, BGR赤: (0, 0, 255)）
    pts = np.array([[rtlx, rtly], [rtrx, rtry], [rbrx, rbry], [rblx, rbly]], np.int32)
    pts = pts.reshape((-1, 1, 2))
    cv2.polylines(img, [pts], isClosed=True, color=(0, 0, 255), thickness=2)

  # テキスト情報の重畳 (複数行表示)
  info_texts = []
  if was_detected == 1:
    info_texts.append(f"Detected: TL({tlx},{tly}), BR({brx},{bry})")
  else:
    info_texts.append("Detected: False")
  info_texts.append(f"ROI: ({rx},{ry}) {rw}x{rh}")

  # フォントサイズや位置を適宜調整
  font_scale = 0.5 * (width / 400.0) # 画像幅に応じた自動調整
  font_scale = max(0.4, min(font_scale, 0.8))

  y_offset = int(30 * font_scale * 1.5)
  line_height = int(25 * font_scale * 1.5)

  for i, text in enumerate(info_texts):
    pos = (10, y_offset + i * line_height)
    color = get_text_color(text)
    draw_outlined_text(img, text, pos, font_scale, color)


def cleanup_files(files, file_to_remove=None):
  """
  @brief 指定されたファイルを削除してクリーンアップする
  @param files 削除するファイルパスのリスト
  @param file_to_remove 個別に削除する単一のファイルパス (オプション)
  """
  if file_to_remove and os.path.exists(file_to_remove):
    try:
      os.remove(file_to_remove)
    except Exception as e:
      print(f"WARNING: ファイルの削除に失敗しました: {file_to_remove} ({e})", file=sys.stderr)
  for path in files:
    if os.path.exists(path):
      try:
        os.remove(path)
      except Exception as e:
        print(f"WARNING: 一時ファイルの削除に失敗しました: {path} ({e})", file=sys.stderr)


def process_chunk(chunk_index, image_subset, args, width, height, temp_output_path):
  """
  @brief 割り当てられた画像リストの処理を行い、部分動画ファイルを生成する
  """
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
      temp_output_path
  ]

  try:
    proc = subprocess.Popen(
        ffmpeg_cmd,
        stdin=subprocess.PIPE,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE
    )
  except FileNotFoundError:
    print(f"ERROR [Chunk {chunk_index}]: ffmpeg コマンドが見つかりませんでした。PATHに登録されているか確認してください。",
          file=sys.stderr)
    return False

  success_count = 0
  try:
    for item in image_subset:
      path = item[0]
      img = cv2.imread(path)
      if img is None:
        print(f"WARNING: 画像の読み込みに失敗しました。スキップします: {path}", file=sys.stderr)
        continue

      # 画像のリサイズ
      resized = cv2.resize(img, (width, height))

      # アノテーションの描画
      draw_frame_annotations(resized, item, args.scale, width)

      # FFmpegの標準入力に生画像バイナリを書き込む
      proc.stdin.write(resized.tobytes())
      success_count += 1

      # 進捗表示
      if success_count % 100 == 0 or success_count == len(image_subset):
        print(f"INFO [Chunk {chunk_index}]: {success_count} / {len(image_subset)} 枚処理完了")

  except IOError as e:
    print(f"ERROR [Chunk {chunk_index}]: FFmpegへの書き込み中にパイプエラーが発生しました: {e}", file=sys.stderr)
  finally:
    if proc.stdin:
      proc.stdin.close()
    stderr_data = b""
    if proc.stderr:
      stderr_data = proc.stderr.read()
    return_code = proc.wait()

  if return_code != 0:
    print(f"ERROR [Chunk {chunk_index}]: FFmpegが異常終了しました (終了コード: {return_code})", file=sys.stderr)
    if stderr_data:
      print(stderr_data.decode('utf-8', errors='ignore'), file=sys.stderr)
    return False

  return True


def main():
  """
  @brief 画像をつなげて動画化
  """
  args = parse_args()

  if args.fps < 1:
    print(f"ERROR:FPSは1より大きい値を指定してください。入力値:{args.fps}" ,file=sys.stderr)
    sys.exit(1)

  if args.scale <= 0:
    print(f"ERROR:scaleは0より大きい値を入力してください。入力値:{args.fps}" ,file=sys.stderr)
    sys.exit(1)

  if not os.path.exists(args.input_dir):
    print(f"ERROR:入力ディレクトリ '{args.input_dir}' が存在しません。", file=sys.stderr)
    sys.exit(1)

  print(f"INFO:画像ファイルをスキャン中: {args.input_dir}")
  images = collect_and_sort_images(args.input_dir)
  total_images = len(images)

  if total_images == 0:
    print("WARNING:処理対象の画像ファイルが見つかりませんでした。", file=sys.stderr)
    sys.exit(0)

  print(f"INFO:合計 {total_images} 枚の画像を見つけました。")

  # 最初の有効な画像を読み込んでサイズを決定する
  first_img = None
  for path, *_ in images:
    first_img = cv2.imread(path)
    if first_img is not None:
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

  # 並列処理のワーカー数の決定
  num_workers = args.num_workers
  if num_workers <= 0:
    num_workers = min(4, os.cpu_count() or 1)
  num_workers = min(num_workers, total_images)

  # 画像リストをチャンクに分割
  chunk_size = int(np.ceil(total_images / num_workers))
  chunks = [images[i * chunk_size : (i + 1) * chunk_size] for i in range(num_workers)]
  chunks = [c for c in chunks if len(c) > 0]
  num_workers = len(chunks)

  start_time = time.time()

  # 一時ファイルパスの設定
  temp_files = [f"{args.output}.temp_{i}.mp4" for i in range(num_workers)]

  print(f"INFO: {num_workers} 個のプロセスで並列処理を開始します...")
  success = True
  with ProcessPoolExecutor(max_workers=num_workers) as executor:
    futures = []
    for i in range(num_workers):
      futures.append(executor.submit(
          process_chunk, i, chunks[i], args, width, height, temp_files[i]
      ))

    for i, future in enumerate(futures):
      try:
        res = future.result()
        if not res:
          success = False
      except Exception as e:
        print(f"ERROR: プロセス {i} の実行中に例外が発生しました: {e}", file=sys.stderr)
        success = False

  if not success:
    print("ERROR: 一部の並列処理が失敗しました。動画化を中止します。", file=sys.stderr)
    cleanup_files(temp_files)
    sys.exit(1)

  # 動画の結合 (FFmpeg concat demuxer)
  concat_file = f"{args.output}.concat.txt"
  try:
    with open(concat_file, "w", encoding="utf-8") as f:
      for path in temp_files:
        # 絶対パスに変換し、Windows環境用にバックスラッシュをスラッシュに置換
        abs_path = os.path.abspath(path).replace("\\", "/")
        f.write(f"file '{abs_path}'\n")

    concat_cmd = [
        "ffmpeg",
        "-y",
        "-f", "concat",
        "-safe", "0",
        "-i", concat_file,
        "-c", "copy",
        args.output
    ]
    print(f"INFO: 動画を結合中: {' '.join(concat_cmd)}")
    subprocess.run(concat_cmd, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)

  except subprocess.CalledProcessError as e:
    print(f"ERROR: 動画の結合に失敗しました: {e}", file=sys.stderr)
    if e.stderr:
      print(e.stderr.decode('utf-8', errors='ignore'), file=sys.stderr)
    success = False
  finally:
    # クリーンアップ
    cleanup_files(temp_files, concat_file)

  if not success:
    sys.exit(1)

  end_time = time.time()
  elapsed_time = end_time - start_time
  average_fps = total_images / elapsed_time if elapsed_time > 0 else 0

  print(
      f"INFO:動画の作成が完了しました: {args.output} \n"
      f"  - 総処理時間: {elapsed_time:.2f} 秒\n"
      f"  - 動画化のフレーム連結の平均FPS: {average_fps:.2f}"
  )

  # クリーンアップ処理 (元のJPEG画像)
  if args.clean and len(images) > 0:
    print("INFO:入力画像ファイルのクリーンアップ中...")
    deleted_count = 0
    for path, *_ in images:
      try:
        if os.path.exists(path):
          os.remove(path)
          deleted_count += 1
      except Exception as e:
        print(f"INFO: ファイルの削除に失敗しました: {path} ({e})", file=sys.stderr)
    print(f"INFO:{deleted_count} 枚の入力画像を削除しました。")


if __name__ == "__main__":
  main()
