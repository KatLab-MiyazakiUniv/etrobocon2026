/**
 * @file   SquareDetector.cpp
 * @brief  正方形検出用の画像処理クラス
 * @author okuyama0528
 */

#include "SquareDetector.h"

#include <algorithm>
#include <climits>
#include <cmath>
#include <limits>

namespace {

  /**
   * @brief 正方形候補の情報
   */
  struct SquareCandidate {
    cv::RotatedRect rect;
    double area;
    double distance;
  };

  /**
   * @brief 初回検出時に候補として残す面積割合
   *
   * 最大面積の70%以上の候補だけを、
   * 初回検出時の選択対象とする。
   *
   * 画像中央付近に小さなノイズが存在した場合に、
   * そのノイズを誤って選択することを防ぐ。
   */
  constexpr double INITIAL_AREA_RATIO = 0.70;

  /**
   * @brief 初回検出時に狙う画像上のY位置
   *
   * ゲートの正方形は画像の下側に映るため、
   * 画像高さの82%付近を基準位置とする。
   *
   * 1920x1080の場合
   *
   * targetY = 1080 * 0.82
   *         = 約886 pixel
   */
  constexpr double INITIAL_TARGET_Y_RATIO = 0.82;

}  // namespace

SquareDetector::SquareDetector(
    const cv::Rect& _roi)
  : roi(_roi),
    previousCenter(0.0F, 0.0F),
    hasPreviousDetection(false),
    missedFrames(0)
{
  validateParameters();

  LOG_CREATE("SquareDetector");
}

SquareDetector::~SquareDetector()
{
  LOG_DESTROY("SquareDetector");
}

void SquareDetector::detect(
    const cv::Mat& frame,
    BoundingBoxDetectionResult& result)
{
  result.wasDetected = false;

  // =====================================================
  // 入力画像確認
  // =====================================================

  if(frame.empty()) {
    Logger::error(
        "SquareDetector:入力フレームが空です。");

    return;
  }

  // =====================================================
  // ROI設定
  // =====================================================

  const cv::Rect frameRect(
      0,
      0,
      frame.cols,
      frame.rows);

  const cv::Rect roiRect =
      roi & frameRect;

  if(roiRect.empty()) {
    Logger::error(
        "SquareDetector:ROIがフレーム内に収まっていません。");

    return;
  }

  const cv::Mat roiFrame =
      frame(roiRect);

  /*
   * ========================================
   * グレースケール化
   * ========================================
   */

  cv::Mat grayFrame;

  cv::cvtColor(
      roiFrame,
      grayFrame,
      cv::COLOR_BGR2GRAY);

  /*
   * ========================================
   * ノイズ除去
   * ========================================
   */

  cv::GaussianBlur(
      grayFrame,
      grayFrame,
      cv::Size(5, 5),
      0);

  /*
   * ========================================
   * 二値化
   * ========================================
   */

  cv::Mat binary;

  cv::threshold(
      grayFrame,
      binary,
      0,
      255,
      cv::THRESH_BINARY_INV
          | cv::THRESH_OTSU);

  /*
   * ========================================
   * モルフォロジー処理
   * ========================================
   */

  const cv::Mat kernel =
      cv::getStructuringElement(
          cv::MORPH_RECT,
          cv::Size(3, 3));

  cv::morphologyEx(
      binary,
      binary,
      cv::MORPH_CLOSE,
      kernel);

  /*
   * ========================================
   * 輪郭検出
   * ========================================
   */

  std::vector<std::vector<cv::Point>> contours;

  cv::findContours(
      binary,
      contours,
      cv::RETR_EXTERNAL,
      cv::CHAIN_APPROX_SIMPLE);

  Logger::printfLog(
      Logger::INFO,
      "SquareDetector: contour count = %d",
      static_cast<int>(contours.size()));

  /*
   * ========================================
   * 正方形候補を探す
   * ========================================
   */

  std::vector<SquareCandidate> candidates;

  for(const auto& contour : contours) {

    const double area =
        cv::contourArea(contour);

    /*
     * 小さすぎる輪郭はノイズ
     */
    if(area < MIN_CONTOUR_AREA) {
      continue;
    }

    // ===================================================
    // 周長
    // ===================================================

    const double perimeter =
        cv::arcLength(
            contour,
            true);

    if(perimeter <= 0.0) {
      continue;
    }

    /*
     * ========================================
     * 円形度
     *
     * 円       → 1.0に近い
     * 正方形   → 約0.785
     * ========================================
     */

    const double circularity =
        4.0
        * CV_PI
        * area
        / (perimeter * perimeter);

    if(circularity > MAX_CIRCULARITY) {

      Logger::printfLog(
          Logger::DEBUG,
          "SquareDetector: circle rejected "
          "area=%.2f circularity=%.2f",
          area,
          circularity);

      continue;
    }

    /*
     * ========================================
     * 最小外接回転矩形
     * ========================================
     */

    const cv::RotatedRect rect =
        cv::minAreaRect(contour);

    const double width =
        rect.size.width;

    const double height =
        rect.size.height;

    if(width <= 0.0
       || height <= 0.0) {

      continue;
    }

    /*
     * ========================================
     * 縦横比
     *
     * 1.0に近いほど正方形
     * ========================================
     */

    const double ratio =
        std::min(
            width,
            height)
        / std::max(
            width,
            height);

    if(ratio < MIN_RATIO) {
      continue;
    }

    /*
     * ========================================
     * 外接矩形に対する面積割合
     * ========================================
     */

    const double rectArea =
        width * height;

    if(rectArea <= 0.0) {
      continue;
    }

    const double fillRatio =
        area / rectArea;

    if(fillRatio < MIN_FILL_RATIO) {

      Logger::printfLog(
          Logger::DEBUG,
          "SquareDetector: fill rejected "
          "area=%.2f fill=%.2f",
          area,
          fillRatio);

      continue;
    }

    /*
     * ========================================
     * フレーム全体での中心座標
     * ========================================
     */

    const cv::Point2f center(
        rect.center.x
            + static_cast<float>(roiRect.x),
        rect.center.y
            + static_cast<float>(roiRect.y));

    /*
     * ========================================
     * 前回の中心からの距離
     * ========================================
     */

    double distance = 0.0;

    if(hasPreviousDetection) {

      const double dx =
          center.x
          - previousCenter.x;

      const double dy =
          center.y
          - previousCenter.y;

      distance =
          std::sqrt(
              dx * dx
              + dy * dy);

      /*
       * ========================================
       * 前回位置から急激に移動した候補を除外
       * ========================================
       */

      if(distance > MAX_CENTER_MOVE) {

        Logger::printfLog(
            Logger::DEBUG,
            "SquareDetector: "
            "position jump rejected "
            "previous=(%.1f,%.1f) "
            "current=(%.1f,%.1f) "
            "distance=%.1f",
            previousCenter.x,
            previousCenter.y,
            center.x,
            center.y,
            distance);

        continue;
      }
    }

    /*
     * ========================================
     * 正方形候補ログ
     * ========================================
     */

    Logger::printfLog(
        Logger::INFO,
        "SquareDetector: candidate "
        "area=%.2f "
        "width=%.2f "
        "height=%.2f "
        "ratio=%.2f "
        "fill=%.2f "
        "circularity=%.2f "
        "center=(%.1f,%.1f) "
        "distance=%.1f",
        area,
        width,
        height,
        ratio,
        fillRatio,
        circularity,
        center.x,
        center.y,
        distance);

    candidates.push_back(
        SquareCandidate{
            rect,
            area,
            distance
        });
  }

  /*
   * ========================================
   * 候補が存在しない場合
   * ========================================
   */

  if(candidates.empty()) {

    ++missedFrames;

    Logger::warning(
        "SquareDetector: "
        "正方形候補が見つかりませんでした。");

    /*
     * 前回検出位置があり、
     * まだ許容する見失いフレーム数以内なら
     * 前回位置を保持する。
     */
    if(hasPreviousDetection
       && missedFrames <= MAX_MISSED_FRAMES) {

      Logger::printfLog(
          Logger::INFO,
          "SquareDetector: "
          "previous detection is used "
          "center=(%.1f,%.1f) "
          "missed=%d",
          previousCenter.x,
          previousCenter.y,
          missedFrames);

      /*
       * 前回の中心位置だけでは
       * BoundingBoxの4頂点を復元できないため、
       * 今回は検出失敗として返す。
       */
      result.wasDetected = false;

      return;
    }

    /*
     * 一定フレーム以上見失った場合は
     * 追跡状態をリセットする。
     */
    if(missedFrames > MAX_MISSED_FRAMES) {

      hasPreviousDetection = false;

      previousCenter =
          cv::Point2f(
              0.0F,
              0.0F);

      missedFrames = 0;

      Logger::info(
          "SquareDetector: "
          "previous detection reset");
    }

    return;
  }

  /*
   * ========================================
   * 最終候補を決定
   * ========================================
   *
   * 前回検出あり：
   *
   *   前回の中心位置に
   *   最も近い候補を選択する。
   *
   * 初回検出：
   *
   *   1. 最大面積を取得
   *   2. 最大面積の70%以上に絞る
   *   3. 画像中央下部に最も近い候補を選択
   *
   * これにより、
   * 単純に面積最大の正方形ではなく、
   * ロボットの正面にあるゲートの正方形を
   * 優先して選択する。
   */

  SquareCandidate bestCandidate =
      candidates.front();

  // =====================================================
  // 追跡中
  // =====================================================

  if(hasPreviousDetection) {

    for(const auto& candidate : candidates) {

      if(candidate.distance
         < bestCandidate.distance) {

        bestCandidate =
            candidate;
      }
    }

  // =====================================================
  // 初回検出
  // =====================================================

  } else {

    // ---------------------------------------------------
    // 候補中の最大面積を取得
    // ---------------------------------------------------

    double maxArea = 0.0;

    for(const auto& candidate : candidates) {

      if(candidate.area > maxArea) {

        maxArea =
            candidate.area;
      }
    }

    // ---------------------------------------------------
    // 小さすぎる候補を除外するための最低面積
    // ---------------------------------------------------

    const double minimumArea =
        maxArea
        * INITIAL_AREA_RATIO;

    /*
     * 初回検出時の基準位置。
     *
     * X:
     *   画像中央
     *
     * Y:
     *   画像高さの82%
     *
     * 1920x1080の場合
     *
     * targetX = 960
     * targetY = 約886
     */
    const double targetX =
        CAM_MAX_WIDTH
        / 2.0;

    const double targetY =
        CAM_MAX_HEIGHT
        * INITIAL_TARGET_Y_RATIO;

    double bestTargetDistance =
        std::numeric_limits<double>::max();

    bool foundLargeCandidate =
        false;

    for(const auto& candidate : candidates) {

      // -----------------------------------------------
      // 面積が小さい候補は除外
      // -----------------------------------------------

      if(candidate.area
         < minimumArea) {

        Logger::printfLog(
            Logger::DEBUG,
            "SquareDetector: "
            "initial candidate area rejected "
            "area=%.2f minimumArea=%.2f",
            candidate.area,
            minimumArea);

        continue;
      }

      // -----------------------------------------------
      // フレーム全体での中心位置
      // -----------------------------------------------

      const double centerX =
          candidate.rect.center.x
          + static_cast<double>(
              roiRect.x);

      const double centerY =
          candidate.rect.center.y
          + static_cast<double>(
              roiRect.y);

      // -----------------------------------------------
      // 基準位置との距離
      // -----------------------------------------------

      const double dx =
          centerX
          - targetX;

      const double dy =
          centerY
          - targetY;

      const double targetDistance =
          std::sqrt(
              dx * dx
              + dy * dy);

      Logger::printfLog(
          Logger::INFO,
          "SquareDetector: "
          "initial candidate "
          "area=%.2f "
          "center=(%.1f,%.1f) "
          "target=(%.1f,%.1f) "
          "targetDistance=%.1f",
          candidate.area,
          centerX,
          centerY,
          targetX,
          targetY,
          targetDistance);

      // -----------------------------------------------
      // 基準位置に最も近い候補を選択
      // -----------------------------------------------

      if(!foundLargeCandidate
         || targetDistance
                < bestTargetDistance) {

        bestCandidate =
            candidate;

        bestTargetDistance =
            targetDistance;

        foundLargeCandidate =
            true;
      }
    }

    /*
     * 通常は最大面積の候補自身が
     * minimumArea以上なので必ず見つかる。
     *
     * 念のため見つからなかった場合は、
     * 従来と同じ面積最大候補を使用する。
     */
    if(!foundLargeCandidate) {

      Logger::warning(
          "SquareDetector: "
          "initial target candidate not found "
          "-> use largest area");

      bestCandidate =
          candidates.front();

      for(const auto& candidate : candidates) {

        if(candidate.area
           > bestCandidate.area) {

          bestCandidate =
              candidate;
        }
      }
    }
  }

  // =====================================================
  // 選択された正方形
  // =====================================================

  const cv::RotatedRect bestRect =
      bestCandidate.rect;

  /*
   * ========================================
   * 正方形の中心位置を保存
   * ========================================
   */

  const cv::Point2f currentCenter(
      bestRect.center.x
          + static_cast<float>(
              roiRect.x),
      bestRect.center.y
          + static_cast<float>(
              roiRect.y));

  previousCenter =
      currentCenter;

  hasPreviousDetection =
      true;

  missedFrames =
      0;

  /*
   * ========================================
   * 4つの頂点を取得
   * ========================================
   */

  cv::Point2f rectPoints[4];

  bestRect.points(
      rectPoints);

  std::vector<cv::Point> corners;

  corners.reserve(4);

  for(int i = 0;
      i < 4;
      ++i) {

    corners.emplace_back(
        static_cast<int>(
            rectPoints[i].x)
            + roiRect.x,
        static_cast<int>(
            rectPoints[i].y)
            + roiRect.y);
  }

  /*
   * ========================================
   * 4頂点を
   *
   * TL / TR / BR / BL
   *
   * に分類
   * ========================================
   */

  cv::Point topLeft;
  cv::Point topRight;
  cv::Point bottomLeft;
  cv::Point bottomRight;

  int minSum =
      INT_MAX;

  int maxSum =
      INT_MIN;

  int minDiff =
      INT_MAX;

  int maxDiff =
      INT_MIN;

  for(const auto& point : corners) {

    const int sum =
        point.x
        + point.y;

    const int diff =
        point.x
        - point.y;

    /*
     * x + y が最小
     * → 左上
     */
    if(sum < minSum) {

      minSum =
          sum;

      topLeft =
          point;
    }

    /*
     * x + y が最大
     * → 右下
     */
    if(sum > maxSum) {

      maxSum =
          sum;

      bottomRight =
          point;
    }

    /*
     * x - y が最大
     * → 右上
     */
    if(diff > maxDiff) {

      maxDiff =
          diff;

      topRight =
          point;
    }

    /*
     * x - y が最小
     * → 左下
     */
    if(diff < minDiff) {

      minDiff =
          diff;

      bottomLeft =
          point;
    }
  }

  /*
   * ========================================
   * 検出結果を設定
   * ========================================
   */

  result.topLeft =
      topLeft;

  result.topRight =
      topRight;

  result.bottomRight =
      bottomRight;

  result.bottomLeft =
      bottomLeft;

  result.wasDetected =
      true;

  /*
   * ========================================
   * ログ
   * ========================================
   */

  Logger::printfLog(
      Logger::INFO,
      "SquareDetector: "
      "===== SQUARE DETECTED ===== "
      "area=%.2f "
      "center=(%.1f,%.1f) "
      "distance=%.1f",
      bestCandidate.area,
      currentCenter.x,
      currentCenter.y,
      bestCandidate.distance);

  Logger::printfLog(
      Logger::INFO,
      "SquareDetector: "
      "TL=(%d,%d) "
      "TR=(%d,%d) "
      "BR=(%d,%d) "
      "BL=(%d,%d)",
      result.topLeft.x,
      result.topLeft.y,
      result.topRight.x,
      result.topRight.y,
      result.bottomRight.x,
      result.bottomRight.y,
      result.bottomLeft.x,
      result.bottomLeft.y);
}

void SquareDetector::setValidatedRoi(
    const cv::Rect& _roi)
{
  roi =
      _roi;

  validateParameters();
}

void SquareDetector::validateParameters()
{
  /*
   * ========================================
   * X座標
   * ========================================
   */

  if(roi.x < 0) {

    roi.x =
        0;
  }

  if(roi.x > CAM_MAX_WIDTH) {

    roi.x =
        CAM_MAX_WIDTH;
  }

  /*
   * ========================================
   * Y座標
   * ========================================
   */

  if(roi.y < 0) {

    roi.y =
        0;
  }

  if(roi.y > CAM_MAX_HEIGHT) {

    roi.y =
        CAM_MAX_HEIGHT;
  }

  /*
   * ========================================
   * 幅
   * ========================================
   */

  if(roi.width < 0) {

    roi.width =
        0;
  }

  if(roi.width
     > CAM_MAX_WIDTH - roi.x) {

    roi.width =
        CAM_MAX_WIDTH
        - roi.x;
  }

  /*
   * ========================================
   * 高さ
   * ========================================
   */

  if(roi.height < 0) {

    roi.height =
        0;
  }

  /*
   * ROIが画像下端を超える場合は、
   * 画像内に収まる高さへ補正する。
   */
  if(roi.height
     > CAM_MAX_HEIGHT - roi.y) {

    roi.height =
        CAM_MAX_HEIGHT
        - roi.y;
  }
}