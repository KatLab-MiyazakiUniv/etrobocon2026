/**
 * @file SquareDetector.cpp
 * @brief 正方形検出用の画像処理クラス
 * @author okuyama0528
 */

#include "SquareDetector.h"

#include <algorithm>
#include <climits>

namespace {

  /**
   * @brief 検出対象とする最小輪郭面積
   */
  constexpr double MIN_AREA = 100.0;

  /**
   * @brief 正方形らしさの最小値
   *
   * min(width, height) / max(width, height)
   *
   * 1.0に近いほど正方形。
   */
  constexpr double MIN_RATIO = 0.6;

  /**
   * @brief 外接矩形に対する輪郭面積の最小割合
   *
   * 正方形は1.0に近く、
   * 円は理論上およそ0.785になる。
   */
  constexpr double MIN_FILL_RATIO = 0.82;

  /**
   * @brief 円形度の最大値
   *
   * circularity =
   * 4 * PI * area / perimeter^2
   *
   * 円は1.0に近い。
   * 正方形は理論上およそ0.785。
   */
  constexpr double MAX_CIRCULARITY = 0.88;

}  // namespace

SquareDetector::SquareDetector(
    const cv::Rect& _roi)
  : roi(_roi)
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
  // =====================================================
  // 初期化
  // =====================================================

  result.wasDetected = false;

  // =====================================================
  // 入力確認
  // =====================================================

  if(frame.empty()) {

    Logger::error(
        "SquareDetector:入力フレームが空です。");

    return;
  }

  // =====================================================
  // ROI
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

  // =====================================================
  // グレースケール変換
  // =====================================================

  cv::Mat grayFrame;

  cv::cvtColor(
      roiFrame,
      grayFrame,
      cv::COLOR_BGR2GRAY);

  // =====================================================
  // ノイズ除去
  // =====================================================

  /*
   * 低解像度で正方形の輪郭が
   * ギザギザになる場合に備えてぼかす。
   */
  cv::GaussianBlur(
      grayFrame,
      grayFrame,
      cv::Size(5, 5),
      0);

  // =====================================================
  // 二値化
  // =====================================================

  cv::Mat binary;

  /*
   * OTSUを使用して、
   * 周囲の明るさが変化しても対応しやすくする。
   */
  cv::threshold(
      grayFrame,
      binary,
      0,
      255,
      cv::THRESH_BINARY_INV
          | cv::THRESH_OTSU);

  // =====================================================
  // モルフォロジー処理
  // =====================================================

  /*
   * 輪郭の小さな切れ目を補完する。
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

  // =====================================================
  // 輪郭検出
  // =====================================================

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

  // =====================================================
  // 最良候補
  // =====================================================

  bool found = false;

  double bestArea = 0.0;

  cv::RotatedRect bestRect;

  // =====================================================
  // 各輪郭を確認
  // =====================================================

  for(const auto& contour : contours) {

    // -----------------------------------------------------
    // 面積
    // -----------------------------------------------------

    const double area =
        cv::contourArea(contour);

    if(area < MIN_AREA) {
      continue;
    }

    // -----------------------------------------------------
    // 周囲長
    // -----------------------------------------------------

    const double perimeter =
        cv::arcLength(
            contour,
            true);

    if(perimeter <= 0.0) {
      continue;
    }

    // -----------------------------------------------------
    // 円形度
    // -----------------------------------------------------

    /*
     * 円:
     *   circularity ≒ 1.0
     *
     * 正方形:
     *   circularity ≒ 0.785
     */
    const double circularity =
        4.0
        * CV_PI
        * area
        / (perimeter * perimeter);

    /*
     * 円に近すぎるものを除外する。
     */
    if(circularity > MAX_CIRCULARITY) {

      Logger::printfLog(
          Logger::DEBUG,
          "SquareDetector: circle rejected "
          "area=%.2f circularity=%.2f",
          area,
          circularity);

      continue;
    }

    // -----------------------------------------------------
    // 回転外接矩形
    // -----------------------------------------------------

    /*
     * approxPolyDPによる4頂点固定は行わない。
     *
     * 低解像度で輪郭が多少崩れていても、
     * minAreaRectで矩形として評価する。
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

    // -----------------------------------------------------
    // 縦横比
    // -----------------------------------------------------

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

    // -----------------------------------------------------
    // 外接矩形面積
    // -----------------------------------------------------

    const double rectArea =
        width * height;

    if(rectArea <= 0.0) {
      continue;
    }

    // -----------------------------------------------------
    // 塗りつぶし率
    // -----------------------------------------------------

    /*
     * 正方形なら1.0に近くなる。
     *
     * 円は外接矩形に対して
     * およそ0.785程度になる。
     */
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

    // -----------------------------------------------------
    // 候補ログ
    // -----------------------------------------------------

    Logger::printfLog(
        Logger::INFO,
        "SquareDetector: candidate "
        "area=%.2f "
        "width=%.2f "
        "height=%.2f "
        "ratio=%.2f "
        "fill=%.2f "
        "circularity=%.2f",
        area,
        width,
        height,
        ratio,
        fillRatio,
        circularity);

    // -----------------------------------------------------
    // 最も大きい候補を選択
    // -----------------------------------------------------

    /*
     * 条件を満たした候補の中から、
     * 最も大きいものを採用する。
     */
    if(!found
       || area > bestArea) {

      bestArea = area;

      bestRect = rect;

      found = true;
    }
  }

  // =====================================================
  // 正方形候補なし
  // =====================================================

  if(!found) {

    Logger::warning(
        "SquareDetector:正方形が見つかりませんでした。");

    return;
  }

  // =====================================================
  // 4頂点取得
  // =====================================================

  cv::Point2f rectPoints[4];

  bestRect.points(
      rectPoints);

  std::vector<cv::Point> corners;

  corners.reserve(4);

  for(int i = 0;
      i < 4;
      ++i) {

    /*
     * ROI内座標から
     * 元画像全体の座標へ変換。
     */
    corners.emplace_back(
        static_cast<int>(
            rectPoints[i].x)
            + roiRect.x,
        static_cast<int>(
            rectPoints[i].y)
            + roiRect.y);
  }

  // =====================================================
  // 頂点分類
  // =====================================================

  cv::Point topLeft;
  cv::Point topRight;
  cv::Point bottomLeft;
  cv::Point bottomRight;

  int minSum = INT_MAX;
  int maxSum = INT_MIN;

  int minDiff = INT_MAX;
  int maxDiff = INT_MIN;

  /*
   * x + y
   *
   * 最小 → 左上
   * 最大 → 右下
   *
   * x - y
   *
   * 最大 → 右上
   * 最小 → 左下
   */
  for(const auto& point : corners) {

    const int sum =
        point.x
        + point.y;

    const int diff =
        point.x
        - point.y;

    if(sum < minSum) {

      minSum = sum;

      topLeft = point;
    }

    if(sum > maxSum) {

      maxSum = sum;

      bottomRight = point;
    }

    if(diff > maxDiff) {

      maxDiff = diff;

      topRight = point;
    }

    if(diff < minDiff) {

      minDiff = diff;

      bottomLeft = point;
    }
  }

  // =====================================================
  // 検出結果
  // =====================================================

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

  // =====================================================
  // 検出ログ
  // =====================================================

  Logger::printfLog(
      Logger::INFO,
      "SquareDetector: "
      "===== SQUARE DETECTED ===== "
      "area=%.2f",
      bestArea);

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
  roi = _roi;

  validateParameters();
}

void SquareDetector::validateParameters()
{
  // =====================================================
  // X
  // =====================================================

  if(roi.x < 0) {
    roi.x = 0;
  }

  if(roi.x > CAM_MAX_WIDTH) {
    roi.x = CAM_MAX_WIDTH;
  }

  // =====================================================
  // Y
  // =====================================================

  if(roi.y < 0) {
    roi.y = 0;
  }

  if(roi.y > CAM_MAX_HEIGHT) {
    roi.y = CAM_MAX_HEIGHT;
  }

  // =====================================================
  // Width
  // =====================================================

  if(roi.width < 0) {
    roi.width = 0;
  }

  if(roi.width
     > CAM_MAX_WIDTH - roi.x) {

    roi.width =
        CAM_MAX_WIDTH - roi.x;
  }

  // =====================================================
  // Height
  // =====================================================

  if(roi.height < 0) {
    roi.height = 0;
  }

  if(roi.height
     > CAM_MAX_HEIGHT - roi.y) {

    roi.height =
        CAM_MAX_HEIGHT - roi.y;
  }
}