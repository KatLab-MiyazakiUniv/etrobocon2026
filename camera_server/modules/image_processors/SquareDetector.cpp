/**
 * @file   SquareDetector.cpp
 * @brief  正方形検出用の画像処理クラス
 * @author okuyama0528 yutaro-1214
 */

#include "SquareDetector.h"

namespace {

  /**
   * @brief 正方形候補
   */
  struct SquareCandidate {
    cv::RotatedRect rect;
    double area;
  };

  /**
   * @brief 正方形を探す基準Y位置
   */
  constexpr double TARGET_Y_RATIO = 0.82;

  /**
   * @brief 画像端から除外する幅[px]
   */
  constexpr int BORDER_MARGIN = 20;

  /**
   * @brief 基準位置から許容する最大距離[px]
   */
  constexpr double MAX_TARGET_DISTANCE = 700.0;

  /**
   * @brief Canny下限閾値
   */
  constexpr double CANNY_THRESHOLD_LOW = 50.0;

  /**
   * @brief Canny上限閾値
   */
  constexpr double CANNY_THRESHOLD_HIGH = 150.0;

  /**
   * @brief モルフォロジー処理のカーネルサイズ
   */
  constexpr int MORPH_KERNEL_SIZE = 7;

  /**
   * @brief デバッグ画像保存用カウンタ
   */
  int saveCount = 0;

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
  result.wasDetected = false;

  ++saveCount;

  // 入力画像確認
  if(frame.empty()) {
    Logger::error(
        "SquareDetector:入力フレームが空です。");

    return;
  }

  // ROI
  const cv::Rect frameRect(
      0,
      0,
      frame.cols,
      frame.rows);

  const cv::Rect roiRect
      = roi & frameRect;

  if(roiRect.empty()) {
    Logger::error(
        "SquareDetector:"
        "ROIがフレーム内に収まっていません。");

    return;
  }

  const cv::Mat roiFrame
      = frame(roiRect);

  // =====================================================
  // 01 Original
  // =====================================================

  const std::string originalPath
      = "/tmp/square_"
        + std::to_string(saveCount)
        + "_01_original.jpg";

  cv::imwrite(
      originalPath,
      roiFrame);

  Logger::printfLog(
      Logger::INFO,
      "SquareDetector: original image saved: %s",
      originalPath.c_str());

  // =====================================================
  // Gray
  // =====================================================

  cv::Mat grayFrame;

  cv::cvtColor(
      roiFrame,
      grayFrame,
      cv::COLOR_BGR2GRAY);

  //デバッグ画像
  const std::string grayPath
      = "/tmp/square_"
        + std::to_string(saveCount)
        + "_02_gray.jpg";

  cv::imwrite(
      grayPath,
      grayFrame);

  Logger::printfLog(
      Logger::INFO,
      "SquareDetector: gray image saved: %s",
      grayPath.c_str());

  // GaussianBlur
  cv::Mat blurFrame;

  cv::GaussianBlur(
      grayFrame,
      blurFrame,
      cv::Size(5, 5),
      0);

  //Blur

  const std::string blurPath
      = "/tmp/square_"
        + std::to_string(saveCount)
        + "_03_blur.jpg";

  cv::imwrite(
      blurPath,
      blurFrame);

  Logger::printfLog(
      Logger::INFO,
      "SquareDetector: blur image saved: %s",
      blurPath.c_str());

  // Canny
  cv::Mat edgeFrame;

  cv::Canny(
      blurFrame,
      edgeFrame,
      CANNY_THRESHOLD_LOW,
      CANNY_THRESHOLD_HIGH);

  // 04 Canny
  const std::string cannyPath
      = "/tmp/square_"
        + std::to_string(saveCount)
        + "_04_canny.jpg";

  cv::imwrite(
      cannyPath,
      edgeFrame);

  Logger::printfLog(
      Logger::INFO,
      "SquareDetector: canny image saved: %s",
      cannyPath.c_str());

  // =====================================================
  // Morphology
  //
  // QR内部の細かい線をまとめるために
  // MORPH_CLOSEを行う。
  // =====================================================

  cv::Mat morphFrame;

  const cv::Mat kernel
      = cv::getStructuringElement(
          cv::MORPH_RECT,
          cv::Size(
              MORPH_KERNEL_SIZE,
              MORPH_KERNEL_SIZE));

  cv::morphologyEx(
      edgeFrame,
      morphFrame,
      cv::MORPH_CLOSE,
      kernel);

  //Morphology

  const std::string morphPath
      = "/tmp/square_"
        + std::to_string(saveCount)
        + "_05_morphology.jpg";

  cv::imwrite(
      morphPath,
      morphFrame);

  Logger::printfLog(
      Logger::INFO,
      "SquareDetector: morphology image saved: %s",
      morphPath.c_str());

  // =====================================================
  // 輪郭検出
  // =====================================================

  std::vector<std::vector<cv::Point>> contours;

  cv::findContours(
      morphFrame,
      contours,
      cv::RETR_EXTERNAL,
      cv::CHAIN_APPROX_SIMPLE);

  Logger::printfLog(
      Logger::INFO,
      "SquareDetector: contour count = %d",
      static_cast<int>(contours.size()));

  // 候補抽出
  std::vector<SquareCandidate> candidates;

  for(const auto& contour : contours) {
    // 面積
    const double area
        = cv::contourArea(contour);

    if(area < MIN_CONTOUR_AREA) {
      Logger::printfLog(
          Logger::DEBUG,
          "SquareDetector: "
          "small area rejected "
          "area=%.2f minimum=%.2f",
          area,
          MIN_CONTOUR_AREA);

      continue;
    }

    // 周長
    const double perimeter
        = cv::arcLength(
            contour,
            true);

    if(perimeter <= 0.0) {
      continue;
    }

    // 円形度
    const double circularity
        = 4.0
          * CV_PI
          * area
          / (perimeter * perimeter);

    if(circularity > MAX_CIRCULARITY) {
      Logger::printfLog(
          Logger::DEBUG,
          "SquareDetector: "
          "circle rejected "
          "area=%.2f circularity=%.2f",
          area,
          circularity);

      continue;
    }

    // 最小外接回転矩形
    const cv::RotatedRect rect
        = cv::minAreaRect(contour);

    const double width
        = rect.size.width;

    const double height
        = rect.size.height;

    if(width <= 0.0
       || height <= 0.0) {

      continue;
    }

    // 中心位置
    const double centerX
        = rect.center.x
          + static_cast<double>(
              roiRect.x);

    const double centerY
        = rect.center.y
          + static_cast<double>(
              roiRect.y);

    // 画像端判定
    const cv::Rect boundingRect
        = rect.boundingRect();

    const int globalLeft
        = boundingRect.x
          + roiRect.x;

    const int globalTop
        = boundingRect.y
          + roiRect.y;

    const int globalRight
        = globalLeft
          + boundingRect.width;

    const int globalBottom
        = globalTop
          + boundingRect.height;

    if(globalLeft <= BORDER_MARGIN
       || globalTop <= BORDER_MARGIN
       || globalRight >= frame.cols - BORDER_MARGIN
       || globalBottom >= frame.rows - BORDER_MARGIN) {

      Logger::printfLog(
          Logger::DEBUG,
          "SquareDetector: "
          "border rejected "
          "area=%.2f "
          "left=%d top=%d "
          "right=%d bottom=%d",
          area,
          globalLeft,
          globalTop,
          globalRight,
          globalBottom);

      continue;
    }

    // ---------------------------------------------------
    // 縦横比
    // ---------------------------------------------------

    const double ratio
        = std::min(
              width,
              height)
          / std::max(
              width,
              height);

    if(ratio < MIN_RATIO) {
      Logger::printfLog(
          Logger::DEBUG,
          "SquareDetector: "
          "ratio rejected "
          "area=%.2f "
          "width=%.2f "
          "height=%.2f "
          "ratio=%.2f",
          area,
          width,
          height,
          ratio);

      continue;
    }

    // 矩形面積
    const double rectArea
        = width * height;

    if(rectArea <= 0.0) {
      continue;
    }

    // 充填率
    const double fillRatio
        = area / rectArea;

    if(fillRatio < MIN_FILL_RATIO) {
      Logger::printfLog(
          Logger::DEBUG,
          "SquareDetector: "
          "fill rejected "
          "area=%.2f fill=%.2f",
          area,
          fillRatio);

      continue;
    }

    // 有効候補
    Logger::printfLog(
        Logger::INFO,
        "SquareDetector: candidate "
        "area=%.2f "
        "width=%.2f "
        "height=%.2f "
        "ratio=%.2f "
        "fill=%.2f "
        "circularity=%.2f "
        "center=(%.1f,%.1f)",
        area,
        width,
        height,
        ratio,
        fillRatio,
        circularity,
        centerX,
        centerY);

    candidates.push_back(
        SquareCandidate{
            rect,
            area
        });
  }

  // =====================================================
  // 候補なし
  // =====================================================

  if(candidates.empty()) {
    Logger::warning(
        "SquareDetector:"
        "正方形候補が見つかりませんでした。");

    return;
  }

  // =====================================================
  // 基準位置
  //
  // 最大面積は使用しない。
  //
  // 条件を満たした候補の中から、
  // 画像中央下側の基準位置に
  // 一番近い候補を選択する。
  // =====================================================

  const double targetX
      = CAM_MAX_WIDTH
        / 2.0;

  const double targetY
      = CAM_MAX_HEIGHT
        * TARGET_Y_RATIO;

  double bestTargetDistance
      = std::numeric_limits<double>::max();

  bool foundCandidate
      = false;

  SquareCandidate bestCandidate{};

  // 最終候補選択
  for(const auto& candidate
      : candidates) {

    const double centerX
        = candidate.rect.center.x
          + static_cast<double>(
              roiRect.x);

    const double centerY
        = candidate.rect.center.y
          + static_cast<double>(
              roiRect.y);

    const double dx
        = centerX
          - targetX;

    const double dy
        = centerY
          - targetY;

    const double targetDistance
        = std::sqrt(
            dx * dx
            + dy * dy);

    Logger::printfLog(
        Logger::INFO,
        "SquareDetector: "
        "selection candidate "
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

    // 遠すぎる候補は除外
    if(targetDistance
       > MAX_TARGET_DISTANCE) {

      Logger::printfLog(
          Logger::DEBUG,
          "SquareDetector: "
          "target distance rejected "
          "area=%.2f "
          "distance=%.1f "
          "maxDistance=%.1f",
          candidate.area,
          targetDistance,
          MAX_TARGET_DISTANCE);

      continue;
    }

    // 基準位置に最も近い候補
    if(!foundCandidate
       || targetDistance
              < bestTargetDistance) {

      bestCandidate
          = candidate;

      bestTargetDistance
          = targetDistance;

      foundCandidate
          = true;
    }
  }

  // 有効候補なし
  if(!foundCandidate) {
    Logger::warning(
        "SquareDetector:"
        "条件を満たす正方形候補がありません。");

    return;
  }

  // 選択候補
  const cv::RotatedRect bestRect
      = bestCandidate.rect;

  const cv::Point2f currentCenter(
      bestRect.center.x
          + static_cast<float>(
              roiRect.x),
      bestRect.center.y
          + static_cast<float>(
              roiRect.y));

  // 4頂点取得
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

  cv::Point topLeft;
  cv::Point topRight;
  cv::Point bottomLeft;
  cv::Point bottomRight;

  int minSum
      = INT_MAX;
  int maxSum
      = INT_MIN;
  int minDiff
      = INT_MAX;
  int maxDiff
      = INT_MIN;

  for(const auto& point
      : corners) {

    const int sum
        = point.x
          + point.y;

    const int diff
        = point.x
          - point.y;

    if(sum < minSum) {
      minSum
          = sum;

      topLeft
          = point;
    }

    if(sum > maxSum) {
      maxSum
          = sum;

      bottomRight
          = point;
    }

    if(diff > maxDiff) {
      maxDiff
          = diff;

      topRight
          = point;
    }

    if(diff < minDiff) {
      minDiff
          = diff;

      bottomLeft
          = point;
    }
  }

  result.topLeft
      = topLeft;
  result.topRight
      = topRight;
  result.bottomRight
      = bottomRight;
  result.bottomLeft
      = bottomLeft;
  result.wasDetected
      = true;

  // ログ
Logger::printfLog(
    Logger::INFO,
    "SquareDetector: "
    "正方形を検出しました"
    "面積=%.2f "
    "中心=(%.1f,%.1f) "
    "目標距離=%.1f",
    bestCandidate.area,
    currentCenter.x,
    currentCenter.y,
    bestTargetDistance);

  Logger::printfLog(
    Logger::INFO,
    "SquareDetector: "
    "左上=(%d,%d) "
    "右上=(%d,%d) "
    "右下=(%d,%d) "
    "左下=(%d,%d)",
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
  if(roi.x < 0) {
    roi.x = 0;
  }

  if(roi.x > CAM_MAX_WIDTH) {
    roi.x = CAM_MAX_WIDTH;
  }

  if(roi.y < 0) {
    roi.y = 0;
  }

  if(roi.y > CAM_MAX_HEIGHT) {
    roi.y = CAM_MAX_HEIGHT;
  }

  if(roi.width < 0) {
    roi.width = 0;
  }

  if(roi.width
     > CAM_MAX_WIDTH - roi.x) {

    roi.width
        = CAM_MAX_WIDTH
          - roi.x;
  }

  if(roi.height < 0) {
    roi.height = 0;
  }

  if(roi.height
     > CAM_MAX_HEIGHT - roi.y) {

    roi.height
        = CAM_MAX_HEIGHT
          - roi.y;
  }
}