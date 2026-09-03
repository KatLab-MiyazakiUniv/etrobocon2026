/*
 * @file SquareDetector.cpp
 * @brief 正方形検出用の画像処理クラス
 * @author okuyama0528
 */

#include "SquareDetector.h"

SquareDetector::SquareDetector(const cv::Rect& _roi) : roi(_roi)
{
  validateParameters();
  LOG_CREATE("SquareDetector");
}

SquareDetector::~SquareDetector()
{
  LOG_DESTROY("SquareDetector");
}

void SquareDetector::detect(const cv::Mat& frame, BoundingBoxDetectionResult& result)
{
  result.wasDetected = false;

  // 入力フレームが空の場合は処理を中断する
  if(frame.empty()) {
    Logger::error("SquareDetector:入力フレームが空です。");
    return;
  }

  // ROI切り出し
  cv::Rect roiRect = roi & cv::Rect(0, 0, frame.cols, frame.rows);

  if(roiRect.empty()) {
    Logger::error("SquareDetector:ROIがフレーム内に収まっていません。");
    return;
  }

  cv::Mat roiFrame = frame(roiRect);

  // グレースケール変換
  cv::Mat grayFrame;
  cv::cvtColor(roiFrame, grayFrame, cv::COLOR_BGR2GRAY);

  // 二値化
  cv::Mat binary;
  cv::threshold(grayFrame, binary, 80, 255, cv::THRESH_BINARY_INV);

  // 輪郭検出
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  // 最も正方形に近い輪郭を保存
  double bestRatio = 0.0;
  std::vector<cv::Point> bestSquare;

  for(const auto& contour : contours) {
    // 小さい領域は無視
    double area = cv::contourArea(contour);

    if(area < MIN_CONTOUR_AREA) {
      continue;
    }

    // 輪郭を多角形に近似
    std::vector<cv::Point> approx;
    double perimeter = cv::arcLength(contour, true);

    cv::approxPolyDP(contour, approx, 0.02 * perimeter, true);

    // 4頂点でなければ無視
    if(approx.size() != 4) {
      continue;
    }

    // 凸四角形でなければ無視
    if(!cv::isContourConvex(approx)) {
      continue;
    }

    // 回転を考慮した外接矩形を取得
    cv::RotatedRect rect = cv::minAreaRect(approx);

    double width = rect.size.width;
    double height = rect.size.height;

    if(width <= 0.0 || height <= 0.0) {
      continue;
    }

    // 正方形らしさを計算
    // 1.0に近いほど正方形
    double ratio = std::min(width, height) / std::max(width, height);

    // 正方形に近いものだけ候補にする
    if(ratio < 0.8) {
      continue;
    }

    // 最も正方形に近いものを選択
    if(ratio > bestRatio) {
      bestRatio = ratio;
      bestSquare = approx;
    }
  }
  // 正方形が見つからなかった
  if(bestSquare.empty()) {
    Logger::error("SquareDetector:正方形が見つかりませんでした。");
    return;
  }

  // ROI内の座標をフレーム全体の座標に変換
  std::vector<cv::Point> corners;

  for(const auto& point : bestSquare) {
    corners.emplace_back(
        point.x + roiRect.x,
        point.y + roiRect.y);
  }

  // 左上・右上・右下・左下に分類
  cv::Point topLeft;
  cv::Point topRight;
  cv::Point bottomLeft;
  cv::Point bottomRight;

  int minSum = INT_MAX;
  int maxSum = INT_MIN;
  int minDiff = INT_MAX;
  int maxDiff = INT_MIN;

  for(const auto& point : corners) {
    int sum = point.x + point.y;
    int diff = point.x - point.y;

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

  result.topLeft = topLeft;
  result.topRight = topRight;
  result.bottomLeft = bottomLeft;
  result.bottomRight = bottomRight;

  result.wasDetected = true;
}

void SquareDetector::setValidatedRoi(const cv::Rect& _roi)
{
  roi = _roi;
  validateParameters();
}

void SquareDetector::validateParameters()
{
  if(roi.x < 0) roi.x = 0;
  if(roi.y < 0) roi.y = 0;
  if(roi.x > CAM_MAX_WIDTH) roi.x = CAM_MAX_WIDTH;
  if(roi.y > CAM_MAX_HEIGHT) roi.y = CAM_MAX_HEIGHT;
  if(roi.width < 0) roi.width = 0;
  if(roi.height < 0) roi.height = 0;
  if(roi.width > CAM_MAX_WIDTH - roi.x) roi.width = CAM_MAX_WIDTH - roi.x;
  if(roi.height > CAM_MAX_HEIGHT - roi.y) roi.height = CAM_MAX_HEIGHT - roi.y;
}