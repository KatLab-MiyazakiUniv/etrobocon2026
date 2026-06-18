/*
 * @file ColorRegionDetector.cpp
 * @brief 色領域検出用の画像処理クラス
 * @author okuyama0528, sadomiya-sousi
 */

#include "ColorRegionDetector.h"

ColorRegionDetector::ColorRegionDetector(
    const std::vector<ColorRegionDetector::HSVRange>& _hsvRanges, const cv::Rect& _roi)
  : hsvRanges(_hsvRanges), roi(_roi)
{
  validateParameters();
  LOG_CREATE("ColorRegionDetector");
}

ColorRegionDetector::~ColorRegionDetector()
{
  LOG_DESTROY("ColorRegionDetector");
}

const std::vector<ColorRegionDetector::HSVRange>& ColorRegionDetector::getHsvRanges() const
{
  return hsvRanges;
}

void ColorRegionDetector::setHsvRanges(const std::vector<ColorRegionDetector::HSVRange>& _hsvRanges)
{
  hsvRanges = _hsvRanges;
}

const cv::Rect& ColorRegionDetector::getRoi() const
{
  return roi;
}

void ColorRegionDetector::setRoi(const cv::Rect& _roi)
{
  roi = _roi;
  validateParameters();
}

void ColorRegionDetector::validateParameters()
{
  if(roi.x < 0) roi.x = 0;
  if(roi.y < 0) roi.y = 0;
  if(roi.x + roi.width > CAM_MAX_WIDTH) roi.width = CAM_MAX_WIDTH - roi.x;
  if(roi.y + roi.height > CAM_MAX_HEIGHT) roi.height = CAM_MAX_HEIGHT - roi.y;
}

void ColorRegionDetector::detect(const cv::Mat& frame, BoundingBoxDetectionResult& result)
{
  cv::Mat hsvFrame;
  cv::Rect boundingBox;

  // 最大色インデックスが不要な場合は、バウンディングボックス検出のみ行う
  if(!detectBoundingBox(frame, result, hsvFrame, boundingBox)) {
    Logger::error("ColorRegionDetector:色領域のバウンディングボックスの検出に失敗");
    return;
  }
}

void ColorRegionDetector::detect(const cv::Mat& frame, BoundingBoxDetectionResult& result,
                                 int32_t& largestColorIndex)
{
  largestColorIndex = -1;
  cv::Mat hsvFrame;
  cv::Rect boundingBox;

  // 複数色を統合したマスクからバウンディングボックスを検出する
  if(!detectBoundingBox(frame, result, hsvFrame, boundingBox)) {
    Logger::error("ColorRegionDetector:色領域のバウンディングボックスの検出に失敗");
    return;
  }
  // 検出したバウンディングボックス内で最も面積が大きい色のインデックスを取得する
  largestColorIndex = findLargestColorIndex(hsvFrame, boundingBox);
}

bool ColorRegionDetector::detectBoundingBox(const cv::Mat& frame,
                                            BoundingBoxDetectionResult& result, cv::Mat& hsvFrame,
                                            cv::Rect& boundingBox)
{
  result.wasDetected = false;

  // 入力フレームが空の場合は処理を中断する
  if(frame.empty()) {
    Logger::error("入力フレームが空です。");
    return false;
  }

  cv::Mat frameProcessed;
  frameProcessed = frame.clone();

  // ROI切り出し
  cv::Rect roiRect = roi & cv::Rect(0, 0, frameProcessed.cols, frameProcessed.rows);

  if(roiRect.empty()) {
    Logger::error("ROIがフレーム内に収まっていません。");
    return false;
  }

  cv::Mat roiFrame = frameProcessed(roiRect);

  // RGBを HSVに変換
  cv::cvtColor(roiFrame, hsvFrame, cv::COLOR_BGR2HSV);

  // 複数色マスク統合
  cv::Mat combinedMask = cv::Mat::zeros(hsvFrame.size(), CV_8UC1);

  for(const auto& range : hsvRanges) {
    cv::Mat mask;
    cv::inRange(hsvFrame, range.lower, range.upper, mask);
    combinedMask |= mask;
  }

  // 画像処理のモルフォロジー処理でノイズを減らす
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  // オープニングで小さなノイズを除去
  cv::morphologyEx(combinedMask, combinedMask, cv::MORPH_OPEN, kernel);
  // クロージングで小さな穴を埋める
  cv::morphologyEx(combinedMask, combinedMask, cv::MORPH_CLOSE, kernel);

  // 輪郭検出
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(combinedMask.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  // 小さいノイズを除外して、一番大きい領域だけ選ぶ
  double maxArea = 0;
  std::vector<cv::Point> largestContour;
  for(const auto& contour : contours) {
    double area = cv::contourArea(contour);
    if(area > MIN_CONTOUR_AREA && area > maxArea) {
      maxArea = area;
      // 最大輪郭が同面積の場合は更新しない
      largestContour = contour;
    }
  }

  // 色が見つからなかった
  if(largestContour.empty()) {
    Logger::error("指定された色領域が見つかりませんでした。");
    return false;
  }

  result.wasDetected = true;

  // バウンディングボックス作成
  boundingBox = cv::boundingRect(largestContour);

  // ROIのオフセットを加算してフレーム全体基準の座標に変換
  setBoundingBoxResult(boundingBox, roiRect, result);

  return true;
}

int32_t ColorRegionDetector::findLargestColorIndex(const cv::Mat& hsvFrame,
                                                   const cv::Rect& boundingBox)
{
  double maxArea = 0.0;
  int32_t largestColorIndex = -1;

  // フレーム中のバウンディングボックス領域を切り取る
  cv::Mat targetHsvFrame = hsvFrame(boundingBox);

  // バウンディングボックス内の, 指定の各HSV範囲の該当画素数を数える
  for(size_t i = 0; i < hsvRanges.size(); ++i) {
    cv::Mat mask;
    cv::inRange(targetHsvFrame, hsvRanges[i].lower, hsvRanges[i].upper, mask);

    double area = static_cast<double>(cv::countNonZero(mask));
    // int area = static_cast<int>(cv::countNonZero(mask));

    if(area > maxArea) {
      maxArea = area;
      largestColorIndex = static_cast<int32_t>(i);
    }
  }
  return largestColorIndex;
}

void ColorRegionDetector::setBoundingBoxResult(const cv::Rect& boundingBox, const cv::Rect& roiRect,
                                               BoundingBoxDetectionResult& result)
{
  result.topLeft = cv::Point(boundingBox.x + roiRect.x, boundingBox.y + roiRect.y);
  result.topRight
      = cv::Point(boundingBox.x + boundingBox.width + roiRect.x, boundingBox.y + roiRect.y);
  result.bottomLeft
      = cv::Point(boundingBox.x + roiRect.x, boundingBox.y + boundingBox.height + roiRect.y);
  result.bottomRight = cv::Point(boundingBox.x + boundingBox.width + roiRect.x,
                                 boundingBox.y + boundingBox.height + roiRect.y);
}