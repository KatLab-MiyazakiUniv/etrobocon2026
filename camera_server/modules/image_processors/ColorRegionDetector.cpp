/**
 * @file ColorRegionDetector.cpp
 * @brief 色領域検出用の画像処理クラス
 * @author okuyama0528 sadomiya
 */

#include "ColorRegionDetector.h"

// ROIと解像度を指定するコンストラクタ
ColorRegionDetector::ColorRegionDetector(const std::vector<cv::Scalar>& _lowerHSV,
                                         const std::vector<cv::Scalar>& _upperHSV,
                                         const cv::Rect& _roi, const cv::Size& _resolution)
  : lowerHSV(_lowerHSV), upperHSV(_upperHSV), roi(_roi), resolution(_resolution)
{
  validateParameters();
}
// 解像度が小さすぎたり大きすぎたりしたら修正する
void ColorRegionDetector::validateParameters()
{
  if(resolution.width < MIN_WIDTH) resolution.width = MIN_WIDTH;
  if(resolution.width > MAX_WIDTH) resolution.width = MAX_WIDTH;
  if(resolution.height < MIN_HEIGHT) resolution.height = MIN_HEIGHT;
  if(resolution.height > MAX_HEIGHT) resolution.height = MAX_HEIGHT;
  // ROIが画像外に出ないように補正
  if(roi.x < 0) roi.x = 0;
  if(roi.y < 0) roi.y = 0;
  // ROIが画像サイズを超えないように調整
  if(roi.x + roi.width > resolution.width) roi.width = resolution.width - roi.x;

  if(roi.y + roi.height > resolution.height) roi.height = resolution.height - roi.y;
}

void ColorRegionDetector::detect(const cv::Mat& frame, BoundingBoxDetectionResult& result)
{
  result.wasDetected = false;
  // 画像が壊れていたら処理中止
  if(frame.empty()) {
    std::cerr << "Error: Input frame is empty." << std::endl;
    return;
  }

  // 画像サイズを統一する
  cv::Mat frameProcessed;
  if(frame.size() != resolution) {
    cv::resize(frame, frameProcessed, resolution);
  } else {
    frameProcessed = frame.clone();
  }

  // ROI切り出し
  cv::Rect roiRect = roi;
  roiRect = roiRect & cv::Rect(0, 0, frameProcessed.cols, frameProcessed.rows);

  if(roiRect.empty()) return;

  cv::Mat roiFrame = frameProcessed(roiRect);

  // RGBを HSVに変換
  cv::Mat hsvFrame;
  cv::cvtColor(roiFrame, hsvFrame, cv::COLOR_BGR2HSV);

  // 複数色マスク統合
  cv::Mat combinedMask = cv::Mat::zeros(hsvFrame.size(), CV_8UC1);

  for(size_t i = 0; i < lowerHSV.size(); i++) {
    cv::Mat mask;
    cv::inRange(hsvFrame, lowerHSV[i], upperHSV[i], mask);
    combinedMask |= mask;
  }

  // ノイズ除去
  // つながっていない部分をつなぐ
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  cv::morphologyEx(combinedMask, combinedMask, cv::MORPH_OPEN, kernel);
  cv::morphologyEx(combinedMask, combinedMask, cv::MORPH_CLOSE, kernel);

  // 輪郭検出
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(combinedMask.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  // 小さいノイズを除外して、一番大きい領域だけ選ぶ
  double maxArea = 0;
  std::vector<cv::Point> largestContour;

  for(const auto& contour : contours) {
    double area = cv::contourArea(contour);
    if(area > MIN_LINE_CONTOUR_AREA && area > maxArea) {
      maxArea = area;
      largestContour = contour;
    }
  }
  // 色が見つからなかった
  if(largestContour.empty()) {
    return;
  }
  result.wasDetected = true;
  // バウンディングボックス作成
  cv::Rect boundingBox = cv::boundingRect(largestContour);

  result.topLeft = cv::Point(boundingBox.x + roiRect.x, boundingBox.y + roiRect.y);
  // ROI補正
  result.topRight
      = cv::Point(boundingBox.x + boundingBox.width + roiRect.x, boundingBox.y + roiRect.y);

  result.bottomLeft
      = cv::Point(boundingBox.x + roiRect.x, boundingBox.y + boundingBox.height + roiRect.y);

  result.bottomRight = cv::Point(boundingBox.x + boundingBox.width + roiRect.x,
                                 boundingBox.y + boundingBox.height + roiRect.y);
}
