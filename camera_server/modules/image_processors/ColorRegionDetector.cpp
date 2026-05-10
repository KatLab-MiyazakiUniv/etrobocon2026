/**
 * @file ColorRegionDetector.cpp
 * @brief 色領域検出用の画像処理クラス
 * @author okuyama0528 sadomiya
 */
// /
#include "ColorRegionDetector.h"
#include "FrameSave.h"

// ROIと解像度を指定するコンストラクタ
ColorRegionDetector::ColorRegionDetector(const std::vector<cv::Scalar>& _lowerHSV,
                                         const std::vector<cv::Scalar>& _upperHSV,
                                         const cv::Rect& _roi, const cv::Size& _resolution)
  : lowerHSV(_lowerHSV), upperHSV(_upperHSV), roi(_roi), resolution(_resolution)
{
  validateParameters();
}

void ColorRegionDetector::validateParameters()
{
  if(resolution.width < MIN_WIDTH) resolution.width = MIN_WIDTH;
  if(resolution.width > MAX_WIDTH) resolution.width = MAX_WIDTH;
  if(resolution.height < MIN_HEIGHT) resolution.height = MIN_HEIGHT;
  if(resolution.height > MAX_HEIGHT) resolution.height = MAX_HEIGHT;

  if(roi.x < 0) roi.x = 0;
  if(roi.y < 0) roi.y = 0;

  if(roi.x + roi.width > resolution.width) roi.width = resolution.width - roi.x;

  if(roi.y + roi.height > resolution.height) roi.height = resolution.height - roi.y;
}

void ColorRegionDetector::detect(const cv::Mat& frame, BoundingBoxDetectionResult& result)
{
  result.wasDetected = false;

  if(frame.empty()) {
    std::cerr << "Error: Input frame is empty." << std::endl;
    return;
  }

  // リサイズ
  cv::Mat frameProcessed;
  if(frame.size() != resolution) {
    cv::resize(frame, frameProcessed, resolution);
  } else {
    frameProcessed = frame.clone();
  }

  // ROI
  cv::Rect roiRect = roi;
  roiRect = roiRect & cv::Rect(0, 0, frameProcessed.cols, frameProcessed.rows);

  if(roiRect.empty()) return;

  cv::Mat roiFrame = frameProcessed(roiRect);

  std::string filepath = "./frames";
  std::string filename = "result-roi";

  FrameSave::save(roiFrame, filepath, filename);

  // HSV変換
  cv::Mat hsvFrame;
  cv::cvtColor(roiFrame, hsvFrame, cv::COLOR_BGR2HSV);

  filename = "result-hsv";

  FrameSave::save(hsvFrame, filepath, filename);

  // 複数色マスク統合
  cv::Mat combinedMask = cv::Mat::zeros(hsvFrame.size(), CV_8UC1);

  for(size_t i = 0; i < lowerHSV.size(); i++) {
    cv::Mat mask;
    cv::inRange(hsvFrame, lowerHSV[i], upperHSV[i], mask);
    combinedMask |= mask;
  }
  filename = "result-cmbinedMask";

  FrameSave::save(combinedMask, filepath, filename);
  // ノイズ除去
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  cv::morphologyEx(combinedMask, combinedMask, cv::MORPH_OPEN, kernel);
  cv::morphologyEx(combinedMask, combinedMask, cv::MORPH_CLOSE, kernel);

  filename = "result-kerenel";

  FrameSave::save(kernel, filepath, filename);

  // 輪郭検出
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(combinedMask.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  double maxArea = 0;
  std::vector<cv::Point> largestContour;

  for(const auto& contour : contours) {
    double area = cv::contourArea(contour);
    if(area > MIN_LINE_CONTOUR_AREA && area > maxArea) {
      maxArea = area;
      largestContour = contour;
    }
  }

  if(largestContour.empty()) return;

  result.wasDetected = true;

  cv::Rect boundingBox = cv::boundingRect(largestContour);

  result.topLeft = cv::Point(boundingBox.x + roiRect.x, boundingBox.y + roiRect.y);

  result.topRight
      = cv::Point(boundingBox.x + boundingBox.width + roiRect.x, boundingBox.y + roiRect.y);

  result.bottomLeft
      = cv::Point(boundingBox.x + roiRect.x, boundingBox.y + boundingBox.height + roiRect.y);

  result.bottomRight = cv::Point(boundingBox.x + boundingBox.width + roiRect.x,
                                 boundingBox.y + boundingBox.height + roiRect.y);
}
