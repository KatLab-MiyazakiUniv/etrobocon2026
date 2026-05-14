/*
 * @file ColorRegionDetector.cpp
 * @brief 色領域検出用の画像処理クラス
 * @author okuyama0528, sadomiya-sousi
 */

#include "ColorRegionDetector.h"

// 全てのパラメータを個別に指定する
ColorRegionDetector::ColorRegionDetector(const std::vector<HSVRange>& _hsvRanges,
                                         const cv::Rect& _roi, const cv::Size& _resolution)
  : hsvRanges(_hsvRanges), roi(_roi), resolution(_resolution)
{
  validateParameters();
}

// 解像度がデフォルトのコンストラクタ
ColorRegionDetector::ColorRegionDetector(const std::vector<HSVRange>& _hsvRanges,
                                         const cv::Rect& _roi)
  : ColorRegionDetector(_hsvRanges, _roi, cv::Size(640, 480))
{
}

// デフォルトのROIおよび解像度のコンストラクタ
ColorRegionDetector::ColorRegionDetector(const std::vector<HSVRange>& _hsvRanges)
  : ColorRegionDetector(_hsvRanges, cv::Rect(50, 240, 540, 240))
{
}

// 解像度の調節後にROIをフレーム内に収める補正
void ColorRegionDetector::validateParameters()
{
  // ROIがフレームサイズ内か確認し、必要に応じて補正
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
  // 入力フレームが空の場合は処理を中断する
  if(frame.empty()) {
    std::cerr << "Error: Input frame is empty." << std::endl;
    return;
  }

  // フレームサイズを統一する
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
    std::cout << "指定された色領域が見つかりませんでした。" << std::endl;
    return;
  }
  result.wasDetected = true;
  // バウンディングボックス作成
  cv::Rect boundingBox = cv::boundingRect(largestContour);

  result.topLeft = cv::Point(boundingBox.x + roiRect.x, boundingBox.y + roiRect.y);
  // ROIのオフセットを加算してフレーム全体基準の座標に変換
  result.topRight
      = cv::Point(boundingBox.x + boundingBox.width + roiRect.x, boundingBox.y + roiRect.y);

  result.bottomLeft
      = cv::Point(boundingBox.x + roiRect.x, boundingBox.y + boundingBox.height + roiRect.y);

  result.bottomRight = cv::Point(boundingBox.x + boundingBox.width + roiRect.x,
                                 boundingBox.y + boundingBox.height + roiRect.y);
}
