
/**
 * @file   SquareDetector.cpp
 * @brief  点群から回転矩形を検出する画像処理クラス
 * @author okuyama0528 yutaro-1214
 */

#include "SquareDetector.h"

#include <climits>
#include <cmath>
#include <vector>

namespace {
  /**
   * @brief 正方形候補
   */
  struct SquareCandidate {
    cv::RotatedRect rect;
    double pointCount;
  };

  /**
   * @brief QRコードの黒と白の境目を検出するための基準値
   */
  constexpr double CANNY_THRESHOLD_LOW = 50.0;

  /**
   * @brief QRコードの黒と白の境目を強い線として判断するための基準値
   */
  constexpr double CANNY_THRESHOLD_HIGH = 150.0;

  /**
   * @brief モルフォロジー処理で線をつなげる範囲の大きさ
   */
  constexpr int MORPH_KERNEL_SIZE = 7;

  /**
   * @brief 左右から検出対象外にする幅 [px]
   */
  constexpr int SIDE_CROP_PX = 0;

  /**
   * @brief 上側から検出対象外にする高さ [px]
   */
  constexpr int TOP_CROP_PX = 000;
}  // namespace

SquareDetector::SquareDetector(const cv::Rect& _roi) : roi(_roi)
{
  validateParameters();
}

SquareDetector::~SquareDetector() {}

void SquareDetector::detect(const cv::Mat& frame, BoundingBoxDetectionResult& result)
{
  result.wasDetected = false;

  // 入力画像確認
  if(frame.empty()) {
    return;
  }

  // ROIの切り出し
  constexpr int SIDE_MARGIN = SIDE_CROP_PX;
  constexpr int TOP_MARGIN = TOP_CROP_PX;

  if(frame.cols <= SIDE_MARGIN * 2) {
    Logger::error("SquareDetector: 画像の横幅が不足しています。");
    return;
  }

  if(frame.rows <= TOP_MARGIN) {
    Logger::error("SquareDetector: 画像の高さが不足しています。");
    return;
  }

  const cv::Rect frameRect(0, 0, frame.cols, frame.rows);
  const cv::Rect croppedRect(SIDE_MARGIN, TOP_MARGIN, frame.cols - SIDE_MARGIN * 2,
                             frame.rows - TOP_MARGIN);
  const cv::Rect roiRect = roi & frameRect & croppedRect;

  if(roiRect.empty()) {
    return;
  }

  const cv::Mat roiFrame = frame(roiRect);

  // グレースケール化
  cv::Mat grayFrame;
  cv::cvtColor(roiFrame, grayFrame, cv::COLOR_BGR2GRAY);

  // 画像を少しぼかして細かいノイズを減らす
  cv::Mat blurFrame;
  cv::GaussianBlur(grayFrame, blurFrame, cv::Size(5, 5), 0);

  // QRコードの黒と白の境目を線として検出
  cv::Mat edgeFrame;
  cv::Canny(blurFrame, edgeFrame, CANNY_THRESHOLD_LOW, CANNY_THRESHOLD_HIGH);

  // 近くにある線や点をつなげる
  cv::Mat morphFrame;
  const cv::Mat kernel
      = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(MORPH_KERNEL_SIZE, MORPH_KERNEL_SIZE));
  cv::morphologyEx(edgeFrame, morphFrame, cv::MORPH_CLOSE, kernel);

  // QRコードの特徴的な点を検出
  std::vector<cv::Point2f> featurePoints;
  cv::goodFeaturesToTrack(morphFrame, featurePoints, 300, 0.01, 8);

  // QRコードの候補
  std::vector<SquareCandidate> candidates;

  constexpr float CLUSTER_RADIUS = 45.0f;  // 点を同じグループとしてまとめる範囲[px]
  constexpr int MIN_CLUSTER_POINTS = 100;   // QRコードの候補として認める最小点数

  // 近くにある点をひとまとめにする
  std::vector<bool> used(featurePoints.size(), false);

  for(size_t i = 0; i < featurePoints.size(); i++) {
    if(used[i]) {
      continue;
    }

    std::vector<cv::Point2f> cluster;
    cluster.push_back(featurePoints[i]);
    used[i] = true;

    bool expanded = true;

    while(expanded) {
      expanded = false;

      for(size_t j = 0; j < featurePoints.size(); j++) {
        if(used[j]) {
          continue;
        }

        for(const auto& p : cluster) {
          if(cv::norm(featurePoints[j] - p) <= CLUSTER_RADIUS) {
            cluster.push_back(featurePoints[j]);
            used[j] = true;
            expanded = true;
            break;
          }
        }
      }
    }

    if(static_cast<int>(cluster.size()) < MIN_CLUSTER_POINTS) {
      continue;
    }

    // 点の集まりを囲む長方形を作る
    cv::RotatedRect rect = cv::minAreaRect(cluster);

    // QRの候補として保存
    candidates.push_back({ rect, static_cast<double>(cluster.size()) });
  }

  if(candidates.empty()) {
    return;
  }

  // ========================================
  // 最終候補選択
  // 候補の中で最も下側にあるものを必ず選択
  // ========================================

  double bestCenterY = -1.0;
  bool foundCandidate = false;
  SquareCandidate bestCandidate{};

  for(const auto& candidate : candidates) {
    const double centerY = candidate.rect.center.y + static_cast<double>(roiRect.y);

    // Y座標が最大の候補を選ぶ
    // X座標や候補の大きさは考慮しない
    if(!foundCandidate || centerY > bestCenterY) {
      bestCandidate = candidate;
      bestCenterY = centerY;
      foundCandidate = true;
    }
  }

  if(!foundCandidate) {
    return;
  }

  // 選択したQRコードの中心座標を取得
  const cv::RotatedRect bestRect = bestCandidate.rect;

  cv::Point2f rectPoints[4];
  bestRect.points(rectPoints);

  std::vector<cv::Point> corners;
  corners.reserve(4);

  for(int i = 0; i < 4; ++i) {
    corners.emplace_back(static_cast<int>(rectPoints[i].x) + roiRect.x,
                         static_cast<int>(rectPoints[i].y) + roiRect.y);
  }

  cv::Point topLeft;
  cv::Point topRight;
  cv::Point bottomLeft;
  cv::Point bottomRight;

  // 4つの角の位置を判定する
  int minSum = INT_MAX;
  int maxSum = INT_MIN;
  int minDiff = INT_MAX;
  int maxDiff = INT_MIN;

  for(const auto& point : corners) {
    const int sum = point.x + point.y;
    const int diff = point.x - point.y;

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
  result.bottomRight = bottomRight;
  result.bottomLeft = bottomLeft;
  result.wasDetected = true;
}

void SquareDetector::setValidatedRoi(const cv::Rect& _roi)
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

  if(roi.width > CAM_MAX_WIDTH - roi.x) {
    roi.width = CAM_MAX_WIDTH - roi.x;
  }

  if(roi.height < 0) {
    roi.height = 0;
  }

  if(roi.height > CAM_MAX_HEIGHT - roi.y) {
    roi.height = CAM_MAX_HEIGHT - roi.y;
  }
}
