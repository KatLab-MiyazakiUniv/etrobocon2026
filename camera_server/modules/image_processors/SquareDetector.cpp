/**
 * @file   SquareDetector.cpp
 * @brief  点群から回転矩形を検出する画像処理クラス
 * @author okuyama0528 yutaro-1214
 */

#include "SquareDetector.h"

namespace {
  /**
   * @brief 正方形候補
   */
  struct SquareCandidate {
    cv::RotatedRect rect;
    double pointCount;
  };

  /**
 　* @brief 検出したいQRコードが画像の下側にあると予想されるため、その基準となるY位置
 　*/
  constexpr double TARGET_Y_RATIO = 0.82;

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
}  // namespace

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

  // 入力画像確認
  if(frame.empty()) {
    Logger::error("SquareDetector:入力フレームが空です。");
    return;
  }

  // ROIの切り出し
  const cv::Rect frameRect(0, 0, frame.cols, frame.rows);
  const cv::Rect roiRect = roi & frameRect;

  if(roiRect.empty()) {
    Logger::error("SquareDetector:"
                  "ROIがフレーム内に収まっていません。");
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
  constexpr int MIN_CLUSTER_POINTS = 25;   // QRコードの候補として認める最小点数

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

    Logger::printfLog(Logger::INFO, "SquareDetector: 中心=(%.1f,%.1f)", rect.center.x + roiRect.x,
                      rect.center.y + roiRect.y);
  }

  if(candidates.empty()) {
    Logger::warning("SquareDetector:"
                    "正方形候補が見つかりませんでした。");
    return;
  }

  // QRコードを探す基準位置の設定
  const double targetX = CAM_MAX_WIDTH / 2.0;
  const double targetY = CAM_MAX_HEIGHT * TARGET_Y_RATIO;

  double bestCenterY = -1.0;
  // 基準位置から一番近い候補までの距離
  double bestTargetDistance = std::numeric_limits<double>::max();
  bool foundCandidate = false;

  SquareCandidate bestCandidate{};
  // 最終候補選択
  for(const auto& candidate : candidates) {
    const double centerX = candidate.rect.center.x + static_cast<double>(roiRect.x);
    const double centerY = candidate.rect.center.y + static_cast<double>(roiRect.y);

    const double dx = centerX - targetX;
    const double dy = centerY - targetY;
    const double targetDistance = std::sqrt(dx * dx + dy * dy);

    Logger::printfLog(Logger::INFO,
                      "SquareDetector: "
                      "候補 "
                      "中心=(%.1f,%.1f) "
                      "基準位置=(%.1f,%.1f) "
                      "基準位置までの距離=%.1f",
                      centerX, centerY, targetX, targetY, targetDistance);

    // 画面下を優先し、同じくらいなら基準位置に近い方を選ぶ
    if(!foundCandidate || centerY > bestCenterY + 40.0
       || (std::abs(centerY - bestCenterY) <= 40.0 && targetDistance < bestTargetDistance)) {
      bestCandidate = candidate;
      bestCenterY = centerY;
      bestTargetDistance = targetDistance;
      foundCandidate = true;
    }
  }

  if(!foundCandidate) {
    Logger::warning("SquareDetector:"
                    "条件を満たす正方形候補がありません。");
    return;
  }

  // 選択したQRコードの中心座標を取得
  const cv::RotatedRect bestRect = bestCandidate.rect;
  const cv::Point2f currentCenter(bestRect.center.x + static_cast<float>(roiRect.x),
                                  bestRect.center.y + static_cast<float>(roiRect.y));

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

  Logger::printfLog(Logger::INFO,
                    "SquareDetector: "
                    "正方形を検出しました "
                    "中心=(%.1f,%.1f) "
                    "目標距離=%.1f",
                    currentCenter.x, currentCenter.y, bestTargetDistance);

  Logger::printfLog(Logger::INFO,
                    "SquareDetector: "
                    "左上=(%d,%d) "
                    "右上=(%d,%d) "
                    "右下=(%d,%d) "
                    "左下=(%d,%d)",
                    result.topLeft.x, result.topLeft.y, result.topRight.x, result.topRight.y,
                    result.bottomRight.x, result.bottomRight.y, result.bottomLeft.x,
                    result.bottomLeft.y);
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