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
  };

  /**
   * @brief 候補として残す面積割合
   *
   * 最大面積の70%以上の候補だけを
   * 最終候補の選択対象とする。
   *
   * 小さなノイズを誤検出しにくくする。
   */
  constexpr double AREA_RATIO = 0.70;

  /**
   * @brief 正方形を探す基準Y位置
   *
   * 画像下側にQRが映ることを想定し、
   * 画像高さの82%付近を基準とする。
   *
   * 1920x1080の場合
   *
   * targetY = 約886px
   */
  constexpr double TARGET_Y_RATIO = 0.82;

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

  // =====================================================
  // 入力画像確認
  // =====================================================

  if(frame.empty()) {
    Logger::error("SquareDetector:"
                  "入力フレームが空です。");

    return;
  }

  // =====================================================
  // ROI設定
  // =====================================================

  const cv::Rect frameRect(0, 0, frame.cols, frame.rows);

  const cv::Rect roiRect = roi & frameRect;

  if(roiRect.empty()) {
    Logger::error("SquareDetector:"
                  "ROIがフレーム内に収まっていません。");

    return;
  }

  const cv::Mat roiFrame = frame(roiRect);

  // =====================================================
  // グレースケール化
  // =====================================================

  cv::Mat grayFrame;

  cv::cvtColor(roiFrame, grayFrame, cv::COLOR_BGR2GRAY);

  // =====================================================
  // ノイズ除去
  // =====================================================

  cv::GaussianBlur(grayFrame, grayFrame, cv::Size(5, 5), 0);

  // =====================================================
  // 二値化
  // =====================================================

  cv::Mat binary;

  cv::threshold(grayFrame, binary, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);

  // =====================================================
  // モルフォロジー処理
  // =====================================================

  const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

  cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, kernel);

  // =====================================================
  // 輪郭検出
  // =====================================================

  std::vector<std::vector<cv::Point>> contours;

  cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  Logger::printfLog(Logger::INFO, "SquareDetector: contour count = %d",
                    static_cast<int>(contours.size()));

  // =====================================================
  // 正方形候補を探す
  // =====================================================

  std::vector<SquareCandidate> candidates;

  for(const auto& contour : contours) {
    const double area = cv::contourArea(contour);

    // ---------------------------------------------------
    // 小さすぎる輪郭を除外
    // ---------------------------------------------------

    if(area < MIN_CONTOUR_AREA) {
      continue;
    }

    // ---------------------------------------------------
    // 周長
    // ---------------------------------------------------

    const double perimeter = cv::arcLength(contour, true);

    if(perimeter <= 0.0) {
      continue;
    }

    // ---------------------------------------------------
    // 円形度
    //
    // 円:
    //   1.0に近い
    //
    // 正方形:
    //   約0.785
    // ---------------------------------------------------

    const double circularity = 4.0 * CV_PI * area / (perimeter * perimeter);

    if(circularity > MAX_CIRCULARITY) {
      Logger::printfLog(Logger::DEBUG,
                        "SquareDetector: "
                        "circle rejected "
                        "area=%.2f circularity=%.2f",
                        area, circularity);

      continue;
    }

    // ---------------------------------------------------
    // 最小外接回転矩形
    // ---------------------------------------------------

    const cv::RotatedRect rect = cv::minAreaRect(contour);

    const double width = rect.size.width;

    const double height = rect.size.height;

    if(width <= 0.0 || height <= 0.0) {
      continue;
    }

    // ---------------------------------------------------
    // 縦横比
    //
    // 1.0に近いほど正方形
    // ---------------------------------------------------

    const double ratio = std::min(width, height) / std::max(width, height);

    if(ratio < MIN_RATIO) {
      Logger::printfLog(Logger::DEBUG,
                        "SquareDetector: "
                        "ratio rejected "
                        "area=%.2f ratio=%.2f",
                        area, ratio);

      continue;
    }

    // ---------------------------------------------------
    // 外接矩形に対する面積割合
    // ---------------------------------------------------

    const double rectArea = width * height;

    if(rectArea <= 0.0) {
      continue;
    }

    const double fillRatio = area / rectArea;

    if(fillRatio < MIN_FILL_RATIO) {
      Logger::printfLog(Logger::DEBUG,
                        "SquareDetector: "
                        "fill rejected "
                        "area=%.2f fill=%.2f",
                        area, fillRatio);

      continue;
    }

    // ---------------------------------------------------
    // フレーム全体での中心位置
    // ---------------------------------------------------

    const double centerX = rect.center.x + static_cast<double>(roiRect.x);

    const double centerY = rect.center.y + static_cast<double>(roiRect.y);

    Logger::printfLog(Logger::INFO,
                      "SquareDetector: candidate "
                      "area=%.2f "
                      "width=%.2f "
                      "height=%.2f "
                      "ratio=%.2f "
                      "fill=%.2f "
                      "circularity=%.2f "
                      "center=(%.1f,%.1f)",
                      area, width, height, ratio, fillRatio, circularity, centerX, centerY);

    candidates.push_back(SquareCandidate{ rect, area });
  }

  // =====================================================
  // 候補なし
  // =====================================================

  if(candidates.empty()) {
    Logger::warning("SquareDetector: "
                    "正方形候補が見つかりませんでした。");

    return;
  }

  // =====================================================
  // 最大面積を取得
  // =====================================================

  double maxArea = 0.0;

  for(const auto& candidate : candidates) {
    if(candidate.area > maxArea) {
      maxArea = candidate.area;
    }
  }

  // =====================================================
  // 面積が極端に小さい候補を除外する
  // =====================================================

  const double minimumArea = maxArea * AREA_RATIO;

  // =====================================================
  // 今回の画像だけを使用して候補を決定
  //
  // X:
  //   画像中央
  //
  // Y:
  //   画像高さの82%
  //
  // 前回の検出結果は一切使用しない。
  // =====================================================

  const double targetX = CAM_MAX_WIDTH / 2.0;

  const double targetY = CAM_MAX_HEIGHT * TARGET_Y_RATIO;

  double bestTargetDistance = std::numeric_limits<double>::max();

  bool foundCandidate = false;

  SquareCandidate bestCandidate = candidates.front();

  for(const auto& candidate : candidates) {
    // ---------------------------------------------------
    // 面積が小さすぎる候補を除外
    // ---------------------------------------------------

    if(candidate.area < minimumArea) {
      Logger::printfLog(Logger::DEBUG,
                        "SquareDetector: "
                        "candidate area rejected "
                        "area=%.2f minimumArea=%.2f",
                        candidate.area, minimumArea);

      continue;
    }

    // ---------------------------------------------------
    // フレーム全体での中心位置
    // ---------------------------------------------------

    const double centerX = candidate.rect.center.x + static_cast<double>(roiRect.x);

    const double centerY = candidate.rect.center.y + static_cast<double>(roiRect.y);

    // ---------------------------------------------------
    // 基準位置との距離
    // ---------------------------------------------------

    const double dx = centerX - targetX;

    const double dy = centerY - targetY;

    const double targetDistance = std::sqrt(dx * dx + dy * dy);

    Logger::printfLog(Logger::INFO,
                      "SquareDetector: "
                      "selection candidate "
                      "area=%.2f "
                      "center=(%.1f,%.1f) "
                      "target=(%.1f,%.1f) "
                      "targetDistance=%.1f",
                      candidate.area, centerX, centerY, targetX, targetY, targetDistance);

    // ---------------------------------------------------
    // 基準位置に一番近い候補を選択
    // ---------------------------------------------------

    if(!foundCandidate || targetDistance < bestTargetDistance) {
      bestCandidate = candidate;

      bestTargetDistance = targetDistance;

      foundCandidate = true;
    }
  }

  // =====================================================
  // 念のため候補が選択されなかった場合
  //
  // 最大面積候補を使用
  // =====================================================

  if(!foundCandidate) {
    Logger::warning("SquareDetector: "
                    "target candidate not found "
                    "-> use largest area");

    bestCandidate = candidates.front();

    for(const auto& candidate : candidates) {
      if(candidate.area > bestCandidate.area) {
        bestCandidate = candidate;
      }
    }
  }

  // =====================================================
  // 選択された正方形
  // =====================================================

  const cv::RotatedRect bestRect = bestCandidate.rect;

  const cv::Point2f currentCenter(bestRect.center.x + static_cast<float>(roiRect.x),
                                  bestRect.center.y + static_cast<float>(roiRect.y));

  // =====================================================
  // 4頂点を取得
  // =====================================================

  cv::Point2f rectPoints[4];

  bestRect.points(rectPoints);

  std::vector<cv::Point> corners;

  corners.reserve(4);

  for(int i = 0; i < 4; ++i) {
    corners.emplace_back(static_cast<int>(rectPoints[i].x) + roiRect.x,
                         static_cast<int>(rectPoints[i].y) + roiRect.y);
  }

  // =====================================================
  // 4頂点をTL / TR / BR / BLに分類
  // =====================================================

  cv::Point topLeft;
  cv::Point topRight;
  cv::Point bottomLeft;
  cv::Point bottomRight;

  int minSum = INT_MAX;

  int maxSum = INT_MIN;

  int minDiff = INT_MAX;

  int maxDiff = INT_MIN;

  for(const auto& point : corners) {
    const int sum = point.x + point.y;

    const int diff = point.x - point.y;

    // 左上
    if(sum < minSum) {
      minSum = sum;

      topLeft = point;
    }

    // 右下
    if(sum > maxSum) {
      maxSum = sum;

      bottomRight = point;
    }

    // 右上
    if(diff > maxDiff) {
      maxDiff = diff;

      topRight = point;
    }

    // 左下
    if(diff < minDiff) {
      minDiff = diff;

      bottomLeft = point;
    }
  }

  // =====================================================
  // 結果設定
  // =====================================================

  result.topLeft = topLeft;

  result.topRight = topRight;

  result.bottomRight = bottomRight;

  result.bottomLeft = bottomLeft;

  result.wasDetected = true;

  // =====================================================
  // ログ
  // =====================================================

  Logger::printfLog(Logger::INFO,
                    "SquareDetector: "
                    "===== SQUARE DETECTED ===== "
                    "area=%.2f "
                    "center=(%.1f,%.1f)",
                    bestCandidate.area, currentCenter.x, currentCenter.y);

  Logger::printfLog(Logger::INFO,
                    "SquareDetector: "
                    "TL=(%d,%d) "
                    "TR=(%d,%d) "
                    "BR=(%d,%d) "
                    "BL=(%d,%d)",
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
  // =====================================================
  // X座標
  // =====================================================

  if(roi.x < 0) {
    roi.x = 0;
  }

  if(roi.x > CAM_MAX_WIDTH) {
    roi.x = CAM_MAX_WIDTH;
  }

  // =====================================================
  // Y座標
  // =====================================================

  if(roi.y < 0) {
    roi.y = 0;
  }

  if(roi.y > CAM_MAX_HEIGHT) {
    roi.y = CAM_MAX_HEIGHT;
  }

  // =====================================================
  // 幅
  // =====================================================

  if(roi.width < 0) {
    roi.width = 0;
  }

  if(roi.width > CAM_MAX_WIDTH - roi.x) {
    roi.width = CAM_MAX_WIDTH - roi.x;
  }

  // =====================================================
  // 高さ
  // =====================================================

  if(roi.height < 0) {
    roi.height = 0;
  }

  if(roi.height > CAM_MAX_HEIGHT - roi.y) {
    roi.height = CAM_MAX_HEIGHT - roi.y;
  }
}