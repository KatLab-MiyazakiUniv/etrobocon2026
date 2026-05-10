/**
 * @file   BoundingBoxDetector.h
 * @brief  バウンディングボックス検出処理の親クラス
 * @author okuyama0528 sadomiya
 */

#ifndef BOUNDING_BOX_DETECTOR_H
#define BOUNDING_BOX_DETECTOR_H

#include "ImageRecoginationResult.h"

#include <string>
#include <opencv2/opencv.hpp>

class BoundingBoxDetector {
 public:
  /**
   * 仮想デストラクタ
   * @brief 派生クラスのデストラクタが正しく呼ばれるようにするために必要
   */
  virtual ~BoundingBoxDetector() = default;

  /**
   * @brief 物体検出を実行する純粋仮想関数
   * @param frame 処理対象の画像フレーム
   * @param result 結果を格納するBoundingBoxDetectionResult構造体の参照
   */

  // 純粋仮想関数>子クラスが強制的に上書きしなければけないメンバー関数＞=0;
  // 参照渡しにすること画像のデータ数MBをcopyせずに値を渡す
  // cv::Mat>cvはopen cv libraly>Mat>行列
  virtual void detect(const cv::Mat& frame, BoundingBoxDetectionResult& result) = 0;
};

#endif
