/**
 * @file   CodeDetector.h
 * @brief  コード検出処理の基底クラス
 * @author HaruArima08
 */

#ifndef CODE_DETECTOR_H
#define CODE_DETECTOR_H

#include <opencv2/core.hpp>

template <typename TResult>
class CodeDetector {
 public:
  /**
   * 仮想デストラクタ
   * @brief 派生クラスのデストラクタが正しく呼ばれるようにするために必要
   */
  virtual ~CodeDetector() = default;

  /**
   * @brief コード検出を実行する純粋仮想関数
   * @param frame 処理対象のフレーム
   * @return 検出結果（TResult型）
   */
  virtual TResult detect(const cv::Mat& frame) = 0;
};

#endif  // CODE_DETECTOR_H
