/**
 * @file   ColorRegionCenterCondition.h
 * @brief  色領域検出結果の中心座標が指定範囲内に入るまで継続する条件クラス
 */

#ifndef COLOR_REGION_CENTER_CONDITION_H
#define COLOR_REGION_CENTER_CONDITION_H

#include "BaseContinuationCondition.h"
#include "SocketProtocol.h"
#include "Logger.h"
#include <cmath>

class ColorRegionCenterCondition : public BaseContinuationCondition {
 public:
  /**
   * コンストラクタ
   * @param _robot Robot クラスのインスタンスの参照
   * @param _colorDetectionRequest 色領域検出リクエスト設定
   * @param _targetCenterX 目標とする中心X座標
   * @param _toleranceX 許容誤差範囲 (目標 ± toleranceX)
   * @param _consecutiveCountThreshold 終了判定に必要な連続合致回数 (デフォルト1)
   */
  ColorRegionCenterCondition(
      Robot& _robot,
      const CameraServer::ColorRegionDetectorRequest& _colorDetectionRequest,
      double _targetCenterX,
      double _toleranceX,
      int _consecutiveCountThreshold = 1);

  /**
   * デストラクタ
   */
  ~ColorRegionCenterCondition();

  /**
   * @brief 初期化処理
   */
  void prepare() override;

  /**
   * @brief 色領域の中心座標が目標範囲内に入った場合、動作を継続しない (false) と判定する
   * @return true: 継続する / false: 終了する (目標範囲に入った)
   */
  bool shouldContinue() override;

  /**
   * @brief 目標中心X座標を取得する
   */
  double getTargetCenterX() const;

  /**
   * @brief 許容誤差範囲を取得する
   */
  double getToleranceX() const;

 private:
  CameraServer::ColorRegionDetectorRequest colorDetectionRequest;
  double targetCenterX;              // 目標中心X座標
  double toleranceX;                 // 許容誤差 (±toleranceX)
  int consecutiveCountThreshold;     // 条件を満たす必要がある連続回数
  int inRangeCount;                  // 範囲内に入った連続回数
};

#endif  // COLOR_REGION_CENTER_CONDITION_H
