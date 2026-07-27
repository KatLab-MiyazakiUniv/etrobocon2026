/**
 * @file   BaseMotion.h
 * @brief  動作の実行処理を共通化するための基底クラス
 * @author takuchi17
 */

#ifndef BASE_MOTION_H
#define BASE_MOTION_H

#include <memory>

#include "Robot.h"
#include "BaseContinuationCondition.h"
#include "ClockUtil.h"

class BaseMotion {
 public:
  /**
   * コンストラクタ
   * @brief Robotと継続条件を初期化する
   * @param _robot Robotクラスのインスタンス
   * @param _continuationCondition 継続条件
   */
  BaseMotion(Robot& _robot, std::unique_ptr<BaseContinuationCondition> _continuationCondition);

  /**
   * デストラクタ
   * @brief 派生クラスを基底クラスのポインタで破棄できるようにする
   */
  virtual ~BaseMotion() = default;

  /**
   * @brief 動作を実行する
   */
  void run();

 protected:
  Robot& robot;  // Robotインスタンスの参照

  /**
   * @brief 動作を開始できるかを判定する
   * @return true 動作を開始できる
   * @return false 動作を開始できない
   */
  virtual bool canStart();

  /**
   * @brief 動作実行前の準備を行う
   */
  virtual void prepare();

  /**
   * @brief 1周期分の動作を実行する
   */
  virtual void executeStep() = 0;

  /**
   * @brief 1周期分の待機を行う
   */
  virtual void wait();

  /**
   * @brief 動作終了後の処理を行う
   */
  virtual void finish();

 private:
  /**
   * @brief 現在のエンコーダ値とIMU角度から自己位置を更新する
   */
  void updateOdometry();

  /**
   * 継続条件クラスのインスタンス
   */
  std::unique_ptr<BaseContinuationCondition> continuationCondition;
};

#endif  // BASE_MOTION_H