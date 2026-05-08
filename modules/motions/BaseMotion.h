/**
 * @file   BaseMotion.h
 * @brief  動作の実行処理を共通化するための基底クラス
 * @author takuchi17
 */

#ifndef BASE_MOTION_H
#define BASE_MOTION_H

#include "Robot.h"
#include "BaseContinuationCondition.h"
#include <memory>

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
  Robot& robot;  // Robot インスタンスの参照

  /**
   * @brief 動作を開始できるかを判定する (デフォルトは true を返す)
   * @return true/動作を開始できる、false/動作を開始できない
   */
  virtual bool canStart();

  /**
   * @brief 動作実行前の準備を行う (デフォルトは何もしない)
   */
  virtual void prepare();

  /**
   * @brief 1周期分の動作を実行する
   */
  virtual void executeStep() = 0;

  /**
   * @brief 1周期分の待機を行う (デフォルトは何もしない)
   */
  virtual void wait();

  /**
   * @brief 動作終了後の処理を行う (デフォルトは何もしない)
   */
  virtual void finish();

 private:
  std::unique_ptr<BaseContinuationCondition> continuationCondition;  // 継続条件クラスのインスタンス
};

#endif