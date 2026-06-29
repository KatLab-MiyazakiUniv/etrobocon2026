/**
 * @file   ChangeEdge.h
 * @brief  エッジを切り替えるクラス
 * @author migaku2645
 */

#ifndef CHANGE_EDGE_H
#define CHANGE_EDGE_H

#include "BaseMotion.h"
#include "Course.h"
#include "Logger.h"
#include <string>
class ChangeEdge : public BaseMotion {
 public:
  /**
   * コンストラクタ
   * @brief Robotと継続条件を初期化する
   * @param _robot Robotクラスのインスタンス
   * @param _continuationCondition 継続条件
   */
  ChangeEdge(Robot& _robot, std::unique_ptr<BaseContinuationCondition> _continuationCondition,
             std::string _edge);

  /**
   * デストラクタ
   * @brief 派生クラスを基底クラスのポインタで破棄できるようにする
   */
  ~ChangeEdge();

 protected:
  /**
   * @brief 動作を開始できるかを判定する (デフォルトは true を返す)
   * @return true/動作を開始できる、false/動作を開始できない
   */
  bool canStart() override;

  /**
   * @brief 1周期分の動作を実行する
   */
  void executeStep() override;

 private:
  std::unique_ptr<BaseContinuationCondition> continuationCondition;  // 継続条件クラスのインスタンス
  std::string edge;
  bool flag = false;
};

#endif