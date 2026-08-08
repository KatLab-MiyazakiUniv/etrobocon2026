/**
 * @file   RouteNavigation.h
 * @brief  Dijkstraで求めた経路を実際に走行するクラス
 */

#ifndef ROUTE_NAVIGATION_H
#define ROUTE_NAVIGATION_H

#include <memory>
#include <vector>

#include "BaseMotion.h"
#include "DijkstraRoutePlanner.h"
#include "Pid.h"

class RouteNavigation : public BaseMotion {
public:
  /**
   * @brief コンストラクタ
   *
   * @param _robot Robotクラスのインスタンス
   * @param _condition RouteNavigation全体の継続条件
   * @param _route Dijkstraで求めた経路
   * @param _targetSpeed 走行速度[mm/s]
   * @param _rotationPid 回頭用PID
   * @param _rightPid 右タイヤPID
   * @param _leftPid 左タイヤPID
   * @param _straightAnglePid 直進角度PID
   */
  RouteNavigation(
      Robot& _robot,
      std::unique_ptr<BaseContinuationCondition> _condition,
      const std::vector<RouteState>& _route,
      double _targetSpeed,
      const Pid::PidGain& _rotationPid,
      const Pid::PidGain& _rightPid,
      const Pid::PidGain& _leftPid,
      const Pid::PidGain& _straightAnglePid);

  ~RouteNavigation() override;

protected:
  void prepare() override;

  void executeStep() override;

  void finish() override;

private:
  /**
   * @brief Dijkstraの論理座標を実際のmm座標へ変換する
   */
  double convertXToMm(int x) const;

  double convertYToMm(int y) const;

  /**
   * @brief 1区間のGoalNavigationを実行する
   */
  void executeSegment(
      const RouteState& current,
      const RouteState& next);

private:
  /**
   * @brief Dijkstraの経路
   */
  std::vector<RouteState> route;

  /**
   * @brief 現在実行している区間
   */
  std::size_t currentSegment;

  /**
   * @brief 走行速度
   */
  double targetSpeed;

  /**
   * @brief PID
   */
  Pid::PidGain rotationPid;
  Pid::PidGain rightPid;
  Pid::PidGain leftPid;
  Pid::PidGain straightAnglePid;

  /**
   * @brief マップ上の1グリッドの実寸[mm]
   *
   * TODO:
   * 実際のETラリーのマップ寸法に合わせて変更する。
   */
  static constexpr double GRID_SIZE_MM = 104.0;
};

#endif  // ROUTE_NAVIGATION_H