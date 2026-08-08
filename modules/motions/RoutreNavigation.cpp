/**
 * @file   RouteNavigation.cpp
 * @brief  Dijkstraで求めた経路を実際に走行するクラス
 */

#include "RouteNavigation.h"

#include <memory>
#include <utility>

#include "GoalNavigation.h"
#include "RepeatCountCondition.h"

RouteNavigation::RouteNavigation(
    Robot& _robot,
    std::unique_ptr<BaseContinuationCondition> _condition,
    const std::vector<RouteState>& _route,
    double _targetSpeed,
    const Pid::PidGain& _rotationPid,
    const Pid::PidGain& _rightPid,
    const Pid::PidGain& _leftPid,
    const Pid::PidGain& _straightAnglePid)
    : BaseMotion(
          _robot,
          std::move(_condition)),
      route(_route),
      currentSegment(0),
      targetSpeed(_targetSpeed),
      rotationPid(_rotationPid),
      rightPid(_rightPid),
      leftPid(_leftPid),
      straightAnglePid(_straightAnglePid)
{
  LOG_CREATE("RouteNavigation");
}

RouteNavigation::~RouteNavigation()
{
  LOG_DESTROY("RouteNavigation");
}

void RouteNavigation::prepare()
{
  currentSegment = 0;

  Logger::printfLog(
      Logger::INFO,
      "RouteNavigation: "
      "route size=%d",
      static_cast<int>(route.size()));
}

void RouteNavigation::executeStep()
{
  /*
   * 経路が空の場合
   */
  if(route.size() < 2) {
    return;
  }

  /*
   * 全区間終了
   */
  if(currentSegment >= route.size() - 1) {
    return;
  }

  /*
   * 現在のノード
   */
  const RouteState& current =
      route[currentSegment];

  /*
   * 次のノード
   */
  const RouteState& next =
      route[currentSegment + 1];

  Logger::printfLog(
      Logger::INFO,
      "RouteNavigation: "
      "segment=%d "
      "(%d,%d) -> (%d,%d)",
      static_cast<int>(currentSegment),
      current.x,
      current.y,
      next.x,
      next.y);

  /*
   * 1区間を走行
   */
  executeSegment(
      current,
      next);

  /*
   * 次の区間へ
   */
  currentSegment++;
}

void RouteNavigation::executeSegment(
    const RouteState& current,
    const RouteState& next)
{
  /*
   * Dijkstra座標
   *
   * 例:
   *
   * (0,0)
   * (2,0)
   * (4,0)
   *
   * を
   *
   * (0mm,0mm)
   * (208mm,0mm)
   * (416mm,0mm)
   *
   * のように変換する。
   */
  const double targetX =
      convertXToMm(next.x);

  const double targetY =
      convertYToMm(next.y);

  /*
   * GoalNavigationを1回だけ実行する。
   *
   * RepeatCountCondition(robot, 1)
   *
   * によってexecuteStep()を
   * 1回だけ呼ぶ。
   */
  auto condition =
      std::make_unique<RepeatCountCondition>(
          robot,
          1);

  GoalNavigation navigation(
      robot,
      std::move(condition),
      targetX,
      targetY,
      targetSpeed,
      rotationPid,
      rightPid,
      leftPid,
      straightAnglePid);

  navigation.run();
}

double RouteNavigation::convertXToMm(
    int x) const
{
  /*
   * Dijkstra座標をmmへ変換
   *
   * 0 → 0mm
   * 2 → 208mm
   * 4 → 416mm
   *
   * という変換。
   *
   * x / 2 でグリッド番号を求める。
   */
  const int gridX =
      x / 2;

  return gridX * GRID_SIZE_MM;
}

double RouteNavigation::convertYToMm(
    int y) const
{
  const int gridY =
      y / 2;

  return gridY * GRID_SIZE_MM;
}

void RouteNavigation::finish()
{
  robot
      .getWheelMotorControllerInstance()
      .stopBoth();
}