/**
 * @file   EtRallyMotion.cpp
 * @brief  ETラリーの経路探索・継続周回・QR補正
 * @note   EtRobocon2026.cppのETラリー部分を移したもの
 */
#include "EtRallyMotion.h"

#include <cmath>
#include <functional>
#include <memory>
#include <utility>
#include <vector>

#include "AngleNormalizer.h"
#include "ClockUtil.h"
#include "Course.h"
#include "CourseMirror.h"
#include "DijkstraRoutePlanner.h"
#include "DistanceCondition.h"
#include "EtRallyMap.h"
#include "GateRoutePlanner.h"
#include "Logger.h"
#include "MapData.h"
#include "Pid.h"
#include "RelativeAngleCondition.h"
#include "RelativeRotation.h"
#include "RouteFollower.h"
#include "RouteTypes.h"
#include "SquareAngleAdjustment.h"
#include "Straight.h"
#include "SystemInfo.h"

namespace {

  // ETラリー全体を1回だけ実行するための継続条件。
  // CSV由来の外部条件により、走行前にスキップされることを防ぐ。
  class EtRallyCompletionCondition : public BaseContinuationCondition {
   public:
    EtRallyCompletionCondition(Robot& _robot, std::function<bool()> _notCompleted)
      : BaseContinuationCondition(_robot), notCompleted(std::move(_notCompleted))
    {
    }

    bool shouldContinue() override { return notCompleted(); }

   private:
    std::function<bool()> notCompleted;
  };

  /**
   * @brief 走行速度[mm/s]
   */
  constexpr double TARGET_SPEED = 300.0;
  /**
   * @brief 直進時のデッドバンド率
   */
  constexpr double STRAIGHT_DEADBAND_RATE = 0.25;
  /**
   * @brief 直進時の最大出力率
   */
  constexpr double STRAIGHT_MAXOUT_RATE = 0.54;
  /**
   * @brief 回頭終了判定許容誤差[deg]
   *
   * RouteFollowerと同じ値を使用する。
   */
  constexpr double ROTATION_TOLERANCE = 2.0;
  /**
   * @brief Lコース座標を現在コース用へ変換する
   * @param point Lコース座標
   * @return 現在コース用へ変換した座標
   */
  Point convertPoint(const Point& point, Course course)
  {
    if(course == Course::Right) {
      return CourseMirror::mirrorPoint(point);
    }
    return point;
  }
  /**
   * @brief Lコース方向を現在コース用へ変換する
   * @param direction Lコース方向
   * @return 現在コース用へ変換した方向
   */
  Direction convertDirection(Direction direction, Course course)
  {
    if(course == Course::Right) {
      return CourseMirror::mirrorDirection(direction);
    }
    return direction;
  }
  /**
   * @brief GoalColorを文字列へ変換する
   * @param color 変換するゲート色
   * @return GoalColorを表す文字列
   */
  const char* colorToString(GoalColor color)
  {
    switch(color) {
      case GoalColor::RED:
        return "RED";
      case GoalColor::BLUE:
        return "BLUE";
      case GoalColor::YELLOW:
        return "YELLOW";
    }
    return "UNKNOWN";
  }
  /**
   * @brief 外周ゲートか判定する
   * @param gate 判定するゲート
   * @return true 外周ゲート
   * @return false 内側ゲート
   */
  bool isOuterGate(const Gate& gate)
  {
    // 横向きゲート
    if(gate.start.y == gate.end.y) {
      const int gateY = gate.start.y;
      return gateY == 1 || gateY == SystemInfo::Y_GRID_NUM - 1;
    }
    // 縦向きゲート
    if(gate.start.x == gate.end.x) {
      const int gateX = gate.start.x;
      return gateX == 1 || gateX == SystemInfo::X_GRID_NUM - 1;
    }
    return false;
  }
  /**
   * @brief 指定した色のゲートを取得する
   * @param mapData マップ情報
   * @param color 取得するゲート色
   * @return 指定色のゲートへのポインタ
   * @return nullptr 指定色のゲートが存在しない場合
   */
  const Gate* findGate(const MapData& mapData, GoalColor color)
  {
    for(const Gate& gate : mapData.getGates()) {
      if(gate.color == color) {
        return &gate;
      }
    }
    return nullptr;
  }
  /**
   * @brief 必要な3色のゲート情報がすべて存在するか判定する
   * @param mapData マップ情報
   * @return true RED、BLUE、YELLOWがすべて存在する
   * @return false 1つ以上のゲート情報が存在しない
   */
  bool hasAllGates(const MapData& mapData)
  {
    return findGate(mapData, GoalColor::RED) != nullptr
           && findGate(mapData, GoalColor::BLUE) != nullptr
           && findGate(mapData, GoalColor::YELLOW) != nullptr;
  }
}  // namespace

EtRallyMotion::EtRallyMotion(Robot& _robot,
                             std::unique_ptr<BaseContinuationCondition> _continuationCondition,
                             const Pid::PidGain& _rotationPid,
                             const Pid::PidGain& _squareRotationPid,
                             const Pid::PidGain& _straightAnglePid)
  : BaseMotion(_robot, std::make_unique<EtRallyCompletionCondition>(
                           _robot, [this]() { return !completed; })),
    rotationPid(_rotationPid),
    squareRotationPid(_squareRotationPid),
    straightAnglePid(_straightAnglePid)
{
  // 元のETラリーには外部の継続条件がなかったため、
  // MotionParserから渡された条件は走行の開始・終了判定に使用しない。
  (void)_continuationCondition;
}

void EtRallyMotion::prepare()
{
  completed = false;
}

void EtRallyMotion::executeStep()
{
  // 走行処理はここで1回だけ実行する（10ms制御周期の動作ではない）。
  // 処理中にreturnした場合にもBaseMotion::run()が再実行しないよう先にtrueにする。
  completed = true;
  Logger::info("Timed ET Rally start");
  // ========================================
  // ETラリー準備
  // ========================================
  // Robotが保持しているMapDataを使用する
  const Course course = robot.getCourse();
  Logger::printfLog(Logger::INFO, "ET Rally course: %s",
                    course == Course::Right ? "Right" : "Left");

  MapData& mapData = robot.getMapDataInstance();
  GateRoutePlanner routePlanner(mapData);
  EtRallyMap etRallyMap;
  /**
   * @brief ET相撲終了後のETラリー開始位置（Lコース基準）
   *
   * ラベル番号によらず(10,2)に固定する。
   */
  const Point leftCourseStartPoint{ 10, 2 };
  // マップの範囲外で経路探索を実行しないようにする。
  if(leftCourseStartPoint.x < 0 || leftCourseStartPoint.x >= SystemInfo::X_GRID_NUM
     || leftCourseStartPoint.y < 0 || leftCourseStartPoint.y >= SystemInfo::Y_GRID_NUM) {
    Logger::error("EtRallyMotion: start point is outside the rally map");
    robot.getWheelMotorControllerInstance().stopBoth();
    return;
  }
  const Point startPoint = convertPoint(leftCourseStartPoint, course);
  int currentGridX = startPoint.x;
  int currentGridY = startPoint.y;
  /**
   * @brief ETラリー開始時の方向
   *
   * Lコースでは右向き（RIGHT）。Rコースは鏡映変換する。
   */
  Direction currentDirection = convertDirection(Direction::RIGHT, course);
  constexpr GoalColor TARGET_COLORS[] = { GoalColor::RED, GoalColor::BLUE, GoalColor::YELLOW };
  // PIDゲインはMotions/ETRally.csvからMotionParser経由で設定済み。
  // ========================================
  // ETラリー開始位置QR補正
  // ========================================
  Logger::info("ET Rally start square adjustment");
  CameraServer::SquareDetectorRequest request{};
  request.roi.x = 0;
  request.roi.y = 0;
  request.roi.width = CAM_MAX_WIDTH;
  request.roi.height = CAM_MAX_HEIGHT;
  SquareAngleAdjustment adjustment(robot);
  SquareAngleAdjustment::Result result = adjustment.calculate(request);
  if(result.wasDetected) {
    Logger::printfLog(Logger::INFO, "Start square detected: angle=%f distance=%f",
                      result.correctionAngle, result.straightDistance);
    // ----------------------------------------
    // QR方向へ回頭
    // ----------------------------------------
    auto rotateCondition = std::make_unique<RelativeAngleCondition>(robot, result.correctionAngle,
                                                                    ROTATION_TOLERANCE);
    RelativeRotation rotate(robot, std::move(rotateCondition), squareRotationPid,
                            result.correctionAngle);
    rotate.run();
    // ----------------------------------------
    // QRまで直進
    // ----------------------------------------
    auto straightCondition = std::make_unique<DistanceCondition>(robot, result.straightDistance);
    Straight straight(robot, std::move(straightCondition), TARGET_SPEED, straightAnglePid, true,
                      STRAIGHT_DEADBAND_RATE, STRAIGHT_MAXOUT_RATE);
    straight.run();
    // ----------------------------------------
    // 元のETラリー開始方向へ戻す
    // ----------------------------------------
    auto returnCondition = std::make_unique<RelativeAngleCondition>(robot, -result.correctionAngle,
                                                                    ROTATION_TOLERANCE);
    RelativeRotation returnRotate(robot, std::move(returnCondition), squareRotationPid,
                                  -result.correctionAngle);
    returnRotate.run();
    Logger::info("ET Rally start square adjustment finished");
  } else {
    /**
     * QR検出失敗時は補正を行わず、
     * そのまま処理を続行する。
     */
    Logger::info("ET Rally start square not detected -> skip adjustment");
  }
  // ========================================
  // RouteFollower生成
  // ========================================
  RouteFollower routeFollower(robot, etRallyMap, TARGET_SPEED, rotationPid, squareRotationPid,
                              straightAnglePid, STRAIGHT_DEADBAND_RATE, STRAIGHT_MAXOUT_RATE);
  // ========================================
  // ゲート情報の確認
  // ========================================
  // ゲート情報が足りない場合、最終地点には向かわず安全に終了する。
  if(!hasAllGates(mapData)) {
    Logger::error("EtRallyMotion: gate information is incomplete");
    robot.getWheelMotorControllerInstance().stopBoth();
    return;
  }

  // ========================================
  // ETラリーを継続して周回
  // ========================================
  // 赤 -> 青 -> 黄 -> 赤 -> ... の順に、終了時刻を設けず繰り返す。
  // 経路探索に失敗した場合は停止する。
  int lap = 1;
  while(true) {
    for(const GoalColor targetColor : TARGET_COLORS) {
      Logger::printfLog(Logger::INFO, "Lap %d Target=%s", lap, colorToString(targetColor));

      GateRouteResult routeResult
          = routePlanner.search(currentGridX, currentGridY, currentDirection, targetColor);
      if(routeResult.route.size() < 2) {
        Logger::error("EtRallyMotion: route search failed");
        robot.getWheelMotorControllerInstance().stopBoth();
        return;
      }

      const Gate* targetGate = findGate(mapData, targetColor);
      if(targetGate == nullptr) {
        Logger::error("EtRallyMotion: target gate not found");
        robot.getWheelMotorControllerInstance().stopBoth();
        return;
      }

      const bool outerGate = isOuterGate(*targetGate);
      routeFollower.setRoute(routeResult.route);
      routeFollower.run();

      // 外周ゲートではRouteFollowerが入口側に戻る。
      if(outerGate) {
        currentGridX = routeResult.entrance.x;
        currentGridY = routeResult.entrance.y;
        currentDirection = routeResult.exitDirection;
      } else {
        currentGridX = routeResult.exit.x;
        currentGridY = routeResult.exit.y;
        currentDirection = routeResult.exitDirection;
      }
    }
    ++lap;
  }
}

void EtRallyMotion::finish()
{
  robot.getWheelMotorControllerInstance().stopBoth();
}
