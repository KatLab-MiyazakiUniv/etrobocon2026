/**
 * @file   EtRobocon2026.cpp
 * @brief  全体を制御するクラス
 * @author HaruArima08
 */

#include "EtRobocon2026.h"

namespace {

  /**
   * @brief コース
   */
  enum class CourseSide { L_COURSE, R_COURSE };

  /**
   * @brief ここでコースを設定する
   */
  constexpr CourseSide COURSE_SIDE = CourseSide::L_COURSE;

  /**
   * @brief 最大周回数
   */
  constexpr int LAP_COUNT = 2;

  /**
   * @brief 最終地点へ向かう時間閾値[ms]
   *
   * 100秒
   */
  constexpr int FINAL_ROUTE_SWITCH_TIME_MS = 100000;

  /**
   * @brief 走行速度[mm/s]
   */
  constexpr double TARGET_SPEED = 300.0;

  /**
   * @brief 最終地点到着後の追加直進距離[mm]
   */
  constexpr double FINAL_STRAIGHT_DISTANCE = 100.0;

  /**
   * @brief 直進時のデッドバンド率
   */
  constexpr double STRAIGHT_DEADBAND_RATE = 0.25;

  /**
   * @brief 直進時の最大出力率
   */
  constexpr double STRAIGHT_MAXOUT_RATE = 0.54;

  /**
   * @brief Lコース座標を現在コース用へ変換する
   * @param point Lコース座標
   * @return 現在コース用へ変換した座標
   */
  Point convertPoint(const Point& point)
  {
    if(COURSE_SIDE == CourseSide::R_COURSE) {
      return CourseMirror::mirrorPoint(point);
    }

    return point;
  }

  /**
   * @brief Lコース方向を現在コース用へ変換する
   * @param direction Lコース方向
   * @return 現在コース用へ変換した方向
   */
  Direction convertDirection(Direction direction)
  {
    if(COURSE_SIDE == CourseSide::R_COURSE) {
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

void EtRobocon2026::start()
{
  Logger::info("Timed ET Rally start");

  RealNetworkSystem networkSystem;
  SocketClient cameraSocketClient(networkSystem);
  Robot robot(cameraSocketClient);

  robot.getCameraSocketClientInstance().connectToServer();

  // Robotが保持しているMapDataを使用する
  MapData& mapData = robot.getMapData();

  // RED
  mapData.setGatePoint(GoalColor::RED, convertPoint({ 1, 1 }), convertPoint({ 3, 1 }));

  // BLUE
  mapData.setGatePoint(GoalColor::BLUE, convertPoint({ 9, 1 }), convertPoint({ 9, 3 }));

  // YELLOW
  mapData.setGatePoint(GoalColor::YELLOW, convertPoint({ 5, 9 }), convertPoint({ 7, 9 }));

  GateRoutePlanner routePlanner(mapData);
  EtRallyMap etRallyMap;

  const Point startPoint = convertPoint({ 0, 4 });

  int currentGridX = startPoint.x;
  int currentGridY = startPoint.y;

  Direction currentDirection = convertDirection(Direction::LEFT);

  constexpr GoalColor TARGET_COLORS[] = { GoalColor::RED, GoalColor::BLUE, GoalColor::YELLOW };

  /**
   * @brief 通常回頭用PID
   *
   * 90度回頭などに使用する。
   */
  const Pid::PidGain rotationPid = { 1.4, 0.0, 0.1 };

  /**
   * @brief 正方形補正回頭用PID
   *
   * QR①への微調整、
   * QR①後の-α、
   * QR②への微調整に使用する。
   */
  const Pid::PidGain squareRotationPid = { 4.9, 0.0, 0.12 };

  /**
   * @brief 直進角度PID
   */
  const Pid::PidGain straightAnglePid = { 0.033, 0.003, 0.03 };

  RouteFollower routeFollower(robot, etRallyMap, TARGET_SPEED, rotationPid, squareRotationPid,
                              straightAnglePid, STRAIGHT_DEADBAND_RATE, STRAIGHT_MAXOUT_RATE);

  const int startTime = ClockUtil::now();

  bool shouldGoFinal = false;

  if(!hasAllGates(mapData)) {
    Logger::info("Gate information is incomplete -> go final");
    shouldGoFinal = true;
  }

  if(!shouldGoFinal) {
    for(int lap = 1; lap <= LAP_COUNT && !shouldGoFinal; ++lap) {
      for(const GoalColor targetColor : TARGET_COLORS) {
        Logger::printfLog(Logger::INFO, "Lap %d Target=%s", lap, colorToString(targetColor));

        GateRouteResult routeResult
            = routePlanner.search(currentGridX, currentGridY, currentDirection, targetColor);

        if(routeResult.route.size() < 2) {
          Logger::error("EtRobocon2026: route search failed");

          robot.getWheelMotorControllerInstance().stopBoth();

          return;
        }

        const Gate* targetGate = findGate(mapData, targetColor);

        if(targetGate == nullptr) {
          Logger::printfLog(Logger::ERROR, "EtRobocon2026: %s gate not found",
                            colorToString(targetColor));

          shouldGoFinal = true;

          break;
        }

        const bool outerGate = isOuterGate(*targetGate);

        routeFollower.run(routeResult.route);

        if(outerGate) {
          currentGridX = routeResult.entrance.x;
          currentGridY = routeResult.entrance.y;
          currentDirection = routeResult.exitDirection;

        } else {
          currentGridX = routeResult.exit.x;
          currentGridY = routeResult.exit.y;
          currentDirection = routeResult.exitDirection;
        }

        if(targetColor == GoalColor::YELLOW) {
          const int elapsedTime = ClockUtil::now() - startTime;

          if(lap >= LAP_COUNT) {
            Logger::info("Lap limit reached -> go final");

            shouldGoFinal = true;

            break;
          }

          if(elapsedTime >= FINAL_ROUTE_SWITCH_TIME_MS) {
            Logger::info("Time threshold reached -> go final");

            shouldGoFinal = true;

            break;
          }
        }
      }
    }
  }

  const Point finalPoint = convertPoint({ 8, 0 });
  const Direction finalDirection = convertDirection(Direction::LEFT);

  DijkstraRoutePlanner finalRoutePlanner(mapData.getGates());

  RouteResult finalRoute = finalRoutePlanner.search(currentGridX, currentGridY, currentDirection,
                                                    finalPoint, finalDirection);

  if(finalRoute.route.size() < 2) {
    Logger::error("EtRobocon2026: final route search failed");

    robot.getWheelMotorControllerInstance().stopBoth();

    return;
  }

  routeFollower.run(finalRoute.route);

  auto finalStraightCondition = std::make_unique<DistanceCondition>(robot, FINAL_STRAIGHT_DISTANCE);

  Straight finalStraight(robot, std::move(finalStraightCondition), TARGET_SPEED, straightAnglePid,
                         true, STRAIGHT_DEADBAND_RATE, STRAIGHT_MAXOUT_RATE);

  finalStraight.run();

  robot.getWheelMotorControllerInstance().stopBoth();

  Logger::info("Timed ET Rally finished");
}