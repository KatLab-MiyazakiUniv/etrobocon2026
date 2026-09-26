/**
 * @file   EtRobocon2026.cpp
 * @brief  全体を制御するクラス
 * @author HaruArima08
 */

#include "EtRobocon2026.h"

#include <cmath>
#include <memory>
#include <vector>

#include "AngleNormalizer.h"

namespace {

  /**
   * @brief コース
   */
  enum class CourseSide { L_COURSE, R_COURSE };

  /**
   * @brief ここでコースを設定する
   *
   * LコースならL_COURSE
   * RコースならR_COURSE
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

  /**
   * @brief Directionを角度へ変換する
   * @param direction 方向
   * @return 方向に対応する角度[deg]
   */
  double directionToHeading(Direction direction)
  {
    switch(direction) {
      case Direction::RIGHT:
        return 0.0;

      case Direction::UP:
        return 90.0;

      case Direction::LEFT:
        return 180.0;

      case Direction::DOWN:
        return -90.0;
    }

    return 0.0;
  }

  /**
   * @brief 2つの方向から必要な相対回頭角度を計算する
   * @param from 現在方向
   * @param to 目標方向
   * @return 相対回頭角度[deg]
   */
  double calculateRotationAngle(Direction from, Direction to)
  {
    const double currentHeading = directionToHeading(from);
    const double targetHeading = directionToHeading(to);

    return AngleNormalizer::normalizeAngle(currentHeading - targetHeading);
  }

  /**
   * @brief 2地点間を1回の直進で移動する場合の距離を計算する
   * @param map ETラリーマップ
   * @param from 開始格子座標
   * @param to 終了格子座標
   * @return 直進距離[mm]
   * @return 0.0 縦横一直線ではない場合
   */
  double calculateStraightDistance(const EtRallyMap& map, const Point& from, const Point& to)
  {
    const EtRallyMap::Node fromNode = map.getNode(from.x, from.y);
    const EtRallyMap::Node toNode = map.getNode(to.x, to.y);

    // Y方向への直進
    if(from.x == to.x) {
      return std::abs(toNode.y - fromNode.y);
    }

    // X方向への直進
    if(from.y == to.y) {
      return std::abs(toNode.x - fromNode.x);
    }

    Logger::printfLog(Logger::ERROR, "EtRobocon2026: diagonal straight route (%d,%d) -> (%d,%d)",
                      from.x, from.y, to.x, to.y);

    return 0.0;
  }

}  // namespace

void EtRobocon2026::start()
{
  // CsvLogger::init();
  // CsvLogger::writeHeader();

  Logger::info("Hello KATLAB");

  RealNetworkSystem real;
  SocketClient client(real);
  Robot robot(client);

  robot.getCameraSocketClientInstance().connectToServer();

  // int voltage = BatteryController::getVoltage();
  // Logger::printfLog(Logger::INFO, "バッテリー電圧: %d mV", voltage);

  // ========================================
  // コース設定
  // ========================================

  // Rコース
  // robot.setCourse(Course::Right);
  // robot.setEdge(Edge::LeftEdge);

  // Lコース
  robot.setCourse(Course::Left);
  robot.setEdge(Edge::RightEdge);

  // ========================================
  // ライントレース走行
  // ========================================

  Area lineTraceArea = Area::LineTrace;
  AreaMaster lineTraceAreaMaster(robot, lineTraceArea);
  lineTraceAreaMaster.run();

  // ========================================
  // ボトルデリバリー走行
  // ========================================

  Area bottleDeliveryArea = Area::BottleDelivery;
  AreaMaster bottleDeliveryAreaMaster(robot, bottleDeliveryArea);
  bottleDeliveryAreaMaster.run();

  // CsvLogger::outputToFile();

  Logger::info("Timed ET Rally start");

  // ========================================
  // ETラリー準備
  // ========================================

  // Robotが保持しているMapDataを使用する
  MapData& mapData = robot.getMapDataInstance();

  GateRoutePlanner routePlanner(mapData);
  EtRallyMap etRallyMap;

  /**
   * @brief QR補正後の開始位置
   */
  Point startPoint;

  /**
   * @brief QR補正前の外周位置
   *
   * ゲート情報不足時はQR補正後にここまでバックする。
   */
  Point outerPoint;

  /**
   * @brief ラベル位置から開始位置と外周位置を決定する
   */
  switch(robot.getIndexOfLabel()) {
    case 0:
      startPoint = convertPoint({ 2, 4 });
      outerPoint = convertPoint({ 0, 4 });
      break;

    case 1:
      startPoint = convertPoint({ 2, 6 });
      outerPoint = convertPoint({ 0, 6 });
      break;

    case 2:
      startPoint = convertPoint({ 2, 8 });
      outerPoint = convertPoint({ 0, 8 });
      break;

    default:
      Logger::error("EtRobocon2026: invalid label index");

      robot.getWheelMotorControllerInstance().stopBoth();

      return;
  }

  int currentGridX = startPoint.x;
  int currentGridY = startPoint.y;

  /**
   * @brief ETラリー開始時の方向
   *
   * LコースではX増加方向へ進むLEFT。
   */
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
   * QR②への微調整などに使用する。
   */
  const Pid::PidGain squareRotationPid = { 4.9, 0.0, 0.12 };

  /**
   * @brief 直進角度PID
   */
  const Pid::PidGain straightAnglePid = { 0.033, 0.003, 0.03 };

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

  /**
   * @brief ETラリー走行時間計測開始
   *
   * 開始位置QR補正に使用した時間は含めない。
   */
  const int startTime = ClockUtil::now();

  bool shouldGoFinal = false;

  /**
   * @brief ゲート情報不足時に固定最終経路を使用するか
   *
   * trueの場合は、
   *
   * QR補正位置
   *   ↓ バック
   * 外周
   *   ↓ 1回のStraight
   * (0,0)
   *   ↓ 1回のStraight
   * (8,0)
   *
   * の順番で走行する。
   */
  bool useFixedFinalRoute = false;

  // ========================================
  // ゲート情報確認
  // ========================================

  if(!hasAllGates(mapData)) {
    Logger::info("Gate information is incomplete -> use fixed final route");

    shouldGoFinal = true;
    useFixedFinalRoute = true;

    // ========================================
    // QR補正後に外周までバック
    // ========================================

    if(result.wasDetected) {
      const double backDistance = calculateStraightDistance(etRallyMap, startPoint, outerPoint);

      if(backDistance <= 0.0) {
        Logger::error("EtRobocon2026: invalid back distance");

        robot.getWheelMotorControllerInstance().stopBoth();

        return;
      }

      Logger::printfLog(Logger::INFO, "Back to outer grid: %.2f mm", backDistance);

      auto backCondition = std::make_unique<DistanceCondition>(robot, backDistance);

      /**
       * 負の速度を指定することで、
       * LEFT方向を向いたまま外周へバックする。
       */
      Straight backStraight(robot, std::move(backCondition), -TARGET_SPEED, straightAnglePid, true,
                            STRAIGHT_DEADBAND_RATE, STRAIGHT_MAXOUT_RATE);

      backStraight.run();

      Logger::info("Back to outer grid finished");

    } else {
      /**
       * QR補正を行っていない場合は、
       * 開始前の外周位置にいるものとして扱う。
       */
      Logger::info("Start square not detected -> no back required");
    }

    /**
     * 固定経路開始位置を外周へ更新する。
     *
     * バックでは向きを変更しないので、
     * currentDirectionはLEFTのまま。
     */
    currentGridX = outerPoint.x;
    currentGridY = outerPoint.y;
    currentDirection = convertDirection(Direction::LEFT);
  }

  // ========================================
  // 通常ETラリー走行
  // ========================================

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

        /**
         * 外周ゲートの場合は、
         * RouteFollower内部で入口側へ戻ってくるため、
         * entranceを現在位置とする。
         */
        if(outerGate) {
          currentGridX = routeResult.entrance.x;
          currentGridY = routeResult.entrance.y;
          currentDirection = routeResult.exitDirection;

        } else {
          currentGridX = routeResult.exit.x;
          currentGridY = routeResult.exit.y;
          currentDirection = routeResult.exitDirection;
        }

        // ----------------------------------------
        // YELLOW通過後に終了条件を確認
        // ----------------------------------------

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

  // ========================================
  // 最終地点
  // ========================================

  const Point finalPoint = convertPoint({ 8, 0 });

  const Direction finalDirection = convertDirection(Direction::LEFT);

  // ========================================
  // ゲート情報不足時の固定経路
  // ========================================

  if(useFixedFinalRoute) {
    /**
     * 固定経路
     *
     * 外周位置
     *   ↓
     * UPへ回頭
     *   ↓
     * Straight 1回
     *   ↓
     * (0,0)
     *   ↓
     * LEFTへ回頭
     *   ↓
     * Straight 1回
     *   ↓
     * (8,0)
     */

    const Point turnPoint = convertPoint({ 0, 0 });

    const Direction turnPointDirection = convertDirection(Direction::UP);

    // ========================================
    // 外周位置でUPへ回頭
    // ========================================

    const double rotateToUpAngle = calculateRotationAngle(currentDirection, turnPointDirection);

    Logger::printfLog(Logger::INFO, "Fixed route: rotation to UP %.2f deg", rotateToUpAngle);

    if(std::abs(rotateToUpAngle) > ROTATION_TOLERANCE) {
      auto rotationCondition
          = std::make_unique<RelativeAngleCondition>(robot, rotateToUpAngle, ROTATION_TOLERANCE);

      RelativeRotation rotation(robot, std::move(rotationCondition), rotationPid, rotateToUpAngle);

      rotation.run();
    }

    currentDirection = turnPointDirection;

    // ========================================
    // 外周位置 -> (0,0)
    //
    // ここはStraightを1回だけ実行する。
    // ========================================

    const Point currentPoint{ currentGridX, currentGridY };

    const double distanceToTurnPoint
        = calculateStraightDistance(etRallyMap, currentPoint, turnPoint);

    if(distanceToTurnPoint <= 0.0) {
      Logger::error("EtRobocon2026: invalid distance to (0,0)");

      robot.getWheelMotorControllerInstance().stopBoth();

      return;
    }

    Logger::printfLog(Logger::INFO, "Fixed route: Straight (%d,%d) -> (0,0) %.2f mm", currentGridX,
                      currentGridY, distanceToTurnPoint);

    auto toTurnPointCondition = std::make_unique<DistanceCondition>(robot, distanceToTurnPoint);

    Straight toTurnPointStraight(robot, std::move(toTurnPointCondition), TARGET_SPEED,
                                 straightAnglePid, true, STRAIGHT_DEADBAND_RATE,
                                 STRAIGHT_MAXOUT_RATE);

    /**
     * 外周から(0,0)まで、
     * マスごとに分割せず1回のStraightで走行する。
     */
    toTurnPointStraight.run();

    currentGridX = turnPoint.x;
    currentGridY = turnPoint.y;
    currentDirection = turnPointDirection;

    // ========================================
    // (0,0)でLEFTへ回頭
    // ========================================

    const double rotateToFinalAngle = calculateRotationAngle(currentDirection, finalDirection);

    Logger::printfLog(Logger::INFO, "Fixed route: rotation at (0,0) %.2f deg", rotateToFinalAngle);

    if(std::abs(rotateToFinalAngle) > ROTATION_TOLERANCE) {
      auto rotationCondition
          = std::make_unique<RelativeAngleCondition>(robot, rotateToFinalAngle, ROTATION_TOLERANCE);

      RelativeRotation rotation(robot, std::move(rotationCondition), rotationPid,
                                rotateToFinalAngle);

      rotation.run();
    }

    currentDirection = finalDirection;

    // ========================================
    // (0,0) -> (8,0)
    //
    // ここもStraightを1回だけ実行する。
    // ========================================

    const double distanceToFinal = calculateStraightDistance(etRallyMap, turnPoint, finalPoint);

    if(distanceToFinal <= 0.0) {
      Logger::error("EtRobocon2026: invalid distance to (8,0)");

      robot.getWheelMotorControllerInstance().stopBoth();

      return;
    }

    Logger::printfLog(Logger::INFO, "Fixed route: Straight (0,0) -> (8,0) %.2f mm",
                      distanceToFinal);

    auto toFinalCondition = std::make_unique<DistanceCondition>(robot, distanceToFinal);

    Straight toFinalStraight(robot, std::move(toFinalCondition), TARGET_SPEED, straightAnglePid,
                             true, STRAIGHT_DEADBAND_RATE, STRAIGHT_MAXOUT_RATE);

    /**
     * (0,0)から(8,0)まで、
     * マスごとに分割せず1回のStraightで走行する。
     */
    toFinalStraight.run();

    currentGridX = finalPoint.x;
    currentGridY = finalPoint.y;
    currentDirection = finalDirection;

  } else {
    // ========================================
    // 通常時の最終地点への経路探索
    // ========================================

    DijkstraRoutePlanner finalRoutePlanner(mapData.getGates());

    RouteResult finalRoute = finalRoutePlanner.search(currentGridX, currentGridY, currentDirection,
                                                      finalPoint, finalDirection);

    if(finalRoute.route.size() < 2) {
      Logger::error("EtRobocon2026: final route search failed");

      robot.getWheelMotorControllerInstance().stopBoth();

      return;
    }

    routeFollower.run(finalRoute.route);
  }

  // ========================================
  // 最終地点到着後の追加直進
  // ========================================

  auto finalStraightCondition = std::make_unique<DistanceCondition>(robot, FINAL_STRAIGHT_DISTANCE);

  Straight finalStraight(robot, std::move(finalStraightCondition), TARGET_SPEED, straightAnglePid,
                         true, STRAIGHT_DEADBAND_RATE, STRAIGHT_MAXOUT_RATE);

  finalStraight.run();

  robot.getWheelMotorControllerInstance().stopBoth();

  Logger::info("Timed ET Rally finished");

  // ========================================
  // ET相撲走行
  // ========================================

  Area ETZumo = Area::ETZumo;

  AreaMaster ETZumouAreaMaster(robot, ETZumo);

  ETZumouAreaMaster.run();
}