/**
 * @file   EtRallyMotion.h
 * @brief  ETラリー全体の動作を管理するクラス
 * @author okuyama0528
 */

#ifndef ET_RALLY_MOTION_H
#define ET_RALLY_MOTION_H

#include <cmath>
#include <functional>
#include <memory>
#include <utility>
#include <vector>
#include "AngleNormalizer.h"
#include "BaseMotion.h"
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

class EtRallyMotion : public BaseMotion {
 public:
  /**
   * @brief コンストラクタ
   * @param _robot ロボット
   * @param _continuationCondition 継続条件
   * @param _rotationPid 通常回頭PID
   * @param _squareRotationPid 正方形補正回頭PID
   * @param _straightAnglePid 直進角度PID
   * @param _targetSpeed 走行速度
   * @param _straightDeadbandRate 直進デッドバンド率
   * @param _straightMaxoutRate 直進マックスアウト率
   */
  EtRallyMotion(Robot& _robot, std::unique_ptr<BaseContinuationCondition> _continuationCondition,
                const Pid::PidGain& _rotationPid, const Pid::PidGain& _squareRotationPid,
                const Pid::PidGain& _straightAnglePid, double _targetSpeed,
                double _straightDeadbandRate, double _straightMaxoutRate);

 protected:
  /**
   * @brief 走行前の準備処理
   */
  void prepare() override;

  /**
   * @brief 走行処理
   */
  void executeStep() override;

  /**
   * @brief 走行後の終了処理
   */
  void finish() override;

 private:
  /**
   * @brief 指定した座標をコースに応じて変換する
   * @param point 変換する座標
   * @param course コース
   * @return 変換後の座標
   */
  static Point convertPoint(const Point& point, Course course);

  /**
   * @brief 指定した方向をコースに応じて変換する
   * @param direction 変換する方向
   * @param course コース
   * @return 変換後の方向
   */
  static Direction convertDirection(Direction direction, Course course);

  /**
   * @brief デバッグ用にGoalColorを文字列へ変換する
   * @param color ゲートの色
   * @return ゲート色を表す文字列
   */
  static const char* colorToString(GoalColor color);

  /**
   * @brief 指定した色のゲートを取得する
   * @param mapData マップ情報
   * @param color 取得するゲート色
   * @return 指定色のゲートへのポインタ
   */
  static const Gate* findGate(const MapData& mapData, GoalColor color);

  /**
   * @brief 必要な3色のゲート情報が存在するか判定する
   * @param mapData マップ情報
   * @return true 3色すべて存在する
   * @return false 1色以上不足している
   */
  static bool hasAllGates(const MapData& mapData);

  Pid::PidGain rotationPid;        // 90度回頭などの通常回頭PID
  Pid::PidGain squareRotationPid;  // QR補正などの調整回頭PID
  Pid::PidGain straightAnglePid;   // 直進角度PID
  double targetSpeed;              // 走行速度
  double straightDeadbandRate;     // 直進時のデッドバンド率
  double straightMaxoutRate;       // 直進時の最大出力率
};

#endif  // ET_RALLY_MOTION_H
