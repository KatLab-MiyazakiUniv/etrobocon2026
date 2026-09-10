/**
 * @file   RouteFollower.h
 * @brief  経路探索結果に従って自己位置を使用せず走行するクラス
 * @author yutaro-1214
 */

#ifndef ROUTE_FOLLOWER_H
#define ROUTE_FOLLOWER_H

#include <vector>

#include "EtRallyMap.h"
#include "MapData.h"
#include "Pid.h"
#include "Robot.h"
#include "RouteTypes.h"

/**
 * @brief 経路探索結果に従って走行するクラス
 *
 * RouteStateで表された経路に従って、
 * 回頭と直進を組み合わせて走行する。
 *
 * 通常区間では、
 * RelativeRotationによる回頭とStraightによる直進を行う。
 *
 * 内側ゲートを通過する区間では、
 * ゲート手前でSquareAngleAdjustmentによる
 * 正方形を利用した角度補正を2回行い、
 * その後Straightでゲートを通過する。
 *
 * 外周ゲートでは正方形補正を行わず、
 * 通常のStraightで通過する。
 */
class RouteFollower {
 public:
  /**
   * @brief コンストラクタ
   *
   * @param _robot ロボットインスタンス
   * @param _map ETラリーマップ
   * @param _mapData ゲート情報を保持するマップデータ
   * @param _targetSpeed 直進時の目標速度[mm/s]
   * @param _rotationPid 回頭用PIDゲイン
   * @param _rightPid 右モータ速度制御用PIDゲイン
   * @param _leftPid 左モータ速度制御用PIDゲイン
   * @param _straightAnglePid 直進時角度補正用PIDゲイン
   * @param _squareAnglePid 正方形中心補正用PIDゲイン
   */
  RouteFollower(Robot& _robot, const EtRallyMap& _map, const MapData& _mapData, double _targetSpeed,
                const Pid::PidGain& _rotationPid, const Pid::PidGain& _rightPid,
                const Pid::PidGain& _leftPid, const Pid::PidGain& _straightAnglePid,
                const Pid::PidGain& _squareAnglePid);

  /**
   * @brief 経路探索結果に従って走行する
   *
   * route[0]を現在位置として、
   * route[1]以降へ順番に移動する。
   *
   * @param route 経路探索結果
   */
  void run(const std::vector<RouteState>& route);

 private:
  /**
   * @brief Directionを角度へ変換する
   *
   * RIGHTを0度として以下の角度へ変換する。
   *
   * RIGHT :   0度
   * UP    :  90度
   * LEFT  : 180度
   * DOWN  : -90度
   *
   * @param direction 方向
   * @return 角度[deg]
   */
  double directionToHeading(Direction direction) const;

  /**
   * @brief 現在方向から目標方向までの回頭角度を計算する
   *
   * @param from 現在方向
   * @param to 目標方向
   * @return 回頭角度[deg]
   */
  double calculateRotationAngle(Direction from, Direction to) const;

  /**
   * @brief 2つのRouteState間の実距離を計算する
   *
   * EtRallyMapのノード座標から、
   * X方向またはY方向の移動距離を計算する。
   *
   * 斜め方向の移動の場合は0.0を返す。
   *
   * @param from 移動開始状態
   * @param to 移動終了状態
   * @return 移動距離[mm]
   */
  double calculateDistance(const RouteState& from, const RouteState& to) const;

  /**
   * @brief 指定された角度だけ回頭する
   *
   * RelativeRotationを使用して回頭する。
   *
   * @param angle 回頭角度[deg]
   */
  void rotate(double angle);

  /**
   * @brief 指定距離だけ直進する
   *
   * Straightを使用して直進する。
   *
   * @param distance 直進距離[mm]
   */
  void straight(double distance);

  /**
   * @brief 正方形を利用してロボットの向きを補正する
   *
   * SquareAngleAdjustmentを使用し、
   * 検出した正方形の中心が画像中央に来るように
   * 左右へ回頭して向きを調整する。
   *
   * @return true 補正成功
   * @return false 正方形未検出または通信失敗
   */
  bool adjustAngleWithSquare();

  /**
   * @brief 指定区間を通過するゲートを取得する
   *
   * MapDataに登録されているGatePassと、
   * from -> toの区間を比較する。
   *
   * @param from 区間開始状態
   * @param to 区間終了状態
   * @return 該当するゲート
   * @return nullptr ゲート区間ではない
   */
  const Gate* findGate(const RouteState& from, const RouteState& to) const;

  /**
   * @brief 指定されたゲートが外周ゲートか判定する
   *
   * マップ上端、下端、左端、右端に存在するゲートを
   * 外周ゲートとして扱う。
   *
   * @param gate 判定対象ゲート
   * @return true 外周ゲート
   * @return false 内側ゲート
   */
  bool isOuterGate(const Gate& gate) const;

  /**
   * @brief 区間開始地点からゲートまでの距離を計算する
   *
   * 横向きゲートの場合はY方向の距離、
   * 縦向きゲートの場合はX方向の距離を使用する。
   *
   * @param from 区間開始状態
   * @param gate 通過するゲート
   * @return ゲートまでの距離[mm]
   */
  double calculateDistanceToGate(const RouteState& from, const Gate& gate) const;

  /**
   * @brief ゲート通過区間を走行する
   *
   * 内側ゲートでは、
   *
   * 1. ゲート375mm手前までStraight
   * 2. 1回目のSquareAngleAdjustment
   * 3. ゲート125mm手前までStraight
   * 4. 2回目のSquareAngleAdjustment
   * 5. 残りをStraight
   *
   * の順番で走行する。
   *
   * 外周ゲートでは正方形補正を行わず、
   * 通常のStraightで通過する。
   *
   * @param from 区間開始状態
   * @param to 区間終了状態
   * @param distance 区間全体の距離[mm]
   */
  void runGateSegment(const RouteState& from, const RouteState& to, double distance);

 private:
  /**
   * @brief ロボットインスタンス
   */
  Robot& robot;

  /**
   * @brief ETラリーの座標マップ
   *
   * グリッド座標から実距離を取得するために使用する。
   */
  const EtRallyMap& map;

  /**
   * @brief ゲート情報
   *
   * GateとGatePassを取得し、
   * 現在の走行区間がゲート通過区間か判定するために使用する。
   */
  const MapData& mapData;

  /**
   * @brief Straightの目標速度[mm/s]
   */
  double targetSpeed;

  /**
   * @brief 回頭用PIDゲイン
   */
  Pid::PidGain rotationPid;

  /**
   * @brief 右モータ速度制御用PIDゲイン
   */
  Pid::PidGain rightPid;

  /**
   * @brief 左モータ速度制御用PIDゲイン
   */
  Pid::PidGain leftPid;

  /**
   * @brief Straight角度補正用PIDゲイン
   */
  Pid::PidGain straightAnglePid;

  /**
   * @brief 正方形中心補正用PIDゲイン
   */
  Pid::PidGain squareAnglePid;
};

#endif