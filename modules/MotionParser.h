/**
 * @file   MotionParser.h
 * @brief  動作コマンドファイルを解析するクラス
 * @author nishijima515
 */

#ifndef MOTION_PARSER_H
#define MOTION_PARSER_H

#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "BaseContinuationCondition.h"
#include "BaseMotion.h"
#include "DistanceCondition.h"
#include "Logger.h"
#include "Robot.h"

class MotionParser {
 public:
  /**
   * @brief 動作コマンドの種類
   */
  enum class MOTION_COMMAND {
    NONE,

    /**
     * @brief 通常直進
     */
    STRAIGHT,

    /**
     * @brief QRコード追従
     */
    QR_TRACKING,

    /**
     * @brief 正方形追従
     */
    SQUARE_TRACKING
  };

  /**
   * @brief 継続条件コマンドの種類
   */
  enum class CONDITION_COMMAND {
    NONE,

    /**
     * @brief 距離条件
     */
    DISTANCE
  };

  /**
   * @brief AreaコマンドCSVから動作リストを生成する
   *
   * AreaコマンドCSVに記述された
   *
   * motionName,
   * motionId,
   * conditionName,
   * conditionId
   *
   * を読み込み、対応する動作インスタンスを生成する。
   *
   * @param robot ロボット
   * @param commandFilePath AreaコマンドCSVのパス
   * @return 生成した動作リスト
   */
  static std::vector<BaseMotion*> createMotionList(
      Robot& robot,
      std::string& commandFilePath);

 private:
  /**
   * @brief CSVの区切り文字
   */
  static constexpr char SEPARATOR = ',';

  /**
   * @brief 指定されたCSVからIDに対応する行を取得する
   *
   * CSVの2列目をIDとして検索する。
   *
   * @param filePath CSVファイルのパス
   * @param id 検索するID
   * @return IDに対応するパラメータ一覧
   */
  static std::vector<std::string> extractParamsFromID(
      const std::string& filePath,
      const std::string& id);

  /**
   * @brief 継続条件インスタンスを生成する
   *
   * @param robot ロボット
   * @param params 条件CSVから取得したパラメータ
   * @return 継続条件インスタンス
   */
  static std::unique_ptr<BaseContinuationCondition>
      createConditionInstance(
          Robot& robot,
          const std::vector<std::string>& params);

  /**
   * @brief 動作インスタンスを生成する
   *
   * @param robot ロボット
   * @param motionParams 動作CSVから取得したパラメータ
   * @param condition 継続条件
   * @return 動作インスタンス
   */
  static BaseMotion* createMotionInstance(
      Robot& robot,
      const std::vector<std::string>& motionParams,
      std::unique_ptr<BaseContinuationCondition> condition);

  /**
   * @brief 動作コマンド文字列を列挙型へ変換する
   *
   * 例:
   *
   * "Straight"
   *     → MOTION_COMMAND::STRAIGHT
   *
   * "QRTracking"
   *     → MOTION_COMMAND::QR_TRACKING
   *
   * "SquareTracking"
   *     → MOTION_COMMAND::SQUARE_TRACKING
   *
   * @param str 動作コマンド名
   * @return 動作コマンド
   */
  static MOTION_COMMAND convertCommand(
      const std::string& str);

  /**
   * @brief 条件コマンド文字列を列挙型へ変換する
   *
   * 例:
   *
   * "Distance"
   *     → CONDITION_COMMAND::DISTANCE
   *
   * @param str 条件コマンド名
   * @return 条件コマンド
   */
  static CONDITION_COMMAND convertCondition(
      const std::string& str);
};

#endif