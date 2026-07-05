/**
 * @file   Snapshot.h
 * @brief  サーバーに写真撮影を依頼するクラス
 * @author sadomiya-sousi
 */

#ifndef SNAPSHOT_H
#define SNAPSHOT_H

#include "BaseMotion.h"
#include "RepeatCountCondition.h"
#include "Logger.h"
#include "SocketClient.h"
#include <memory>
#include <string>
#include <cstring>

class Snapshot : public BaseMotion {
 public:
  /**
   * コンストラクタ
   * @param _robot ロボットインスタンス
   * @param _fileName 保存ファイル名
   * @param continuationCondition 継続条件
   */
  Snapshot(Robot& _robot, const std::string& _fileName,
           std::unique_ptr<BaseContinuationCondition> continuationCondition);

  /**
 * デストラクタ
 */
  ~Snapshot();

 protected:
  /**
   * @brief 1周期分の撮影処理
   */
  void executeStep() override;

 private:
  std::string fileName;  // 保存する画像ファイル名
};

#endif