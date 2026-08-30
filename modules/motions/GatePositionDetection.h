/**
 * @file   GatePositionDetection.h
 * @brief  QRコードからゲート位置情報を取得する動作クラス
 * @author migaku2645
 */

#ifndef GATE_POSITION_DETECTION_H
#define GATE_POSITION_DETECTION_H

#include "BaseMotion.h"
#include "QrCodeDetector.h"

#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include "AesDecryptor.h"
#include "GatePositionParser.h"
#include "Logger.h"
#include "RepeatCountCondition.h"
#include "Snapshot.h"

class GatePositionDetection : public BaseMotion {
 public:
  /**
   * @brief コンストラクタ
   * @param _robot ロボットインスタンス
   * @param _fileName 撮影画像の保存ファイル名
   * @param _key AES復号キー
   * @param _roi QRコード検出範囲
   * @param continuationCondition 継続条件
   */
  GatePositionDetection(Robot& _robot, const std::string& _fileName, const std::string& _key,
                        const cv::Rect& _roi,
                        std::unique_ptr<BaseContinuationCondition> continuationCondition);

  /**
   * @brief デストラクタ
   */
  ~GatePositionDetection();

 protected:
  /**
   * @brief 写真撮影からゲート位置情報設定までを実行する
   */
  void executeStep() override;

 private:
  std::string fileName;
  std::string key;

  QrCodeDetector qrCodeDetector;
};

#endif