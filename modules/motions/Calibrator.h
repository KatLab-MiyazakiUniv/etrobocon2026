/**
 * @file Calibrator.h
 * @brief キャリブレーションからスタートまでを担当するクラス
 * @author okuyama0528 sadomiya-sousi
 */

#ifndef CALIBRATOR_H
#define CALIBRATOR_H

#include "Robot.h"
#include "ClockUtil.h"
#include "Logger.h"
#include "Snapshot.h"
#include "Course.h"
#include "BaseMotion.h"
#include "RepeatCountCondition.h"
#include <thread>
#include <chrono>
#include <iostream>
#include <cstring>
#include <fstream>

#define PRESS_POWER 0.5f

class Calibrator : public BaseMotion {
 public:
  /**
   * コンストラクタ
   * @param _robot ロボットインスタンス
   * @param continuationCondition 継続条件
   */
  Calibrator(Robot& _robot, std::unique_ptr<BaseContinuationCondition> condition);

  /**
  * デストラクタ
  */
  ~Calibrator();

  /**
   * @brief 左右ボタンでLRコースを選択してisLeftCourseをセットする
   */
  void selectAndSetCourse();

  /**
   * @brief 黒と白の輝度を測定して目標輝度を求めtargetBrightnessをセットする
   */
  void measureAndSetTargetBrightness();

  /**
   * @brief カメラ角度調整用のフレーム取得をする
   */
  void getAngleCheckFrame();

  /**
   * @brief スタート合図が出るまで待機状態にする
   */
  void waitForStart();

  /**
   * @brief 4文字の復号キーをPCサーバーから通信で取得して，メンバ変数に保存する
   */
  void inputAndSetDecryptionKey();

  /**
   * @brief 4文字の復号キーをメンバ変数に保存する
   * @param key 4文字の復号キー
   */
  void setDecryptionKey(const std::string& key);

 private:
  void executeStep() override;  // 処理を1回実行する
};
#endif