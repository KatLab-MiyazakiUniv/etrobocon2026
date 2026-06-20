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

#define PRESS_POWER 0.5f

class Calibrator {
 public:
  /**
   * コンストラクタ
   */
  Calibrator(Robot& _robot);

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

 private:
  Robot& robot;  // Robotインスタンスの参照
};

#endif