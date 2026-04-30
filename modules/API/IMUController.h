/**
 *@file IMUController.h
 *@brief IMUセンサを制御するラッパークラス
 *@author yutaro-1214
 */

#ifndef IMU_CONTROLLER_H
#define IMU_CONTROLLER_H

#include "IMU.h"

class IMUController {
 public:
  /**
   * コンストラクタ
   */
  IMUController();

  /**
   * @brief IMUの方位角を取得する
   */

  float getAzimuth();

 private:
  spikeapi::IMU imu;  // IMUインスタンス
};

#endif