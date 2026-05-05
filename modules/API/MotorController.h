/**
 *@file MotorController.h
 *@brief MotorControllerクラスの定数の定義情報を統一したファイル
 *@author sadomiya-sousi
 */

#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

static constexpr double PI = 3.1415926;           // 円周率
static constexpr double RAD_TO_DEG = 180.0 / PI;  // ラジアンを度数に変換するための係数
static constexpr double DEG_TO_RAD = PI / 180.0;  // 度数をラジアンに変換するための係数
static constexpr double WHEEL_RADIUS = 28.0;      // 車輪の半径[mm]

#endif
