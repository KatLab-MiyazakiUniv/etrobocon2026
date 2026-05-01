/**
@file SystemInfo.h
@brief MotorControllerクラスの定数の定義情報を統一したファイル
@author nishijima515, sadomiya-sousi

*/

#ifndef SYSTEM_INFO_H
#define SYSTEM_INFO_H

#define PI 3.1415926           // 円周率
#define RAD_TO_DEG 180.0 / PI  // ラジアンを度数に変換するための係数
#define DEG_TO_RAD PI / 180.0  // 度数をラジアンに変換するための係数

static constexpr double WHEEL_RADIUS = 28.0;  // 車輪の半径[mm]

#endif SYSTEM_INFO_H