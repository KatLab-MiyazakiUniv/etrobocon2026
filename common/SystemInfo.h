/**
 * @file SystemInfo.h
 * @brief システム全体の定義情報を統一したファイル
 * @author sadomiya-sousi
 */

#ifndef SYSTEM_INFO_H
#define SYSTEM_INFO_H

static constexpr double PI = 3.1415926;           // 円周率
static constexpr double RAD_TO_DEG = 180.0 / PI;  // ラジアンを度数に変換するための係数
static constexpr double DEG_TO_RAD = PI / 180.0;  // 度数をラジアンに変換するための係数
static constexpr double WHEEL_RADIUS = 28.0;      // 車輪の半径[mm]
static constexpr int MOTOR_POWER_MAX = 100;       // モータのPower値の上限値
static constexpr int MOTOR_POWER_MIN = -100;      // モータのPower値の下限値
static constexpr int CAM_MIN_WIDTH = 320;         // カメラ撮影解像度の幅（最小値）[px]
static constexpr int CAM_MIN_HEIGHT = 240;        // カメラ撮影解像度の高さ（最小値）[px]
static constexpr int CAM_MAX_WIDTH = 1920;        // カメラ撮影解像度の幅（最大値）[px]
static constexpr int CAM_MAX_HEIGHT = 1080;       // カメラ撮影解像度の高さ（最大値）[px]

#endif
