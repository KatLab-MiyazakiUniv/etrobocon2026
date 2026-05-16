/**
 * @file ColorSensorController.h
 * @brief カラーセンサを制御するラッパークラス
 * @author sadomiya-sousi
 */

#ifndef COLOR_SENSOR_CONTROLLER_H
#define COLOR_SENSOR_CONTROLLER_H

#include "ColorSensor.h"

using namespace spikeapi;

class ColorSensorController {
 public:
  /**
   * コンストラクタ
   * @param port 接続ポート
   */
  ColorSensorController(EPort port = EPort::PORT_E);

  /**
   * @brief 反射光強度を取得する (0-100)
   * @return 反射光強度
   */
  int getReflection();

  /**
   * @brief 周囲の明るさを取得する (0-100)
   * @return 周囲の明るさ
   */
  int getAmbient();

  /**
   * @brief 生のRGB値を取得する
   * @param rgb [out] RGB値を格納する構造体
   */
  void getRawRGB(ColorSensor::RGB& rgb);

  /**
   * @brief HSV値を取得する (近似なし)
   * @param hsv [out] HSV値を格納する構造体
   */
  void getRawHSV(ColorSensor::HSV& hsv);

  /**
   * @brief ライトを点灯する (標準の白)
   */
  void lightOn();

  /**
   * @brief ライトを消灯する
   */
  void lightOff();

  /**
   * @brief ライトの色を個別に設定する
   * @param r 赤 (0-100)
   * @param g 緑 (0-100)
   * @param b 青 (0-100)
   */
  void setLightColor(int r, int g, int b);

 private:
  ColorSensor colorSensor;  // カラーセンサインスタンス
};

#endif
