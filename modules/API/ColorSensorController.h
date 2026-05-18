/**
 * @file ColorSensorController.h
 * @brief カラーセンサを制御するラッパークラス
 * @author sadomiya-sousi
 */

#ifndef COLOR_SENSOR_CONTROLLER_H
#define COLOR_SENSOR_CONTROLLER_H

#include "ColorSensor.h"
#include <cstdint>   // uint16_t, uint8_tを使用するため
#include <iostream>  // エラー出力用

class ColorSensorController {
 public:
  /**
   * @brief RGB値を保持する構造体
   */
  struct RGB {
    uint16_t r;
    uint16_t g;
    uint16_t b;
  };

  /**
   * @brief HSV値を保持する構造体
   */
  struct HSV {
    uint16_t h;
    uint8_t s;
    uint8_t v;
  };

  /**
   * コンストラクタ
   * @param port 接続ポート
   */
  ColorSensorController(EPort port = EPort::PORT_E);

  /**
   * @brief 反射光強度を取得する (0-100)
   * @return 反射光強度
   */
  int getReflectance();

  /**
   * @brief 周囲の明るさを取得する (0-100)
   * @return 周囲の明るさ
   */
  int getAmbient();

  /**
   * @brief 生のRGB値を取得する
   * @param rgb [out] RGB値を格納する構造体
   */
  void getRawRGB(RGB& rgb);

  /**
   * @brief HSV値を取得する (近似なし)
   * @param hsv [out] HSV値を格納する構造体
   * @param surface [in] trueならば表面の色から、falseならば他の光源の色を検出する
   */
  void getRawHSV(HSV& hsv, bool surface = true);

  /**
   * @brief カラーセンサで色を測定する
   * @param hsv [out] HSV値を格納する構造体
   * @param surface [in] trueならば表面の色から、falseならば他の光源の色を検出する
   */
  void getColor(HSV& hsv, bool surface = true);

  /**
   * @brief ライトを点灯する (標準の白)
   */
  void lightOn();

  /**
   * @brief ライトを消灯する(動作しない)
   */
  void lightOff();

  /**
   * @brief ライトの色を個別に設定する
   * @param r 赤 (0-100)
   * @param g 緑 (0-100)
   * @param b 青 (0-100)
   */
  void setLightColor(int r, int g, int b);

  /*
   * @brief カラーセンサが検知する色を設定する
   * @note 現在は公式apiの未実装だが実装予定のためコメントアウト
   * @param size カラーの配列のサイズ
   * @param colors カラーの配列
   * @return -
   */
  // void setDetectableColors(int32_t size, pup_color_hsv_t *colors);

  /**
   * @brief インスタンス生成が成功:0/失敗:1を返す
   * @retval 0 成功
   * @retval 1 失敗
   */
  bool hasError();

 private:
  spikeapi::ColorSensor colorSensor;  // カラーセンサインスタンス
};

#endif
