/**
 * @file ColorSensorController.h
 * @brief カラーセンサを制御するラッパークラス
 * @author sadomiya-sousi
 */

#ifndef COLOR_SENSOR_CONTROLLER_H
#define COLOR_SENSOR_CONTROLLER_H

#include "ColorSensor.h"
#include <cstdint>
#include <iostream>

/**
 *  @brief 色を表す列挙型
 */
enum class COLOR : int {
  NONE = 0,
  BLACK = 1,
  WHITE = 2,
  BLUE = 3,
  GREEN = 4,
  YELLOW = 5,
  RED = 6,
};

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
   * @brief 文字列を列挙型COLORに変換
   * @param str 変換する文字列
   * @return 色
   */
  static COLOR convertStringToColor(std::string_view str);

  /**
   * @brief 列挙型COLORを文字列に変換
   * @param color 色
   * @return 文字列の色
   */
  static const char* convertColorToString(const COLOR& color);

  /**
   * @brief カラーセンサーでhsv値を測定
   * @param hsv値を設定するHSV構造体
   * @return 色（hsvによる表現）
   */
  static COLOR convertHsvToColor(HSV& hsv);

  /**
   * コンストラクタ
   * @param port 接続ポート
   */
  ColorSensorController();

  /**
   * @brief 反射光強度を取得(0-100)
   * @return 反射光強度
   */
  int getReflectance();

  /**
   * @brief 周囲の明るさを取得(0-100)
   * @return 周囲の明るさ
   */
  int getAmbient();

  /**
   * @brief 生のRGB値を取得
   * @param rgb RGB値を格納する構造体
   */
  void getRawRGB(RGB& rgb);

  /**
   * @brief HSV値を取得する (近似なし)
   * @param hsv HSV値を格納する構造体
   * @param surface trueならば表面の色から、falseならば他の光源の色を検出する
   */
  void getRawHSV(HSV& hsv, bool surface = true);

  /**
   * @brief カラーセンサで色を測定(自作)
   * @param hsv HSV値を格納する構造体
   * @return 文字列で表現された色
   */
  const char* getColor(HSV& hsv);

  /**
   * @brief ライトを点灯する (標準の白)
   */
  void lightOn();

  /**
   * @brief ライトを消灯する(動作しない)
   */
  void lightOff();

  /**
   * @brief ライトの輝度を個別に設定する
   * @param left 左側 (0-255)
   * @param center 中央 (0-255)
   * @param right 右側 (0-255)
   */
  void setLightBrightness(int left, int center, int right);

  /*
   * @brief カラーセンサが検知する色を設定する
   * @note 現在は公式apiの未実装だが実装予定のためコメントアウト
   * @param size カラーの配列のサイズ
   * @param colors カラーの配列
   * @return -
   */
  // void setDetectableColors(int32_t size, pup_color_hsv_t *colors);

 private:
  static constexpr int SATURATION_BORDER = 47;    // 無彩色かどうかの彩度の境界
  static constexpr int BLACK_LIMIT_BORDER = 10;   // 黒の明度の境界
  static constexpr int WHITE_LIMIT_BORDER = 250;  // 白の明度の境界
  static constexpr int BLACK_BORDER = 95;         // 無彩色の黒の明度の境界
  static constexpr int RED_BORDER = 25;           // 赤の色相の境界
  static constexpr int YELLOW_BORDER = 50;        // 黄の色相の境界
  static constexpr int GREEN_BORDER = 170;        // 緑の色相の境界
  static constexpr int BLUE_BORDER = 300;         // 青の色相の境界

  spikeapi::ColorSensor colorSensor;  // カラーセンサインスタンス
};

#endif
