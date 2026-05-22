/**
 * @file ColorSensor.h
 * @brief カラーセンサクラス(ダミー)
 * @author sadomiya-sousi
 */

#ifndef SPIKE_CPP_API_COLOR_SENSOR_H_
#define SPIKE_CPP_API_COLOR_SENSOR_H_

#include <stdint.h>
#include "Port.h"

namespace spikeapi {
  /**
   * SPIKE カラーセンサクラス
   */
  class ColorSensor {
   public:
    struct RGB {
      uint16_t r;
      uint16_t g;
      uint16_t b;
    };

    struct HSV {
      uint16_t h;
      uint8_t s;
      uint8_t v;
    };

    /**
     * コンストラクタ
     * @param port PUPポートID
     */
    ColorSensor(EPort port) : mPort(port)
    {
      // 静的メンバは一度だけ初期化されるため、ここでは何もしない
    }

    /**
     * @brief カラーセンサのRGB値を取得する
     * @param rgb 値を設定するRGB構造体
     * @return -
     */
    void getRGB(RGB& rgb) const { rgb = sRGB; }

    /**
     * @brief カラーセンサで色を測定する
     * @param hsv 値を設定するHSV構造体
     * @param surface trueならば表面の色から、falseならば他の光源の色を検出する
     * @return -
     */
    void getColor(HSV& hsv, bool surface = true) const { hsv = sHSV; }

    /**
     * @brief カラーセンサで色を測定する（近似なし）
     * @param hsv 値を設定するHSV構造体
     * @param surface trueならば表面の色から、falseならば他の光源の色を検出する
     * @return -
     */
    void getHSV(HSV& hsv, bool surface = true) const { hsv = sHSV; }

    /**
     * @brief センサーの発する光を表面がどの程度反射するかを測定する
     * @return どの程度反射しているか（％）
     */
    int32_t getReflection() const { return sReflection; }

    /**
     * @brief 周囲の光の強度を測定する
     * @return 周囲の光の強度（％）
     */
    int32_t getAmbient() const { return sAmbient; }

    /**
     * @brief カラーセンサのライトを設定する
     * @param bv1 輝度1
     * @param bv2 輝度2
     * @param bv3 輝度3
     * @return -
     */
    void setLight(int32_t bv1, int32_t bv2, int32_t bv3) const
    {
      // ダミーなので何もしない
    }

    /**
     * @brief カラーセンサのライトを点灯する
     * @param -
     * @return -
     */
    void lightOn() const
    {
      // ダミーなので何もしない
    }

    /**
     * @brief カラーセンサのライトを消灯する
     * @param -
     * @return -
     */
    void lightOff() const
    {
      // ダミーなので何もしない
    }

    /**
     * @brief インスタンス生成が正常にできたかどうかを確認するための共通メソッド
     */
    bool hasError() { return sHasError; }

    // テスト用メンバ（値を外部から操作可能にする）
    static RGB sRGB;
    static HSV sHSV;
    static int32_t sReflection;
    static int32_t sAmbient;
    static bool sHasError;  // hasError()の戻り値を制御するため

    static void setRGB(RGB rgb) { sRGB = rgb; }
    static void setHSV(HSV hsv) { sHSV = hsv; }
    static void setReflection(int32_t reflection) { sReflection = reflection; }
    static void setAmbient(int32_t ambient) { sAmbient = ambient; }
    // static void setHasError(bool hasError) { sHasError = hasError; }

   private:
    EPort mPort;
  };  // class ColorSensor
}  // namespace spikeapi

#endif  // !SPIKE_CPP_API_COLOR_SENSOR_H_
