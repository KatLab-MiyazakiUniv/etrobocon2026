/**
 * @file ColorSensorControllerTest.cpp
 * @brief カラーセンサを制御するラッパークラスのテスト
 * @author sadomiya-sousi
 */

#include "gtest/gtest.h"
#include "ColorSensorController.h"
#include "ColorSensor.h"  // For dummy spikeapi::ColorSensor

namespace spikeapi {

  // 静的メンバの定義
  ColorSensor::RGB ColorSensor::sRGB = { 0, 0, 0 };
  ColorSensor::HSV ColorSensor::sHSV = { 0, 0, 0 };
  int32_t ColorSensor::sReflection = 0;
  int32_t ColorSensor::sAmbient = 0;
  // bool ColorSensor::sHasError = false;

}  // namespace spikeapi

namespace etrobocon2026_test {

  // 反射光強度を取得できるかのテスト
  TEST(ColorSensorControllerTest, GetReflectance)
  {
    ColorSensorController colorController;
    // ダミーの値をセット
    spikeapi::ColorSensor::setReflection(50);
    EXPECT_EQ(50, colorController.getReflectance());
  }

  // 環境光強度を取得できるかのテスト
  TEST(ColorSensorControllerTest, GetAmbient)
  {
    ColorSensorController colorController;
    // ダミーの値をセット
    spikeapi::ColorSensor::setAmbient(75);
    EXPECT_EQ(75, colorController.getAmbient());
  }

  // RGB値を取得できるかのテスト
  TEST(ColorSensorControllerTest, GetRawRGB)
  {
    ColorSensorController colorController;
    ColorSensorController::RGB rgb;
    // ダミーの値をセット
    spikeapi::ColorSensor::RGB dummyRgb = { 100, 150, 200 };
    spikeapi::ColorSensor::setRGB(dummyRgb);

    colorController.getRawRGB(rgb);
    EXPECT_EQ(100, rgb.r);
    EXPECT_EQ(150, rgb.g);
    EXPECT_EQ(200, rgb.b);
  }

  // HSV値を取得できるかのテスト (surface = true)
  TEST(ColorSensorControllerTest, GetRawHSV_SurfaceTrue)
  {
    ColorSensorController colorController;
    ColorSensorController::HSV hsv;
    // ダミーの値をセット
    spikeapi::ColorSensor::HSV dummyHsv = { 250, 80, 90 };
    spikeapi::ColorSensor::setHSV(dummyHsv);

    colorController.getRawHSV(hsv, true);
    EXPECT_EQ(250, hsv.h);
    EXPECT_EQ(80, hsv.s);
    EXPECT_EQ(90, hsv.v);
  }

  // HSV値を取得できるかのテスト (surface = false)
  TEST(ColorSensorControllerTest, GetRawHSV_SurfaceFalse)
  {
    ColorSensorController colorController;
    ColorSensorController::HSV hsv;
    // ダミーの値をセット
    spikeapi::ColorSensor::HSV dummyHsv = { 120, 50, 60 };
    spikeapi::ColorSensor::setHSV(dummyHsv);

    colorController.getRawHSV(hsv, false);
    EXPECT_EQ(120, hsv.h);
    EXPECT_EQ(50, hsv.s);
    EXPECT_EQ(60, hsv.v);
  }

  // 色を測定できるかのテスト (getColor, surface = true)
  TEST(ColorSensorControllerTest, GetColor_SurfaceTrue)
  {
    ColorSensorController colorController;
    ColorSensorController::HSV hsv;
    // ダミーの値をセット
    spikeapi::ColorSensor::HSV dummyHsv = { 300, 70, 85 };
    spikeapi::ColorSensor::setHSV(dummyHsv);

    colorController.getColor(hsv, true);
    EXPECT_EQ(300, hsv.h);
    EXPECT_EQ(70, hsv.s);
    EXPECT_EQ(85, hsv.v);
  }

  // 色を測定できるかのテスト (getColor, surface = false)
  TEST(ColorSensorControllerTest, GetColor_SurfaceFalse)
  {
    ColorSensorController colorController;
    ColorSensorController::HSV hsv;
    // ダミーの値をセット
    spikeapi::ColorSensor::HSV dummyHsv = { 60, 40, 70 };
    spikeapi::ColorSensor::setHSV(dummyHsv);

    colorController.getColor(hsv, false);
    EXPECT_EQ(60, hsv.h);
    EXPECT_EQ(40, hsv.s);
    EXPECT_EQ(70, hsv.v);
  }

  // ライトを点灯/消灯するテスト
  TEST(ColorSensorControllerTest, LightControl)
  {
    ColorSensorController colorController;
    // コンパイルが通ることを確認するのみ。
    colorController.lightOn();
    colorController.lightOff();
    colorController.setLightBrightness(10, 20, 30);
    SUCCEED();
  }

}  // namespace etrobocon2026_test
