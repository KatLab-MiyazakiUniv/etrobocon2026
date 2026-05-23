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
    int expectedReflectance = 50;
    int settedReflectance = 50;
    // ダミーの値をセット
    spikeapi::ColorSensor::setReflection(settedReflectance);
    EXPECT_EQ(expectedReflectance, colorController.getReflectance());
  }

  // 環境光強度を取得できるかのテスト
  TEST(ColorSensorControllerTest, GetAmbient)
  {
    ColorSensorController colorController;
    int expectedAmbient = 75;
    int settedAmbient = 75;
    // ダミーの値をセット
    spikeapi::ColorSensor::setAmbient(settedAmbient);
    EXPECT_EQ(expectedAmbient, colorController.getAmbient());
  }

  // RGB値を取得できるかのテスト
  TEST(ColorSensorControllerTest, GetRawRGB)
  {
    ColorSensorController colorController;
    ColorSensorController::RGB rgb;
    // ダミーの値をセット
    spikeapi::ColorSensor::RGB dummyRgb = { 100, 150, 200 };
    int expectedR = 100;
    int expectedG = 150;
    int expectedB = 200;

    spikeapi::ColorSensor::setRGB(dummyRgb);

    colorController.getRawRGB(rgb);
    EXPECT_EQ(expectedR, rgb.r);
    EXPECT_EQ(expectedG, rgb.g);
    EXPECT_EQ(expectedB, rgb.b);
  }

  // HSV値を取得できるかのテスト (surface = true)
  TEST(ColorSensorControllerTest, GetRawHSV_SurfaceTrue)
  {
    ColorSensorController colorController;
    ColorSensorController::HSV hsv;
    // ダミーの値をセット
    spikeapi::ColorSensor::HSV dummyHsv = { 250, 80, 90 };
    int expectedH = 250;
    int expectedS = 80;
    int expectedV = 90;
    spikeapi::ColorSensor::setHSV(dummyHsv);

    colorController.getRawHSV(hsv, true);
    EXPECT_EQ(expectedH, hsv.h);
    EXPECT_EQ(expectedS, hsv.s);
    EXPECT_EQ(expectedV, hsv.v);
  }

  // HSV値を取得できるかのテスト (surface = false)
  TEST(ColorSensorControllerTest, GetRawHSV_SurfaceFalse)
  {
    ColorSensorController colorController;
    ColorSensorController::HSV hsv;
    // ダミーの値をセット
    spikeapi::ColorSensor::HSV dummyHsv = { 120, 50, 60 };
    int expectedH = 120;
    int expectedS = 50;
    int expectedV = 60;
    spikeapi::ColorSensor::setHSV(dummyHsv);

    colorController.getRawHSV(hsv, false);
    EXPECT_EQ(expectedH, hsv.h);
    EXPECT_EQ(expectedS, hsv.s);
    EXPECT_EQ(expectedV, hsv.v);
  }

  // カラーセンサで色を測定するテスト (黒)
  TEST(ColorSensorControllerTest, GetColor_Black)
  {
    ColorSensorController colorController;
    ColorSensorController::HSV hsv;
    // 黒と判定されるHSV値をダミーでセット
    spikeapi::ColorSensor::HSV dummyHsv = { 0, 0, 5 };
    spikeapi::ColorSensor::setHSV(dummyHsv);

    const char* resultColor = colorController.getColor(hsv);
    EXPECT_STREQ("BLACK", resultColor);
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
