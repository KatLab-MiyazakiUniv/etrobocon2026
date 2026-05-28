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
}  // namespace spikeapi

namespace etrobocon2026_test {

  // 文字列を渡した時に列挙型COLORに変換出来るかを確認
  TEST(ColorSensorControllerTest, ConvertStringToColor)
  {
    ColorSensorController::COLOR expectedColor = ColorSensorController::COLOR::BLACK;
    EXPECT_EQ(expectedColor, ColorSensorController::convertStringToColor("BLACK"));

    expectedColor = ColorSensorController::COLOR::WHITE;
    EXPECT_EQ(expectedColor, ColorSensorController::convertStringToColor("WHITE"));

    expectedColor = ColorSensorController::COLOR::BLUE;
    EXPECT_EQ(expectedColor, ColorSensorController::convertStringToColor("BLUE"));

    expectedColor = ColorSensorController::COLOR::GREEN;
    EXPECT_EQ(expectedColor, ColorSensorController::convertStringToColor("GREEN"));

    expectedColor = ColorSensorController::COLOR::YELLOW;
    EXPECT_EQ(expectedColor, ColorSensorController::convertStringToColor("YELLOW"));

    expectedColor = ColorSensorController::COLOR::NONE;
    EXPECT_EQ(expectedColor, ColorSensorController::convertStringToColor("INVALIDCOLOR"));
  }

  // 引数の型として,string, string_view, const char*,const char* [6]を受け付けることを確認
  TEST(ColorSensorControllerTest, ConverToStringTypecheck)
  {
    ColorSensorController::COLOR expectedColor = ColorSensorController::COLOR::BLACK;
    std::string stringTypeBlack = "BLACK";
    std::string_view stringViewTypeBlack = "BLACK";
    const char* charPointaTypeBlack = "BLACK";

    EXPECT_EQ(expectedColor, ColorSensorController::convertStringToColor(stringTypeBlack));
    EXPECT_EQ(expectedColor, ColorSensorController::convertStringToColor(stringViewTypeBlack));
    EXPECT_EQ(expectedColor, ColorSensorController::convertStringToColor(charPointaTypeBlack));
    EXPECT_EQ(expectedColor, ColorSensorController::convertStringToColor("BLACK"));
  }

  // ColorSensorControllerのCOLORのメンバ変数を文字列に変換できるかを確認
  TEST(ColorSensorControllerTest, ConvertColorToStringcheck)
  {
    // 明度が低い
    ColorSensorController::HSV lowValue = { 0, 100, 9 };
    EXPECT_EQ(ColorSensorController::COLOR::BLACK,
              ColorSensorController::convertHsvToColor(lowValue));

    // 明度が高い
    ColorSensorController::HSV highValue = { 0, 100, 251 };
    EXPECT_EQ(ColorSensorController::COLOR::WHITE,
              ColorSensorController::convertHsvToColor(highValue));

    // 彩度が低く,明度も低い
    ColorSensorController::HSV lowSaturationLowValue = { 0, 46, 94 };
    EXPECT_EQ(ColorSensorController::COLOR::BLACK,
              ColorSensorController::convertHsvToColor(lowSaturationLowValue));

    // 彩度が低く,明度は高い
    ColorSensorController::HSV lowSaturationHighValue = { 0, 46, 95 };
    EXPECT_EQ(ColorSensorController::COLOR::WHITE,
              ColorSensorController::convertHsvToColor(lowSaturationHighValue));

    // 赤の場合
    ColorSensorController::HSV redValue = { 24, 100, 120 };
    EXPECT_EQ(ColorSensorController::COLOR::RED,
              ColorSensorController::convertHsvToColor(redValue));

    // 黄の場合
    ColorSensorController::HSV yellowValue = { 49, 100, 120 };
    EXPECT_EQ(ColorSensorController::COLOR::YELLOW,
              ColorSensorController::convertHsvToColor(yellowValue));

    // 緑の場合
    ColorSensorController::HSV greenValue = { 169, 100, 120 };
    EXPECT_EQ(ColorSensorController::COLOR::GREEN,
              ColorSensorController::convertHsvToColor(greenValue));

    // 青の場合
    ColorSensorController::HSV blueValue = { 299, 100, 120 };
    EXPECT_EQ(ColorSensorController::COLOR::BLUE,
              ColorSensorController::convertHsvToColor(blueValue));

    // 最後の条件分岐
    ColorSensorController::HSV defaultRedValue = { 300, 100, 120 };
    EXPECT_EQ(ColorSensorController::COLOR::RED,
              ColorSensorController::convertHsvToColor(defaultRedValue));
  }

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
    ColorSensorController::HSV hsv = { 1, 0, 5 };
    spikeapi::ColorSensor::HSV dummyHsv = { 1, 0, 5 };
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
