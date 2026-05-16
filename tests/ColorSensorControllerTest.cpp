/**
 * @file ColorSensorControllerTest.cpp
 * @brief ColorSensorControllerクラスをテストする
 * @author sadomiya-sousi
 */

#include "gtest/gtest.h"
#include "ColorSensorController.h"

using namespace spikeapi;

namespace etrobocon2026_test {

  // 反射光強度を取得できるかのテスト
  TEST(ColorSensorControllerTest, GetReflection)
  {
    ColorSensorController colorController;
    // ダミーの値をセット（実際にはテスト用にダミーを操作する手段が必要だが、
    // ここでは作成したばかりなのでデフォルト値や動作を確認する）
    EXPECT_EQ(0, colorController.getReflection());
  }

  // 環境光強度を取得できるかのテスト
  TEST(ColorSensorControllerTest, GetAmbient)
  {
    ColorSensorController colorController;
    EXPECT_EQ(0, colorController.getAmbient());
  }

  // RGB値を取得できるかのテスト
  TEST(ColorSensorControllerTest, GetRawRGB)
  {
    ColorSensorController colorController;
    ColorSensor::RGB rgb;
    colorController.getRawRGB(rgb);
    EXPECT_EQ(0, rgb.r);
    EXPECT_EQ(0, rgb.g);
    EXPECT_EQ(0, rgb.b);
  }

  // HSV値を取得できるかのテスト
  TEST(ColorSensorControllerTest, GetRawHSV)
  {
    ColorSensorController colorController;
    ColorSensor::HSV hsv;
    colorController.getRawHSV(hsv);
    EXPECT_EQ(0, hsv.h);
    EXPECT_EQ(0, hsv.s);
    EXPECT_EQ(0, hsv.v);
  }

}  // namespace etrobocon2026_test
