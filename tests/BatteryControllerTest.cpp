/**
 * @file BatteryControllerTest.cpp
 * @brief BatteryControllerクラスをテストする
 * @author migaku2645
 */

#include <gtest/gtest.h>
#include "BatteryController.h"

namespace etrobocon2026 {
  // バッテリーの電圧が取得できるかテスト
  TEST(BatteryControllerTest, VolageTest)
  {
    uint16_t voltage = BatteryController::getVoltage();
    uint16_t expected = 100;
    EXPECT_EQ(expected, voltage);
  }

  // バッテリーの電流が取得できるかテスト
  TEST(BatteryControllerTest, CurrentTest)
  {
    uint16_t current = BatteryController::getCurrent();
    uint16_t expected = 200;
    EXPECT_EQ(expected, current);
  }

}  // namespace etrobocon2026