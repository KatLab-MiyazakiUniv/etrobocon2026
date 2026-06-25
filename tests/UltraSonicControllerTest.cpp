/**
 * @file   UltraSonicControllerTest.cpp
 * @brief  UltraSonicControllerクラスのテスト
 * @author nishijima515
 */

#include <gtest/gtest.h>
#include "UltraSonicController.h"

namespace UltraSonicController_test {

  // 距離を取得できるかのテスト
  TEST(UltraSonicControllerTest, getDistance)
  {
    UltraSonicController ultraSonicController;
    int expectedDistance = 100;  // テスト用の固定値 (mm)

    int distance = ultraSonicController.getDistance();

    EXPECT_EQ(expectedDistance, distance);
  }

}  // namespace UltraSonicController_test
