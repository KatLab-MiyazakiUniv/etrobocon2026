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

    int distance = ultraSonicController.getDistance();

    EXPECT_EQ(10, distance);
  }

}  // namespace UltraSonicController_test
