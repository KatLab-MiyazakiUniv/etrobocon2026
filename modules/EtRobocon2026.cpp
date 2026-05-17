/**
 * @file   EtRobocon2026.cpp
 * @brief  全体を制御するクラス
 * @author HaruArima08
 */
/*
#include "EtRobocon2026.h"

void EtRobocon2026::start()
{
  std::cout << "Hello KATLAB" << std::endl;
}*/
/**
 * @file   EtRobocon2026Test.cpp
 * @brief  EtRobocon2026クラスのテスト
 */

#include "EtRobocon2026.h"

namespace etrobocon2026_test {

  TEST(EtRobocon2026Test, Start)
  {
    std::string expected = "Hello KATLAB\n";

    testing::internal::CaptureStdout();

    EtRobocon2026::start();

    std::string output = testing::internal::GetCapturedStdout();

    EXPECT_EQ(expected, output);
  }

}  // namespace etrobocon2026_test