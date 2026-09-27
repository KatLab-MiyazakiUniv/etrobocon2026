/**
 * @file   RouteFollowerTest.cpp
 * @brief  RouteFollowerクラスをテストする
 * @author okuyama0528
 */

#include <gtest/gtest.h>
#include "RouteFollower.h"
#include "MockNetworkSystem.h"

namespace etrobocon2026_test {

  // QRの位置が許容誤差以内の場合回頭をスキップするテスト
  TEST(RouteFollowerTest, RotationUnderTolerancen)
  {
    MockNetworkSystem mockNetworkSystem;
    SocketClient mockSocketClient(mockNetworkSystem);
    Robot robot(mockSocketClient);

    EtRallyMap map;
    Pid::PidGain gain{ 1.0, 0.0, 0.0 };

    RouteFollower routeFollower(robot, map, 300, gain, gain, gain, 0.0, 1.0);

    std::vector<RouteState> route = {
      { 2, 8, Direction::LEFT }, { 2, 8, Direction::LEFT }  // 同じ向きなので回頭不要
    };

    EXPECT_NO_THROW(routeFollower.run(route));
  }
}  // namespace etrobocon2026_test