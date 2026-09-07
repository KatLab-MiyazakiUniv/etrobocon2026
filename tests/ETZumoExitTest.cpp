/**
 * @file ETZumoExitTest.cpp
 * @brief ETZumoExitクラスをテストする
 * @author miyahara046
 */

#include <gtest/gtest.h>
#include <filesystem>
#include "ETZumoExit.h"
#include "ETZumoExitCondition.h"
#include "Mileage.h"
#include "MotionParser.h"
#include "MockNetworkSystem.h"

namespace etrobocon2026_test {
  namespace {
    class SwitchCondition : public BaseContinuationCondition {
     public:
      explicit SwitchCondition(Robot& robot) : BaseContinuationCondition(robot) {}
      bool keepRunning = true;
      bool shouldContinue() override { return keepRunning; }
    };
  }  // namespace

  TEST(ETZumoExitTest, SharedDistanceSurvivesPhasePreparationAndStopsAt800)
  {
    MockNetworkSystem network;
    SocketClient client(network);
    Robot robot(client);
    auto mileage = std::make_shared<ProjectedMileage>();
    auto& wheels = robot.getWheelMotorControllerInstance();
    double current = Mileage::calculateMileage(wheels.getRightCount(), wheels.getLeftCount());
    mileage->reset(current - 799.0, 0.0);
    mileage->update(current, 0.0);

    auto inner = std::make_unique<SwitchCondition>(robot);
    auto* tracking = inner.get();
    ETZumoExitCondition first(robot, mileage, 800.0, std::move(inner));
    first.prepare();
    EXPECT_TRUE(first.shouldContinue());
    tracking->keepRunning = false;
    EXPECT_FALSE(first.shouldContinue());

    ETZumoExitCondition straight(robot, mileage, 800.0);
    straight.prepare();
    EXPECT_DOUBLE_EQ(mileage->getDistance(), 799.0);
    EXPECT_TRUE(straight.shouldContinue());
    mileage->reset(current - 800.0, 0.0);
    mileage->update(current, 0.0);
    EXPECT_FALSE(straight.shouldContinue());
  }

  TEST(ETZumoExitTest, ProductionCsvCreatesExitAndKeepsFinishMotions)
  {
    // MotionParserはプロジェクトの親ディレクトリから実行する仕様。
    const auto original = std::filesystem::current_path();
    const auto root = std::filesystem::path(__FILE__).parent_path().parent_path();
    struct RestoreDirectory {
      std::filesystem::path path;
      ~RestoreDirectory() { std::filesystem::current_path(path); }
    } restore{ original };
    std::filesystem::current_path(root.parent_path());
    MockNetworkSystem network;
    SocketClient client(network);
    Robot robot(client);
    std::string path = (root / "datafiles/commands/Area/LineTraceLeft.csv").string();
    auto motions = MotionParser::createMotionList(robot, path);
    EXPECT_EQ(motions.size(), 7u);
    if(!motions.empty()) EXPECT_NE(dynamic_cast<ETZumoExit*>(motions.front()), nullptr);
    for(auto* motion : motions) delete motion;
  }

}  // namespace etrobocon2026_test

namespace etrobocon2026_test {
  TEST(ETZumoExitTest, InvalidConditionsStopWithoutDereferencingMissingMileage)
  {
    MockNetworkSystem network;
    SocketClient client(network);
    Robot robot(client);
    ETZumoExitCondition missing(robot, nullptr, 800.0);
    EXPECT_FALSE(missing.shouldContinue());
    auto mileage = std::make_shared<ProjectedMileage>();
    ETZumoExitCondition uninitialized(robot, mileage, 800.0);
    EXPECT_FALSE(uninitialized.shouldContinue());
    mileage->reset(0.0, 0.0);
    ETZumoExitCondition invalidTarget(robot, mileage, -1.0);
    EXPECT_FALSE(invalidTarget.shouldContinue());
  }

  TEST(ETZumoExitTest, MissingMotionConfigurationStopsMotors)
  {
    MockNetworkSystem network;
    SocketClient client(network);
    Robot robot(client);
    auto& wheels = robot.getWheelMotorControllerInstance();
    wheels.setRightPower(30);
    wheels.setLeftPower(30);
    ETZumoExit exit(robot, std::make_unique<RepeatCountCondition>(robot, 1),
                    std::make_shared<ProjectedMileage>(), {});
    exit.run();
    EXPECT_EQ(wheels.getRightPower(), 0);
    EXPECT_EQ(wheels.getLeftPower(), 0);
  }
}  // namespace etrobocon2026_test
