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
    // 子動作の継続・終了をテスト中に切り替えるための継続条件
    class SwitchCondition : public BaseContinuationCondition {
     public:
      explicit SwitchCondition(Robot& robot) : BaseContinuationCondition(robot) {}
      bool keepRunning = true;
      bool shouldContinue() override { return keepRunning; }
    };
  }  // namespace

  // 動作の切り替えで共有距離がリセットされず、800mmに到達すると終了するかのテスト
  TEST(ETZumoExitTest, SharedDistanceSurvivesPhasePreparationAndStopsAt800)
  {
    MockNetworkSystem network;
    SocketClient client(network);
    Robot robot(client);
    auto mileage = std::make_shared<ProjectedMileage>();
    auto& wheels = robot.getWheelMotorControllerInstance();
    double current = Mileage::calculateMileage(wheels.getRightCount(), wheels.getLeftCount());
    // 車輪の累計走行距離を変更せず、基準方向に799mm進んだ状態を作る。
    mileage->reset(current - 799.0, 0.0);
    mileage->update(current, 0.0);

    auto inner = std::make_unique<SwitchCondition>(robot);
    auto* tracking = inner.get();
    ETZumoExitCondition first(robot, mileage, 800.0, std::move(inner));
    first.prepare();
    EXPECT_TRUE(first.shouldContinue());
    // 目標距離に未到達でも、子動作の継続条件が成立しなければ終了する。
    tracking->keepRunning = false;
    EXPECT_FALSE(first.shouldContinue());

    // 次の直進動作を準備しても799mmを保持し、800mmに到達した時点で終了する。
    ETZumoExitCondition straight(robot, mileage, 800.0);
    straight.prepare();
    EXPECT_DOUBLE_EQ(mileage->getDistance(), 799.0);
    EXPECT_TRUE(straight.shouldContinue());
    mileage->reset(current - 800.0, 0.0);
    mileage->update(current, 0.0);
    EXPECT_FALSE(straight.shouldContinue());
  }

  // 実際のCSVからET相撲〜フィニッシュの複合動作を生成できるかのテスト
  TEST(ETZumoExitTest, ProductionCsvCreatesExitAndKeepsFinishMotions)
  {
    const auto original = std::filesystem::current_path();
    const auto root = std::filesystem::path(__FILE__).parent_path().parent_path();
    // テスト終了時に、実行前の作業ディレクトリへ戻す。
    struct RestoreDirectory {
      std::filesystem::path path;
      ~RestoreDirectory() { std::filesystem::current_path(path); }
    } restore{ original };
    std::filesystem::current_path(root.parent_path());
    MockNetworkSystem network;
    SocketClient client(network);
    Robot robot(client);
    std::string path = (root / "datafiles/commands/Area/ETZumoLeft.csv").string();
    auto motions = MotionParser::createMotionList(robot, path);
    EXPECT_EQ(motions.size(), 1u);
    if(!motions.empty()) EXPECT_NE(dynamic_cast<ETZumoFinish*>(motions.front()), nullptr);
    for(auto* motion : motions) delete motion;
  }

  // 距離計測が未設定・未初期化、または目標距離が不正なときに継続しないかのテスト
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

  // 子動作の設定がない場合、動作中の左右モータを停止するかのテスト
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
