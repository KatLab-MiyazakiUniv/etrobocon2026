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
 * @file   EtRobocon2026.cpp
 * @brief  全体を制御するクラス（動作確認）
 */

#include "EtRobocon2026.h"
#include "Robot.h"
#include "RelativeRotation.h"
#include "BaseContinuationCondition.h"

// ===== ダミー継続条件 =====
class DummyContinuationCondition : public BaseContinuationCondition {
 public:
  DummyContinuationCondition(Robot& robot) : BaseContinuationCondition(robot) {}

  bool shouldContinue() override { return true; }
};

void EtRobocon2026::start()
{
  Robot robot;

  auto cond = std::make_unique<DummyContinuationCondition>(robot);

  // ★ここが唯一の正解：作るだけ
  RelativeRotation rotation(robot, std::move(cond), 80.0);

  // ★何もしない（フレームワークが勝手に動かす）
}