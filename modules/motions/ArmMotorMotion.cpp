/**
 * @file   ArmMotorMotion.cpp
 * @brief  アームモータを指定角度まで回転させるクラス
 * @author HaruArima08
 */

#include "ArmMotorMotion.h"

#include <functional>
#include <memory>
#include <utility>

#include "ClockUtil.h"
#include "Logger.h"

namespace {

  /**
   * @brief 回転量の記録間隔[ms]
   */
  constexpr int ARM_LOG_INTERVAL_MS = 50;

  /**
   * @brief 角位置が変化しない場合の停止判定時間[ms]
   */
  constexpr int ARM_STALL_TIMEOUT_MS = 350;

  /**
   * @brief 逆回転と判定する回転量[deg]
   */
  constexpr int32_t ARM_REVERSE_LIMIT = 10;

  /**
   * @brief アーム動作の完了と外部の継続条件を組み合わせるクラス
   */
  class ArmMotorCompletionCondition : public BaseContinuationCondition {
   public:
    /**
     * @brief コンストラクタ
     * @param _robot Robotクラスのインスタンス
     * @param _condition 外部から指定された継続条件
     * @param _notCompleted アーム動作が完了していないかを返す関数
     */
    ArmMotorCompletionCondition(Robot& _robot,
                                std::unique_ptr<BaseContinuationCondition> _condition,
                                std::function<bool()> _notCompleted)
      : BaseContinuationCondition(_robot),
        condition(std::move(_condition)),
        notCompleted(std::move(_notCompleted))
    {
    }

    /**
     * @brief 外部の継続条件を初期化する
     */
    void prepare() override
    {
      condition->prepare();
    }

    /**
     * @brief 目標未到達かつ外部の継続条件を満たす場合に継続する
     * @return true 動作を継続する、false 動作を終了する
     */
    bool shouldContinue() override
    {
      return notCompleted() && condition->shouldContinue();
    }

   private:
    std::unique_ptr<BaseContinuationCondition> condition;  // 外部の継続条件
    std::function<bool()> notCompleted;                    // 完了状態の判定関数
  };

}  // namespace

ArmMotorMotion::ArmMotorMotion(Robot& _robot,
                               std::unique_ptr<BaseContinuationCondition> _continuationCondition,
                               int _power, int32_t _targetCount)
  : BaseMotion(_robot,
               std::make_unique<ArmMotorCompletionCondition>(
                   _robot, std::move(_continuationCondition),
                   [this]() { return !completed; })),
    power(_power),
    targetCount(_targetCount)
{
}

void ArmMotorMotion::prepare()
{
  completed = false;
  reachedTarget = false;
  stalled = false;
  wrongDirection = false;

  // モーションを実行する時点でアームモータを生成する。
  armMotor = std::make_unique<ArmMotorController>();
  armMotor->stopMotor();

  startCount = armMotor->getCount();
  lastCount = startCount;
  startTime = ClockUtil::now();
  lastMovementTime = startTime;
  lastLogTime = startTime;

  Logger::printfLog(Logger::INFO,
                    "ArmMotorMotion start: power=%d target=%ld deg count=%ld",
                    power, static_cast<long>(targetCount), static_cast<long>(startCount));

  armMotor->setPower(power);
}

void ArmMotorMotion::executeStep()
{
  const int now = ClockUtil::now();
  const int32_t currentCount = armMotor->getCount();
  const int32_t movedCount = currentCount - startCount;
  const int32_t progress = power > 0 ? movedCount : -movedCount;

  if(now - lastLogTime >= ARM_LOG_INTERVAL_MS) {
    Logger::printfLog(Logger::INFO, "ArmMotorMotion: time=%d ms count=%ld deg power=%d",
                      now - startTime, static_cast<long>(movedCount), armMotor->getPower());
    lastLogTime = now;
  }

  if(progress >= targetCount) {
    reachedTarget = true;
    completed = true;
    return;
  }

  if(progress <= -ARM_REVERSE_LIMIT) {
    wrongDirection = true;
    completed = true;
    return;
  }

  if(currentCount != lastCount) {
    lastCount = currentCount;
    lastMovementTime = now;
  } else if(now - lastMovementTime >= ARM_STALL_TIMEOUT_MS) {
    stalled = true;
    completed = true;
  }
}

void ArmMotorMotion::finish()
{
  if(!armMotor) return;

  // 目標到達・タイムアウト・異常停止のいずれでもモータを停止する。
  armMotor->stopMotor();

  const int elapsedTime = ClockUtil::now() - startTime;
  const int32_t movedCount = armMotor->getCount() - startCount;

  Logger::printfLog(Logger::INFO, "ArmMotorMotion result: count=%ld deg elapsed=%d ms",
                    static_cast<long>(movedCount), elapsedTime);

  if(reachedTarget) {
    Logger::info("ArmMotorMotion: target reached");
  } else if(stalled) {
    Logger::error("ArmMotorMotion: arm motor stalled");
  } else if(wrongDirection) {
    Logger::error("ArmMotorMotion: arm motor moved in the wrong direction");
  } else {
    Logger::error("ArmMotorMotion: timeout before target");
  }
}