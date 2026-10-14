
/**
 * @file   EtRobocon2026.cpp
 * @brief  全体を制御するクラス
 * @author HaruArima08
 */

#include "EtRobocon2026.h"

#include <chrono>
#include <thread>

#include "AreaMaster.h"
#include "Robot.h"
#include "Course.h"
#include "CsvLogger.h"
#include "RealNetworkSystem.h"
#include "BatteryController.h"
#include "IMUCalibrator.h"

namespace {

  /**
   * @brief IMUキャリブレーションのサンプル数
   */
  constexpr unsigned int IMU_CALIBRATION_SAMPLES = 500;

  /**
   * @brief IMUキャリブレーションの測定間隔[ms]
   */
  constexpr unsigned int IMU_CALIBRATION_INTERVAL_MS = 10;

  /**
   * @brief キャリブレーションの最大待機時間[ms]
   */
  constexpr unsigned int IMU_CALIBRATION_TIMEOUT_MS = 30000;

  /**
   * @brief 補正確認の測定時間[ms]
   */
  constexpr unsigned int IMU_TEST_DURATION_MS = 30000;

  /**
   * @brief 補正確認の測定間隔[ms]
   */
  constexpr unsigned int IMU_TEST_INTERVAL_MS = 10;

  /**
   * @brief 補正確認ログの出力間隔[ms]
   */
  constexpr unsigned int IMU_LOG_INTERVAL_MS = 1000;

}  // namespace

void EtRobocon2026::start()
{
  // CsvLogger::init();
  // CsvLogger::writeHeader();

  Logger::info("Hello KATLAB");

  RealNetworkSystem real;
  SocketClient client(real);
  Robot robot(client);

  // int voltage = BatteryController::getVoltage();
  // Logger::printfLog(Logger::INFO, "バッテリー電圧: %d mV", voltage);

  // robot.getCameraSocketClientInstance().connectToServer();

  IMUController& imu = robot.getIMUControllerInstance();

  IMUCalibrator calibrator(imu);

  // IMUキャリブレーション
  Logger::info("IMUキャリブレーション開始");

  calibrator.start(IMU_CALIBRATION_SAMPLES);

  auto calibrationStart = std::chrono::steady_clock::now();

  while(!calibrator.isCalibrated()) {
    calibrator.update();

    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                       std::chrono::steady_clock::now() - calibrationStart)
                       .count();

    if(elapsed >= IMU_CALIBRATION_TIMEOUT_MS) {
      Logger::error("IMUキャリブレーションがタイムアウトしました");
      return;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(IMU_CALIBRATION_INTERVAL_MS));
  }

  Logger::info("IMUキャリブレーション完了");

  // キャリブレーション結果を取得
  auto bias = calibrator.getBias();
  auto noise = calibrator.getStandardDeviation();

  Logger::printfLog(Logger::INFO, "IMU Bias X: %.6f Y: %.6f Z: %.6f deg/s", bias.x, bias.y, bias.z);

  Logger::printfLog(Logger::INFO, "IMU Noise X: %.6f Y: %.6f Z: %.6f deg/s", noise.x, noise.y,
                    noise.z);

  // 補正前後の比較
  Logger::info("IMU補正確認開始");
  Logger::info("ロボットを静止させたまま30秒間測定します");

  double rawAngle = 0.0;
  double correctedAngle = 0.0;

  double rawSum = 0.0;
  double correctedSum = 0.0;

  unsigned int sampleCount = 0;

  auto testStart = std::chrono::steady_clock::now();
  auto previousTime = testStart;
  auto previousLogTime = testStart;

  while(true) {
    auto currentTime = std::chrono::steady_clock::now();

    auto elapsedMs
        = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - testStart).count();

    if(elapsedMs >= IMU_TEST_DURATION_MS) {
      break;
    }

    double dt = std::chrono::duration<double>(currentTime - previousTime).count();

    previousTime = currentTime;

    IMUController::AngularVelocity raw{};
    imu.getAngularVelocity(raw);

    // 同じ測定値にバイアス補正を適用する
    double rawZ = raw.z;
    double correctedZ = rawZ - bias.z;

    // 角速度から累積角度を計算する
    rawAngle += rawZ * dt;
    correctedAngle += correctedZ * dt;

    // 平均角速度を計算するために加算する
    rawSum += rawZ;
    correctedSum += correctedZ;

    ++sampleCount;

    auto logElapsed
        = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - previousLogTime)
              .count();

    if(logElapsed >= IMU_LOG_INTERVAL_MS) {
      Logger::printfLog(Logger::INFO, "IMU Test: %ld ms RAW Z: %.6f Corrected Z: %.6f",
                        static_cast<long>(elapsedMs), rawZ, correctedZ);

      Logger::printfLog(Logger::INFO, "IMU Angle: RAW %.6f Corrected %.6f deg", rawAngle,
                        correctedAngle);

      previousLogTime = currentTime;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(IMU_TEST_INTERVAL_MS));
  }

  // 測定結果の表示
  Logger::info("IMU補正確認終了");

  if(sampleCount > 0) {
    double averageRaw = rawSum / sampleCount;
    double averageCorrected = correctedSum / sampleCount;

    Logger::printfLog(Logger::INFO, "IMU Average RAW Z: %.6f deg/s", averageRaw);

    Logger::printfLog(Logger::INFO, "IMU Average Corrected Z: %.6f deg/s", averageCorrected);

    Logger::printfLog(Logger::INFO, "IMU Total RAW Angle: %.6f deg", rawAngle);

    Logger::printfLog(Logger::INFO, "IMU Total Corrected Angle: %.6f deg", correctedAngle);
  }
}
