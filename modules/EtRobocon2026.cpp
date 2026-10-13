/**
 * @file   EtRobocon2026.cpp
 * @brief  全体を制御するクラス
 * @author HaruArima08
 */

#include "EtRobocon2026.h"

#include <chrono>
#include <thread>

#include "AreaMaster.h"
#include "BatteryController.h"
#include "Course.h"
#include "CsvLogger.h"
#include "IMUCalibrator.h"
#include "RealNetworkSystem.h"
#include "Robot.h"

namespace {

  /**
   * @brief IMUキャリブレーションのサンプル数
   */
  constexpr unsigned int IMU_CALIBRATION_SAMPLES = 500;

  /**
   * @brief IMUキャリブレーションの測定間隔[ms]
   */
  constexpr unsigned int IMU_CALIBRATION_INTERVAL_MS = 10;

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

  // IMUキャリブレーション
  Logger::info("IMUキャリブレーション開始");

  IMUCalibrator calibrator(robot.getIMUControllerInstance());
  calibrator.start(IMU_CALIBRATION_SAMPLES);

  while(!calibrator.isCalibrated()) {
    calibrator.update();

    std::this_thread::sleep_for(std::chrono::milliseconds(IMU_CALIBRATION_INTERVAL_MS));
  }

  Logger::info("IMUキャリブレーション完了");

  // 測定結果を取得
  auto bias = calibrator.getBias();
  auto noise = calibrator.getStandardDeviation();

  Logger::printfLog(Logger::INFO, "IMU Bias X: %.6f Y: %.6f Z: %.6f deg/s", bias.x, bias.y, bias.z);

  Logger::printfLog(Logger::INFO, "IMU Noise X: %.6f Y: %.6f Z: %.6f deg/s", noise.x, noise.y,
                    noise.z);
}