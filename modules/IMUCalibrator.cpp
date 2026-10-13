
/**
 * @file   IMUCalibrator.cpp
 * @brief  IMUの静止時角速度を測定し、バイアスを補正するクラス
 * @author yutaro-1214
 */

#include "IMUCalibrator.h"

#include <cmath>

IMUCalibrator::IMUCalibrator(IMUController& _imuController) : imuController(_imuController) {}

void IMUCalibrator::start(unsigned int sampleCount)
{
  requiredSamples = sampleCount;
  currentSamples = 0;

  calibrated = false;
  calibrating = sampleCount > 0;

  bias = {};
  standardDeviation = {};

  sumX = 0.0;
  sumY = 0.0;
  sumZ = 0.0;

  sumSquareX = 0.0;
  sumSquareY = 0.0;
  sumSquareZ = 0.0;
}

bool IMUCalibrator::update()
{
  if(!calibrating || !imuController.isReady()) {
    return false;
  }

  // ロボットが動いている場合は測定をやり直す
  if(!imuController.isStationary()) {
    unsigned int samples = requiredSamples;
    start(samples);
    return false;
  }

  IMUController::AngularVelocity ang{};
  imuController.getAngularVelocity(ang);

  // 角速度の合計
  sumX += ang.x;
  sumY += ang.y;
  sumZ += ang.z;

  // 角速度の二乗和
  sumSquareX += static_cast<double>(ang.x) * ang.x;
  sumSquareY += static_cast<double>(ang.y) * ang.y;
  sumSquareZ += static_cast<double>(ang.z) * ang.z;

  ++currentSamples;

  if(currentSamples < requiredSamples) {
    return false;
  }

  double count = static_cast<double>(currentSamples);

  // 平均値（バイアス）の計算
  bias.x = static_cast<float>(sumX / count);
  bias.y = static_cast<float>(sumY / count);
  bias.z = static_cast<float>(sumZ / count);

  // 分散の計算
  double varianceX = sumSquareX / count - static_cast<double>(bias.x) * bias.x;

  double varianceY = sumSquareY / count - static_cast<double>(bias.y) * bias.y;

  double varianceZ = sumSquareZ / count - static_cast<double>(bias.z) * bias.z;

  // 丸め誤差による負の値を防ぐ
  if(varianceX < 0.0) varianceX = 0.0;
  if(varianceY < 0.0) varianceY = 0.0;
  if(varianceZ < 0.0) varianceZ = 0.0;

  standardDeviation.x = static_cast<float>(std::sqrt(varianceX));
  standardDeviation.y = static_cast<float>(std::sqrt(varianceY));
  standardDeviation.z = static_cast<float>(std::sqrt(varianceZ));

  calibrated = true;
  calibrating = false;

  return true;
}

void IMUCalibrator::getCorrectedAngularVelocity(IMUController::AngularVelocity& ang)
{
  imuController.getAngularVelocity(ang);

  if(!calibrated) {
    return;
  }

  // 静止中に測定した平均誤差を取り除く
  ang.x -= bias.x;
  ang.y -= bias.y;
  ang.z -= bias.z;
}

IMUController::AngularVelocity IMUCalibrator::getBias() const
{
  return bias;
}

IMUController::AngularVelocity IMUCalibrator::getStandardDeviation() const
{
  return standardDeviation;
}

bool IMUCalibrator::isCalibrated() const
{
  return calibrated;
}

unsigned int IMUCalibrator::getSampleCount() const
{
  return currentSamples;
}
