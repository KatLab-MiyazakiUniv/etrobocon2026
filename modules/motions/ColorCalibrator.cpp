
/**
 * @file   ColorCalibrator.cpp
 * @brief  カラーセンサーのHSV基準値を取得・保存するクラス
 */

#include "ColorCalibrator.h"
#include "Logger.h"
#include <cmath>
#include <memory>

namespace {
  constexpr double PI = 3.14159265358979323846;
  constexpr double DEGREE_TO_RADIAN = PI / 180.0;
  constexpr double RADIAN_TO_DEGREE = 180.0 / PI;
}  // namespace

ColorCalibrator::ColorCalibrator(Robot& _robot, ColorSensorController::COLOR _targetColor,
                                 int _sampleCount)
  : BaseMotion(_robot, std::make_unique<RepeatCountCondition>(_robot, _sampleCount)),
    targetColor(_targetColor),
    sampleCount(_sampleCount)
{
  LOG_CREATE("ColorCalibrator");
}

bool ColorCalibrator::canStart()
{
  if(targetColor == ColorSensorController::COLOR::NONE) {
    Logger::error("キャリブレーション対象の色が無効です。");
    return false;
  }

  if(sampleCount <= 0) {
    Logger::error("HSV値の取得回数が無効です。");
    return false;
  }

  return true;
}

void ColorCalibrator::prepare()
{
  hsvSamples.clear();
  hsvSamples.reserve(sampleCount);
}

void ColorCalibrator::executeStep()
{
  ColorSensorController::HSV hsv;
  robot.getColorSensorControllerInstance().getRawHSV(hsv, true);

  hsvSamples.push_back(hsv);
}

ColorSensorController::HSV ColorCalibrator::calculateAverageHSV() const
{
  double sumSin = 0.0;
  double sumCos = 0.0;
  double sumS = 0.0;
  double sumV = 0.0;

  for(const auto& hsv : hsvSamples) {
    const double radian = hsv.h * DEGREE_TO_RADIAN;

    sumSin += std::sin(radian);
    sumCos += std::cos(radian);
    sumS += hsv.s;
    sumV += hsv.v;
  }

  ColorSensorController::HSV averageHSV{};

  // Hは円周平均を計算する
  double averageH = std::atan2(sumSin, sumCos) * RADIAN_TO_DEGREE;

  if(averageH < 0.0) {
    averageH += 360.0;
  }

  averageHSV.h = static_cast<uint16_t>(std::lround(averageH));

  // 360度は0度として扱う
  if(averageHSV.h >= 360) {
    averageHSV.h = 0;
  }

  // SとVは算術平均を計算する
  averageHSV.s = static_cast<uint8_t>(std::lround(sumS / hsvSamples.size()));

  averageHSV.v = static_cast<uint8_t>(std::lround(sumV / hsvSamples.size()));

  return averageHSV;
}

void ColorCalibrator::finish()
{
  if(hsvSamples.size() != static_cast<size_t>(sampleCount)) {
    Logger::error("HSV値の取得回数が一致しません。");
    return;
  }

  const ColorSensorController::HSV averageHSV = calculateAverageHSV();

  ColorData data{ targetColor, averageHSV, true };

  // 色データを更新
  if(!robot.getColorDataManagerInstance().updateColorData(data)) {
    Logger::error("色データの更新に失敗しました。");
    return;
  }

  // CSVに保存
  if(!robot.getColorDataManagerInstance().saveToCsv()) {
    Logger::error("色データのCSV保存に失敗しました。");
    return;
  }

  Logger::info("Color calibration completed");
}
