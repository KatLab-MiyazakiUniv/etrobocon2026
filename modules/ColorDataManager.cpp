
/**
 * @file ColorDataManager.cpp
 * @brief HSV基準値の管理とCSVファイルの読み書きを行うクラス
 */

#include "ColorDataManager.h"

#include <filesystem>
#include <fstream>
#include <sstream>
#include <limits>
#include <system_error>

#include "Logger.h"

// コンストラクタ
ColorDataManager::ColorDataManager()
  : fileName(std::string(DEFAULT_COLOR_DATA_PATH) + DEFAULT_COLOR_DATA_NAME)
{
  initializeColorData();
}

// 全色のデータを初期化する
void ColorDataManager::initializeColorData()
{
  using COLOR = ColorSensorController::COLOR;

  colorDataList.clear();

  const COLOR colors[] = {
    COLOR::BLACK, COLOR::WHITE, COLOR::BLUE, COLOR::GREEN, COLOR::YELLOW, COLOR::RED, COLOR::GRAY,
  };

  for(const COLOR color : colors) {
    ColorData data;
    data.color = color;
    data.hsv = { 0, 0, 0 };
    data.acquired = false;

    colorDataList.push_back(data);
  }
}

// CSVファイルから色データを読み込む
bool ColorDataManager::loadFromCsv()
{
  // 読み込み前に初期状態へ戻す
  initializeColorData();

  std::ifstream file(fileName);

  if(!file.is_open()) {
    std::error_code errorCode;
    const bool fileExists = std::filesystem::exists(fileName, errorCode);

    // CSVファイルが存在しない場合は初期データを作成する
    if(!fileExists && !errorCode) {
      Logger::info("ColorDataManager: csv file not found. Creating default file.");
      return saveToCsv();
    }

    Logger::error("ColorDataManager: failed to open csv file");
    return false;
  }

  std::string line;

  // ヘッダーを読み込む
  if(!std::getline(file, line)) {
    Logger::error("ColorDataManager: csv file is empty");
    return false;
  }

  // 各行を読み込む
  while(std::getline(file, line)) {
    std::stringstream stream(line);

    std::string colorStr;
    std::string hStr;
    std::string sStr;
    std::string vStr;
    std::string acquiredStr;

    if(!std::getline(stream, colorStr, ',') || !std::getline(stream, hStr, ',')
       || !std::getline(stream, sStr, ',') || !std::getline(stream, vStr, ',')
       || !std::getline(stream, acquiredStr)) {
      continue;
    }

    // Windows形式の改行コードによる末尾のCRを除去する
    if(!acquiredStr.empty() && acquiredStr.back() == '\r') {
      acquiredStr.pop_back();
    }

    try {
      ColorData data;

      data.color = ColorSensorController::convertStringToColor(colorStr);

      if(data.color == ColorSensorController::COLOR::NONE) {
        continue;
      }

      // 数値全体が正しく変換されたことを確認する
      size_t hPos = 0;
      size_t sPos = 0;
      size_t vPos = 0;

      const unsigned long h = std::stoul(hStr, &hPos);
      const unsigned long s = std::stoul(sStr, &sPos);
      const unsigned long v = std::stoul(vStr, &vPos);

      if(hPos != hStr.size() || sPos != sStr.size() || vPos != vStr.size()) {
        continue;
      }

      // Hは0～360度、SとVは0～255を想定する
      if(h > 360 || s > std::numeric_limits<uint8_t>::max()
         || v > std::numeric_limits<uint8_t>::max()) {
        continue;
      }

      data.hsv.h = static_cast<uint16_t>(h);
      data.hsv.s = static_cast<uint8_t>(s);
      data.hsv.v = static_cast<uint8_t>(v);

      if(acquiredStr == "true" || acquiredStr == "1") {
        data.acquired = true;
      } else if(acquiredStr == "false" || acquiredStr == "0") {
        data.acquired = false;
      } else {
        continue;
      }

      updateColorData(data);

    } catch(...) {
      Logger::error("ColorDataManager: invalid csv data");
      continue;
    }
  }

  if(file.bad()) {
    Logger::error("ColorDataManager: failed to read csv file");
    return false;
  }

  return true;
}

// 色データをCSVファイルに保存する
bool ColorDataManager::saveToCsv() const
{
  const std::filesystem::path outputPath(fileName);

  // 保存先ディレクトリが存在しない場合は作成する
  if(!outputPath.parent_path().empty()) {
    try {
      std::filesystem::create_directories(outputPath.parent_path());
    } catch(...) {
      Logger::error("ColorDataManager: failed to create directory");
      return false;
    }
  }

  std::ofstream file(outputPath, std::ios::trunc);

  if(!file.is_open()) {
    Logger::error("ColorDataManager: failed to open csv file");
    return false;
  }

  file << "color,h,s,v,acquired\n";

  for(const ColorData& data : colorDataList) {
    file << ColorSensorController::convertColorToString(data.color) << "," << data.hsv.h << ","
         << static_cast<int>(data.hsv.s) << "," << static_cast<int>(data.hsv.v) << ","
         << (data.acquired ? "true" : "false") << "\n";
  }

  file.flush();

  if(!file.good()) {
    Logger::error("ColorDataManager: failed to write csv file");
    return false;
  }

  file.close();

  if(file.fail()) {
    Logger::error("ColorDataManager: failed to close csv file");
    return false;
  }

  return true;
}

// 指定した色のデータを更新する
bool ColorDataManager::updateColorData(const ColorData& data)
{
  for(ColorData& currentData : colorDataList) {
    if(currentData.color == data.color) {
      currentData = data;
      return true;
    }
  }

  return false;
}

// 指定した色のデータを取得する
const ColorData* ColorDataManager::getColorData(ColorSensorController::COLOR color) const
{
  for(const ColorData& data : colorDataList) {
    if(data.color == color) {
      return &data;
    }
  }

  return nullptr;
}

// CSVファイルの保存先を変更する
void ColorDataManager::setFileName(const std::string& name, const std::string& path)
{
  const std::string finalName = name.empty() ? DEFAULT_COLOR_DATA_NAME : name;
  const std::string finalPath = path.empty() ? DEFAULT_COLOR_DATA_PATH : path;

  fileName = (std::filesystem::path(finalPath) / finalName).string();
}
