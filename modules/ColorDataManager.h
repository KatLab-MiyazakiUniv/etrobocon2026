/**
 * @file ColorDataManager.h
 * @brief HSV基準値の管理とCSVファイルの読み書きを行うクラス
 */

#ifndef COLOR_DATA_MANAGER_H
#define COLOR_DATA_MANAGER_H

#include <string>
#include <vector>
#include "ColorData.h"

#define DEFAULT_COLOR_DATA_PATH "etrobocon2026/datafiles/"
#define DEFAULT_COLOR_DATA_NAME "ColorData.csv"

class ColorDataManager {
 public:
  ColorDataManager();

  /**
   * @brief CSVファイルから色データを読み込む
   */
  bool loadFromCsv();

  /**
   * @brief 色データをCSVファイルに保存する
   */
  bool saveToCsv() const;

  /**
   * @brief 指定した色のデータを更新する
   */
  bool updateColorData(const ColorData& data);

  /**
   * @brief 指定した色のデータを取得する
   */
  const ColorData* getColorData(ColorSensorController::COLOR color) const;

  /**
   * @brief CSVファイルの保存先を変更する
   */
  void setFileName(const std::string& name, const std::string& path);

 private:
  std::vector<ColorData> colorDataList;
  std::string fileName;

  /**
   * @brief 全色のデータを初期化する
   */
  void initializeColorData();
};

#endif