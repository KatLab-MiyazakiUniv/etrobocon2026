/**
 * @file   FrameSave.cpp
 * @brief  フレームを保存するクラス
 * @author okuyama0528 sadomiya-sousi
 */

#include "FrameSave.h"

void FrameSave::save(cv::Mat& frame, const std::string& filePath, const std::string& fileName)
{
  if(frame.empty()) {
    Logger::error("FrameSave:保存するフレームがありません。");
    return;
  }

  if(!std::filesystem::exists(filePath)) {
    if(!std::filesystem::create_directories(filePath)) {
      Logger::printfLog(Logger::ERROR, "FrameSave:ディレクトリの作成に失敗しました: %s",
                        filePath.c_str());
      return;
    }
  }

  std::string imagePath = filePath + "/" + fileName + ".JPEG";
  if(!cv::imwrite(imagePath, frame)) {
    Logger::printfLog(Logger::ERROR, "FrameSave:フレームの保存に失敗しました: %s",
                      imagePath.c_str());
    return;
  }
  return;
}

void FrameSave::save(cv::Mat& frame, const std::string& filePath,
                     const BoundingBoxDetectionResult& result, const cv::Rect& roi)
{
  // clang-format off
  std::string fileName = "det_d" + std::to_string(result.wasDetected ? 1 : 0) +
                          "_tlx" + toStrInt(result.topLeft.x) +
                          "_tly" + toStrInt(result.topLeft.y) +
                          "_trx" + toStrInt(result.topRight.x) +
                          "_try" + toStrInt(result.topRight.y) +
                          "_blx" + toStrInt(result.bottomLeft.x) +
                          "_bly" + toStrInt(result.bottomLeft.y) +
                          "_brx" + toStrInt(result.bottomRight.x) +
                          "_bry" + toStrInt(result.bottomRight.y) +
                          "_rx"  + toStrInt(roi.x) +
                          "_ry"  + toStrInt(roi.y) +
                          "_rw"  + toStrInt(roi.width) +
                          "_rh"  + toStrInt(roi.height) +
                          "_"    + std::to_string(ClockUtil::now());
  // clang-format on

  save(frame, filePath, fileName);
}

std::string FrameSave::toStrInt(double value)
{
  return std::to_string(static_cast<int>(value));
}
