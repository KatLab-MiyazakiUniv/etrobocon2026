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
                          "_tlx" + Utils::toStrInt(result.topLeft.x) +
                          "_tly" + Utils::toStrInt(result.topLeft.y) +
                          "_trx" + Utils::toStrInt(result.topRight.x) +
                          "_try" + Utils::toStrInt(result.topRight.y) +
                          "_blx" + Utils::toStrInt(result.bottomLeft.x) +
                          "_bly" + Utils::toStrInt(result.bottomLeft.y) +
                          "_brx" + Utils::toStrInt(result.bottomRight.x) +
                          "_bry" + Utils::toStrInt(result.bottomRight.y) +
                          "_rx"  + Utils::toStrInt(roi.x) +
                          "_ry"  + Utils::toStrInt(roi.y) +
                          "_rw"  + Utils::toStrInt(roi.width) +
                          "_rh"  + Utils::toStrInt(roi.height) +
                          "_"    + std::to_string(ClockUtil::now());
  // clang-format on

  save(frame, filePath, fileName);
}

void FrameSave::save(cv::Mat& frame,
                     const std::string& filePath,
                     const QrCodeDetectionResult& result,
                     const cv::Rect& roi)
{
  std::string fileName =
      "det_d" + std::to_string(result.wasDetected ? 1 : 0) +
      "_tlx" + Utils::toStrInt(result.corners[0].x) +
      "_tly" + Utils::toStrInt(result.corners[0].y) +
      "_trx" + Utils::toStrInt(result.corners[1].x) +
      "_try" + Utils::toStrInt(result.corners[1].y) +
      "_blx" + Utils::toStrInt(result.corners[3].x) +
      "_bly" + Utils::toStrInt(result.corners[3].y) +
      "_brx" + Utils::toStrInt(result.corners[2].x) +
      "_bry" + Utils::toStrInt(result.corners[2].y) +
      "_rx" + Utils::toStrInt(roi.x) +
      "_ry" + Utils::toStrInt(roi.y) +
      "_rw" + Utils::toStrInt(roi.width) +
      "_rh" + Utils::toStrInt(roi.height) +
      "_" + std::to_string(ClockUtil::now());

  save(frame, filePath, fileName);
}