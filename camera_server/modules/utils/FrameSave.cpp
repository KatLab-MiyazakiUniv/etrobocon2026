/**
 * @file   FrameSave.cpp
 * @brief  フレームを保存するクラス
 * @author okuyama0528 sadomiya-sousi
 */

#include "FrameSave.h"
#include "ClockUtil.h"

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
  double scaleX = 640.0 / frame.cols;
  double scaleY = 360.0 / frame.rows;
  std::string fileName
      = "det_d" + std::to_string(result.wasDetected ? 1 : 0) +
      "_tlx" + toStrInt(result.topLeft.x * scaleX) +
      "_tly"+ toStrInt(result.topLeft.y * scaleY) +
      "_trx" + toStrInt(result.topRight.x * scaleX) +
      "_try" + toStrInt(result.topRight.y * scaleY) +
      "_blx" + toStrInt(result.bottomLeft.x * scaleX) +
      "_bly" + toStrInt(result.bottomLeft.y * scaleY) +
      "_brx" + toStrInt(result.bottomRight.x * scaleX) +
      "_bry" + toStrInt(result.bottomRight.y * scaleY) +
      "_rx" + toStrInt(roi.x * scaleX) +
      "_ry" + toStrInt(roi.y * scaleY) +
      "_rw" + toStrInt(roi.width * scaleX) +
      "_rh" + toStrInt(roi.height * scaleY) +
      "_" + std::to_string(ClockUtil::now());

  if(frame.empty()) return;

  cv::Mat resized;
  cv::resize(frame, resized, cv::Size(640, 360), 0, 0, cv::INTER_LINEAR);

  save(resized, filePath, fileName);
}

std::string FrameSave::toStrInt(double value)
{
  return std::to_string(static_cast<int>(value));
}
