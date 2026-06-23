/**
 * @file   FrameSave.cpp
 * @brief  フレームを保存するクラス
 * @author okuyama0528
 */

#include "FrameSave.h"
#include "ClockUtil.h"

void FrameSave::save(cv::Mat& frame, const std::string& filePath, const std::string& fileName)
{
  if(frame.empty()) {
    Logger::error("保存するフレームがありません。");
  }

  // ディレクトリが存在しない場合は作成
  if(!std::filesystem::exists(filePath)) {
    if(!std::filesystem::create_directories(filePath)) {
      Logger::printfLog(Logger::ERROR, "ディレクトリの作成に失敗しました: %s", filePath.c_str());
    }
  }

  std::string imagePath = filePath + "/" + fileName + ".JPEG";
  if(!cv::imwrite(imagePath, frame)) {
    Logger::printfLog(Logger::ERROR, "フレームの保存に失敗しました: %s", imagePath.c_str());
  }
}

void FrameSave::save(cv::Mat& frame, const std::string& filePath,
                     const BoundingBoxDetectionResult& result)
{
  std::string fileName = "det_d" + std::to_string(result.wasDetected ? 1 : 0)
                         + "_tlx" + std::to_string(result.topLeft.x)
                         + "_tly" + std::to_string(result.topLeft.y)
                         + "_trx" + std::to_string(result.topRight.x)
                         + "_try" + std::to_string(result.topRight.y)
                         + "_blx" + std::to_string(result.bottomLeft.x)
                         + "_bly" + std::to_string(result.bottomLeft.y)
                         + "_brx" + std::to_string(result.bottomRight.x)
                         + "_bry" + std::to_string(result.bottomRight.y)
                         + "_" + std::to_string(ClockUtil::now());
  save(frame, filePath, fileName);
}
