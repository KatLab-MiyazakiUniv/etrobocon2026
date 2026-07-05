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
                     const BoundingBoxDetectionResult& result, const cv::Rect& roi)
{
  double scaleX = 640.0 / frame.cols;
  double scaleY = 360.0 / frame.rows;

  // ROIとフレームの重複領域（切り出し領域）を取得してスケーリング
  cv::Rect roiRect = roi & cv::Rect(0, 0, frame.cols, frame.rows);

  std::string fileName = "det_d" + std::to_string(result.wasDetected ? 1 : 0) + "_tlx"
                         + std::to_string(static_cast<int>(result.topLeft.x * scaleX)) + "_tly"
                         + std::to_string(static_cast<int>(result.topLeft.y * scaleY)) + "_trx"
                         + std::to_string(static_cast<int>(result.topRight.x * scaleX)) + "_try"
                         + std::to_string(static_cast<int>(result.topRight.y * scaleY)) + "_blx"
                         + std::to_string(static_cast<int>(result.bottomLeft.x * scaleX)) + "_bly"
                         + std::to_string(static_cast<int>(result.bottomLeft.y * scaleY)) + "_brx"
                         + std::to_string(static_cast<int>(result.bottomRight.x * scaleX)) + "_bry"
                         + std::to_string(static_cast<int>(result.bottomRight.y * scaleY)) + "_rx"
                         + std::to_string(static_cast<int>(roiRect.x * scaleX)) + "_ry"
                         + std::to_string(static_cast<int>(roiRect.y * scaleY)) + "_rw"
                         + std::to_string(static_cast<int>(roiRect.width * scaleX)) + "_rh"
                         + std::to_string(static_cast<int>(roiRect.height * scaleY)) + "_"
                         + std::to_string(ClockUtil::now());
  cv::Mat resized;
  cv::resize(frame, resized, cv::Size(640, 360), 0, 0, cv::INTER_LINEAR);
  // cv::resize(frame, resized, cv::Size(1920, 1080), 0, 0, cv::INTER_LINEAR);
  save(resized, filePath, fileName);
}
