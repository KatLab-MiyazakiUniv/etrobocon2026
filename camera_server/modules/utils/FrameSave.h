/**
 * @file   FrameSave.h
 * @brief  フレームを保存するクラス
 * @author okuyama0528
 */

#ifndef FRAME_SAVE_H
#define FRAME_SAVE_H

#include <opencv2/opencv.hpp>
#include <filesystem>
#include <string>
#include "Logger.h"
#include "ImageRecognitionResults.h"

class FrameSave {
 public:
  /**
   * @brief フレームを保存する
   * @param frame 保存するフレームを格納するcv::Mat参照
   * @param filePath　フレーム保存先のディレクトリパス
   * @param fileName 保存するフレームの名前
   */
  static void save(cv::Mat& frame, const std::string& filePath, const std::string& fileName);

  /**
   * @brief 検出結果を含んだファイル名でフレームを保存する
   * @param frame 保存するフレームを格納するcv::Mat参照
   * @param filePath フレーム保存先のディレクトリパス
   * @param result 検出結果を格納するBoundingBoxDetectionResult構造体の参照
   */
  static void save(cv::Mat& frame, const std::string& filePath,
                   const BoundingBoxDetectionResult& result);

 private:
  FrameSave();  // インスタンス化の禁止
};

#endif  // FRAME_SAVE_H