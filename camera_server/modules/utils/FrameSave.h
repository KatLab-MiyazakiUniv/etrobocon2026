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

class FrameSave {
 public:
  /**
   * @brief フレームを保存する
   * @param frame 保存するフレームを格納するcv::Mat参照
   * @param filePath　フレーム保存先のディレクトリパス
   * @param fileName 保存するフレームの名前
   */
  static void save(cv::Mat& frame, const std::string& filePath, const std::string& fileName);

 private:
  FrameSave();  // インスタンス化の禁止
};

#endif  // FRAME_SAVE_H