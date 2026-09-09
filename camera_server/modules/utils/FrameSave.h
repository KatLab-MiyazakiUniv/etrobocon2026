/**
 * @file   FrameSave.h
 * @brief  フレームを保存するクラス
 * @author okuyama0528 sadomiya-sousi
 */

#ifndef FRAME_SAVE_H
#define FRAME_SAVE_H

#include <opencv2/opencv.hpp>
#include <filesystem>
#include "Logger.h"
#include "Utils.h"

#include "ClockUtil.h"
#include "ImageRecognitionResults.h"
#include "QrCodeDetectionResult.h"

class FrameSave {
 public:
  /**
   * @brief フレームを保存する
   * @param frame 保存するフレームを格納するcv::Mat参照
   * @param filePath フレーム保存先のディレクトリパス
   * @param fileName 保存するフレームの名前
   */
  static void save(cv::Mat& frame, const std::string& filePath, const std::string& fileName);

  /**
   * @brief 検出結果とROIを含んだファイル名を作成し,フレームを保存する
   * @param frame 保存するフレームを格納するcv::Mat参照
   * @param filePath フレーム保存先のディレクトリパス
   * @param result 検出結果を格納するBoundingBoxDetectionResult構造体の参照
   * @param roi 検出対象の領域(ROI)
   */
  static void save(cv::Mat& frame, const std::string& filePath,
                   const BoundingBoxDetectionResult& result, const cv::Rect& roi);

  /**
   * @brief 色領域検出結果とQRコード検出結果、ROIを含んだファイル名を作成し,フレームを保存する
   * @param frame 保存するフレームを格納するcv::Mat参照
   * @param filePath フレーム保存先のディレクトリパス
   * @param result 色領域検出結果を格納するBoundingBoxDetectionResult構造体の参照
   * @param roi 検出対象の領域(ROI)
   * @param qrResult QRコード検出結果を格納するQrCodeDetectionResult構造体の参照
   */
  static void save(cv::Mat& frame, const std::string& filePath,
                   const BoundingBoxDetectionResult& result, const cv::Rect& roi,
                   const QrCodeDetectionResult& qrResult);

  /**
   * @brief QRコード検出結果とROIを含んだファイル名を作成し,フレームを保存する
   * @param frame 保存するフレームを格納するcv::Mat参照
   * @param filePath フレーム保存先のディレクトリパス
   * @param qrResult QRコード検出結果を格納するQrCodeDetectionResult構造体の参照
   * @param roi 検出対象の領域(ROI)
   */
  static void save(cv::Mat& frame, const std::string& filePath,
                   const QrCodeDetectionResult& qrResult, const cv::Rect& roi);

 private:
  FrameSave();  // インスタンス化の禁止
};

#endif  // FRAME_SAVE_H