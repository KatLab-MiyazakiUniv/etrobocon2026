/**
 * @file   CameraCapture.h
 * @brief  カメラを制御するクラス
 * @author okuyama0528
 */

#ifndef CAMERA_CAPTURE_H
#define CAMERA_CAPTURE_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <thread>
#include <chrono>

class CameraCapture {
 public:
  /**
   * @brief コンストラクタ
   */
  CameraCapture();

  /**
   * @brief デストラクタ
   */
  ~CameraCapture();

  /**
   * @brief 利用可能なカメラIDを探索する
   * @param maxTested 探索する回数の上限
   * @return 利用可能なカメラID（見つからなければ-1）
   */
  int findAvailableCameraID(int maxTested = 10);

  /**
   * @brief クラスの変数に格納されている現在のカメラIDを取得する
   * @return 現在のクラスの変数に格納されているカメラID
   */
  int getCameraID();

  /**
   * @brief カメラIDをクラスの変数に格納
   * @param id クラスの変数に格納するカメラid
   * @return 成功した場合はtrue、無効な値の場合はfalse
   */
  bool setCameraID(int id);

  /**
   * @brief カメラデバイスをオープンする
   * @return openに成功した場合はtrue、失敗した場合はfalse、
   */
  bool openCamera();

  /**
   * @brief カメラフレームの高さと幅を設定する
   * @param width 設定するフレームの幅 (px)
   * @param height 設定するフレームの高さ (px)
   */
  void setCapProps(double width, double height);

  /**
   * @brief 1枚のカメラフレームを取得する
   * @param outFrame 取得したフレームを格納するcv::Mat参照
   * @return フレーム取得に成功した場合はtrue、失敗した場合はfalse
   */
  bool getFrame(cv::Mat& outFrame);

  /**
   * @brief 指定した枚数だけ、指定したミリ秒間隔でカメラフレームを取得し、配列に保存する
   * @param frames 取得したフレームを格納するstd::vector<cv::Mat>型の参照
   * @param numFrames 取得するフレームの枚数
   * @param millisecondInterval フレーム取得の間隔（ミリ秒）
   * @return 全てのフレーム取得に成功した場合はtrue、1つでも失敗した場合はfalse
   */
  bool getFrames(std::vector<cv::Mat>& frames, int numFrames, int millisecondInterval);

 private:
  cv::VideoCapture cap;  // カメラデバイス
  int cameraID;          // カメラID
};

#endif