/**
 * @file   CameraCapture.cpp
 * @brief  カメラを制御するクラス
 * @author okuyama0528
 */

#include "CameraCapture.h"
#include <filesystem>

using namespace std;

CameraCapture::CameraCapture()
  : cap(),       // カメラデバイス（未接続で初期化）
    cameraID(0)  // カメラIDを0で初期化
{
  LOG_CREATE("CameraCapture");
}

CameraCapture::~CameraCapture()
{
  if(cap.isOpened()) {
    cap.release();
  }
  LOG_DESTROY("CameraCapture");
}

int CameraCapture::findAvailableCameraID(int maxTested)
{
  for(int i = 0; i < maxTested; ++i) {
    cv::VideoCapture cap(i);
    if(cap.isOpened()) {
      cap.release();
      // 利用可能なIDを返す
      Logger::printfLog(Logger::INFO, "カメラID: %d が見つかりました", i);
      return i;
    }
  }
  // どのIDも利用できなかった場合は-1を返す
  Logger::error("使用できるカメラIDが見つかりません");
  return -1;
}

int CameraCapture::getCameraID()
{
  return cameraID;
}

bool CameraCapture::setCameraID(int id)
{
  if(id < 0) {
    Logger::error("無効なカメラIDです");
    return false;
  }
  cameraID = id;
  return true;
}

bool CameraCapture::openCamera()
{
  // V4L2を指定してカメラを開く
  cap.open(cameraID, cv::CAP_V4L2);
  if(!cap.isOpened()) {
    Logger::error("カメラを開くことができませんでした。");
    return false;
  }

  // MJPEG形式に設定
  if(!cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'))) {
    Logger::error("MJPEG形式の設定に失敗しました。");
    return false;
  }

  // FPSを30に設定
  if(!cap.set(cv::CAP_PROP_FPS, 30)) {
    Logger::error("FPS設定に失敗しました。");
    return false;
  }

  // 解像度設定
  if(!cap.set(cv::CAP_PROP_FRAME_WIDTH, CAM_MAX_WIDTH)) {
    Logger::error("幅設定に失敗しました。");
    return false;
  }

  if(!cap.set(cv::CAP_PROP_FRAME_HEIGHT, CAM_MAX_HEIGHT)) {
    Logger::error("高さ設定に失敗しました。");
    return false;
  }

  // 設定後の確認
  double width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
  double height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
  double fps = cap.get(cv::CAP_PROP_FPS);
  Logger::printfLog(Logger::INFO, "設定後の解像度: %fx%f, FPS: %f", width, height, fps);

  return true;
}

void CameraCapture::setCapProps(double width, double height)
{
  cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
}

bool CameraCapture::getFrame(cv::Mat& outFrame)
{
  if(!cap.isOpened()) {
    Logger::error("カメラを開いていません");
    return false;
  }

  for(int i = 0; i < 5; i++) {
    cap >> outFrame;
    if(!outFrame.empty()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  Logger::error("フレームの取得に失敗しました。");
  return false;
}

bool CameraCapture::getFrames(vector<cv::Mat>& frames, int numFrames, int millisecondInterval)
{
  if(numFrames <= 0) {
    Logger::printfLog(Logger::ERROR, "フレーム数が無効です。動作を終了します。: %d", numFrames);
    return false;
  }
  if(millisecondInterval <= 0) {
    Logger::printfLog(Logger::ERROR, "インターバルが無効です。動作を終了します。: %d ms",
                      millisecondInterval);
    return false;
  }

  frames.resize(numFrames);
  bool allSuccess = true;
  for(int i = 0; i < numFrames; ++i) {
    if(!getFrame(frames[i])) {
      Logger::printfLog(Logger::ERROR, "フレーム %d の取得に失敗しました。", i);
      allSuccess = false;
    }
    if(i < numFrames - 1) {
      // 最後の1回以外は、milliseconds ミリ秒だけ待機
      this_thread::sleep_for(chrono::milliseconds(millisecondInterval));
    }
  }
  return allSuccess;
}