/**
 * @file MockCameraCapture.h
 * @brief テスト用のMockCameraCaptureクラス
 * @author sadomiya-sousi
 */

#ifndef MOCK_CAMERA_CAPTURE_H
#define MOCK_CAMERA_CAPTURE_H

#include "CameraCapture.h"

namespace etrobocon2026_test {

  class MockCameraCapture : public CameraCapture {
   public:
    bool mockSuccess = true;
    cv::Mat mockFrame;

    bool getFrame(cv::Mat& outFrame) override
    {
      if(!mockSuccess) {
        return false;
      }
      outFrame = mockFrame.clone();
      return true;
    }

    int clearBufferCallCount = 0;
    void clearBuffer(int dropCount = 5) override
    {
      (void)dropCount;
      clearBufferCallCount++;
    }
  };

}  // namespace etrobocon2026_test

#endif  // MOCK_CAMERA_CAPTURE_H
