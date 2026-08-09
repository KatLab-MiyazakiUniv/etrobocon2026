/**
 * @file   QrCodeDetector.cpp
 * @brief  QRコード検出処理クラス
 * @author HaruArima08
 */

#include "QrCodeDetector.h"

QrCodeDetector::QrCodeDetector(const cv::Rect& _roi) : roi(_roi)
{
  // QRコードのみを検出対象として設定
  options.setFormats(ZXing::BarcodeFormat::QRCode);
  // QRコードを複数の向きやサイズで詳細に探索し、検出・デコードを行う
  options.setTryHarder(true);
  validateParameters();
  LOG_CREATE("QrCodeDetector");
}

QrCodeDetector::~QrCodeDetector()
{
  LOG_DESTROY("QrCodeDetector");
}

void QrCodeDetector::setRoi(const cv::Rect& _roi)
{
  roi = _roi;
  validateParameters();
}

void QrCodeDetector::validateParameters()
{
  if(roi.x < 0) roi.x = 0;
  if(roi.y < 0) roi.y = 0;
  if(roi.x > CAM_MAX_WIDTH) roi.x = CAM_MAX_WIDTH;
  if(roi.y > CAM_MAX_HEIGHT) roi.y = CAM_MAX_HEIGHT;
  if(roi.width < 0) roi.width = 0;
  if(roi.height < 0) roi.height = 0;
  if(roi.width > CAM_MAX_WIDTH - roi.x) roi.width = CAM_MAX_WIDTH - roi.x;
  if(roi.height > CAM_MAX_HEIGHT - roi.y) roi.height = CAM_MAX_HEIGHT - roi.y;
}

QrCodeDetectionResult QrCodeDetector::detect(const cv::Mat& frame)
{
  QrCodeDetectionResult result;

  if(frame.empty()) {
    Logger::error("QrCodeDetector: 入力フレームが空です。");
    return result;
  }

  // ROI切り出し
  cv::Rect roiRect = roi & cv::Rect(0, 0, frame.cols, frame.rows);
  if(roiRect.empty()) {
    Logger::error("QrCodeDetector: ROIがフレーム内に収まっていません。");
    return result;
  }
  cv::Mat roiFrame = frame(roiRect);

  // ROI内のフレームからZXing用のImageViewを生成
  ZXing::ImageView iv(roiFrame.data, roiFrame.cols, roiFrame.rows, ZXing::ImageFormat::BGR,
                      static_cast<int>(roiFrame.step));

  // ROI内のフレームからQRコードを検出し、デコード結果を取得
  auto qrCode = ZXing::ReadBarcode(iv, options);

  // QRコードが検出できなかった場合は終了
  if(!qrCode.isValid()) {
    Logger::error("QrCodeDetector: QRコードの検出に失敗しました。");
    return result;
  }

  // 検出結果を保存
  result.wasDetected = true;
  result.content = qrCode.text();

  // QRコードの4頂点座標を、ROIのオフセットを加算してフレーム全体基準の座標に変換して保存
  auto position = qrCode.position();
  result.corners[0]
      = cv::Point2f(position.topLeft().x + roiRect.x, position.topLeft().y + roiRect.y);
  result.corners[1]
      = cv::Point2f(position.topRight().x + roiRect.x, position.topRight().y + roiRect.y);
  result.corners[2]
      = cv::Point2f(position.bottomRight().x + roiRect.x, position.bottomRight().y + roiRect.y);
  result.corners[3]
      = cv::Point2f(position.bottomLeft().x + roiRect.x, position.bottomLeft().y + roiRect.y);

  return result;
}
