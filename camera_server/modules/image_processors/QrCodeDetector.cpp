/**
 * @file   QrCodeDetector.cpp
 * @brief  QRコード検出処理クラス
 * @author HaruArima08 sadomiya-sousi
 */

#include "QrCodeDetector.h"

QrCodeDetector::QrCodeDetector(const cv::Rect& _roi) : roi(_roi)
{
  // QRコードのみを検出対象として設定
  options.setFormats(ZXing::BarcodeFormat::QRCode);
  // QRコードを複数の向きやサイズで詳細に探索し、検出・デコードを行う
  options.setTryHarder(true);

  // デバッグのために追加
  options.setReturnErrors(true);

  validateParameters();
  LOG_CREATE("QrCodeDetector");
}

QrCodeDetector::~QrCodeDetector()
{
  LOG_DESTROY("QrCodeDetector");
}

void QrCodeDetector::setValidatedRoi(const cv::Rect& _roi)
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
  // auto qrCode = ZXing::ReadBarcode(iv, options);

  // QRコードが検出できなかった場合は終了
  // if(!qrCode.isValid()) {
  //   Logger::error("QrCodeDetector: QRコードの検出に失敗しました。");
  //   return result;
  // }

  // ROI内のフレームからQRコードを検出し、デコード結果を取得
  auto qrCode = ZXing::ReadBarcode(iv, options);

  // 1. 完全成功（検出 OK ＆ デコード OK）
  if(qrCode.isValid()) {
    // 検出結果を保存
    // result.wasDetected = true;
    // result.content = qrCode.text();

    // QRコードの4頂点座標を、ROIのオフセットを加算してフレーム全体基準の座標に変換して保存
    // auto position = qrCode.position();
    // result.corners[0]
    //     = cv::Point2f(position.topLeft().x + roiRect.x, position.topLeft().y + roiRect.y);
    // result.corners[1]
    //     = cv::Point2f(position.topRight().x + roiRect.x, position.topRight().y + roiRect.y);
    // result.corners[2]
    //     = cv::Point2f(position.bottomRight().x + roiRect.x, position.bottomRight().y +
    //     roiRect.y);
    // result.corners[3]
    //     = cv::Point2f(position.bottomLeft().x + roiRect.x, position.bottomLeft().y + roiRect.y);

    // return result;

    result.wasDetected = true;
    result.content = qrCode.text();

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

  // 2. 検出は成功したが、デコード（復号）フェーズで失敗した場合
  if(qrCode.format() != ZXing::BarcodeFormat::None) {
    auto position = qrCode.position();
    cv::Point2f tl(position.topLeft().x + roiRect.x, position.topLeft().y + roiRect.y);
    cv::Point2f br(position.bottomRight().x + roiRect.x, position.bottomRight().y + roiRect.y);

    // エラー種別（ChecksumError, FormatError など）とメッセージを取得
    std::string errorType = ZXing::ToString(qrCode.error().type());
    std::string errorMsg = qrCode.error().msg();

    Logger::printfLog(
        Logger::DEBUG,
        "QrCodeDetector: [デコード失敗] QRコードの位置は検出できましたが、復号に失敗しました。"
        " エラー種別: %s (%s) 推定位置(ROI加算後): TL(%d, %d) BR(%d, %d)",
        errorType.c_str(), errorMsg.c_str(), static_cast<int>(tl.x), static_cast<int>(tl.y),
        static_cast<int>(br.x), static_cast<int>(br.y));

    // ※用途に応じて「位置だけは取れた」として result.corners を詰めて返す設計も可能です
    return result;
  }

  // 3. そもそもQRコードの位置パターンが見つからなかった場合（検出失敗）
  Logger::debug("QrCodeDetector: [検出失敗] ROI内にQRコードのパターンが見つかりませんでした。");
  return result;
}
