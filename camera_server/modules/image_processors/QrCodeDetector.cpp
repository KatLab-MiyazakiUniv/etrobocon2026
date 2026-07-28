/**
 * @file   QrCodeDetector.cpp
 * @brief  QRコード検出処理クラス
 * @author HaruArima08
 */

#include "QrCodeDetector.h"

QrCodeDetector::QrCodeDetector()
{
  // QRコードのみを検出対象として設定
  options.setFormats(ZXing::BarcodeFormat::QRCode);
  // QRコードを複数の向きやサイズで詳細に探索し、検出・デコードを行う
  options.setTryHarder(true);
  LOG_CREATE("QrCodeDetector");
}

QrCodeDetector::~QrCodeDetector()
{
  LOG_DESTROY("QrCodeDetector");
}

QrCodeDetectionResult QrCodeDetector::detect(const cv::Mat& frame)
{
  QrCodeDetectionResult result;

  if(frame.empty()) {
    Logger::error("QrCodeDetector: 入力フレームが空です。");
    return result;
  }

//① 元のコード
ZXing::ImageView iv(frame.data, frame.cols, frame.rows,
                    ZXing::ImageFormat::BGR,
                    static_cast<int>(frame.step));




/*② グレースケールのみ
cv::Mat gray;
cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

ZXing::ImageView iv(gray.data, gray.cols, gray.rows,
                    ZXing::ImageFormat::Lum,
                    static_cast<int>(gray.step));
*/



/*③ グレースケール＋シャープ化
cv::Mat gray;
cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

cv::Mat sharpened;
cv::Mat kernel = (cv::Mat_<float>(3, 3) <<
     0, -1,  0,
    -1,  5, -1,
     0, -1,  0);

cv::filter2D(gray, sharpened, -1, kernel);

ZXing::ImageView iv(sharpened.data,
                    sharpened.cols,
                    sharpened.rows,
                    ZXing::ImageFormat::Lum,
                    static_cast<int>(sharpened.step));
*/




/*④ グレースケール＋ノイズ除去
cv::Mat gray;
cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

// ノイズ除去（どちらか一方を使用）
cv::medianBlur(gray, gray, 3);
// cv::GaussianBlur(gray, gray, cv::Size(3,3), 0);

ZXing::ImageView iv(gray.data,
                    gray.cols,
                    gray.rows,
                    ZXing::ImageFormat::Lum,
                    static_cast<int>(gray.step));
*/



/*⑤ グレースケール＋二値化
cv::Mat gray;
cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

cv::Mat binary;
cv::adaptiveThreshold(gray,
                      binary,
                      255,
                      cv::ADAPTIVE_THRESH_GAUSSIAN_C,
                      cv::THRESH_BINARY,
                      21,
                      10);

ZXing::ImageView iv(binary.data,
                    binary.cols,
                    binary.rows,
                    ZXing::ImageFormat::Lum,
                    static_cast<int>(binary.step));
*/





/*⑥ 一番おすすめ（全部入り）
cv::Mat gray;
cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

// ノイズ除去
cv::medianBlur(gray, gray, 3);

// シャープ化
cv::Mat sharpened;
cv::Mat kernel = (cv::Mat_<float>(3,3) <<
     0,-1,0,
    -1, 5,-1,
     0,-1,0);

cv::filter2D(gray, sharpened, -1, kernel);

// 二値化
cv::Mat binary;
cv::adaptiveThreshold(sharpened,
                      binary,
                      255,
                      cv::ADAPTIVE_THRESH_GAUSSIAN_C,
                      cv::THRESH_BINARY,
                      21,
                      10);

ZXing::ImageView iv(binary.data,
                    binary.cols,
                    binary.rows,
                    ZXing::ImageFormat::Lum,
                    static_cast<int>(binary.step));
*/







  // 処理対象のフレームからQRコードを検出し、デコード結果を取得
  auto qrCode = ZXing::ReadBarcode(iv, options);

  // QRコードが検出できなかった場合は終了
  if(!qrCode.isValid()) {
    Logger::error("QrCodeDetector: QRコードの検出に失敗しました。");
    return result;
  }

  // 検出結果を保存
  result.wasDetected = true;
  result.content = qrCode.text();

  // QRコードの4頂点座標を保存
  auto position = qrCode.position();
  result.corners[0] = cv::Point2f(position.topLeft().x, position.topLeft().y);
  result.corners[1] = cv::Point2f(position.topRight().x, position.topRight().y);
  result.corners[2] = cv::Point2f(position.bottomRight().x, position.bottomRight().y);
  result.corners[3] = cv::Point2f(position.bottomLeft().x, position.bottomLeft().y);

  return result;
}