// /**
//  * @file   QrCodeDetector.cpp
//  * @brief  QRコード検出処理クラス
//  * @author HaruArima08 sadomiya-sousi
//  */

// #include "QrCodeDetector.h"
// #include <opencv2/imgproc.hpp>
// #include <vector>

// QrCodeDetector::QrCodeDetector(const cv::Rect& _roi) : roi(_roi)
// {
//   // QRコードのみを検出対象として設定
//   options.setFormats(ZXing::BarcodeFormat::QRCode);
//   // QRコードを複数の向きやサイズで詳細に探索し、検出・デコードを行う
//   // 検証のために、コメントアウトし実行したが、検出成功率は変わらず、処理速度も変化なかった
//   // options.setTryHarder(true);

//   // デバッグのために追加
//   options.setReturnErrors(true);

//   validateParameters();
//   LOG_CREATE("QrCodeDetector");
// }

// QrCodeDetector::~QrCodeDetector()
// {
//   LOG_DESTROY("QrCodeDetector");
// }

// void QrCodeDetector::setValidatedRoi(const cv::Rect& _roi)
// {
//   roi = _roi;
//   validateParameters();
// }

// void QrCodeDetector::validateParameters()
// {
//   if(roi.x < 0) roi.x = 0;
//   if(roi.y < 0) roi.y = 0;
//   if(roi.x > CAM_MAX_WIDTH) roi.x = CAM_MAX_WIDTH;
//   if(roi.y > CAM_MAX_HEIGHT) roi.y = CAM_MAX_HEIGHT;
//   if(roi.width < 0) roi.width = 0;
//   if(roi.height < 0) roi.height = 0;
//   if(roi.width > CAM_MAX_WIDTH - roi.x) roi.width = CAM_MAX_WIDTH - roi.x;
//   if(roi.height > CAM_MAX_HEIGHT - roi.y) roi.height = CAM_MAX_HEIGHT - roi.y;
// }

// cv::Mat QrCodeDetector::applySharpen(const cv::Mat& src, double amount)
// {
//   cv::Mat blurred, sharp;
//   cv::GaussianBlur(src, blurred, cv::Size(0, 0), 3);
//   cv::addWeighted(src, 1.0 + amount, blurred, -amount, 0, sharp);
//   return sharp;
// }

// cv::Mat QrCodeDetector::applyCLAHE(const cv::Mat& src, double clipLimit,
//                                    const cv::Size& tileGridSize)
// {
//   cv::Mat result;
//   if(src.channels() == 3) {
//     cv::Mat lab;
//     cv::cvtColor(src, lab, cv::COLOR_BGR2Lab);
//     std::vector<cv::Mat> labPlanes;
//     cv::split(lab, labPlanes);
//     cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(clipLimit, tileGridSize);
//     clahe->apply(labPlanes[0], labPlanes[0]);
//     cv::merge(labPlanes, lab);
//     cv::cvtColor(lab, result, cv::COLOR_Lab2BGR);
//   } else {
//     cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(clipLimit, tileGridSize);
//     clahe->apply(src, result);
//   }
//   return result;
// }

// cv::Mat QrCodeDetector::applyBilateral(const cv::Mat& src, int d, double sigmaColor,
//                                        double sigmaSpace)
// {
//   cv::Mat filtered;
//   cv::bilateralFilter(src, filtered, d, sigmaColor, sigmaSpace);
//   return filtered;
// }

// QrCodeDetectionResult QrCodeDetector::createResult(const ZXing::Barcode& barcode,
//                                                    const cv::Rect& roiRect,
//                                                    const std::string& stepName)
// {
//   QrCodeDetectionResult result;
//   result.wasDetected = true;
//   result.content = barcode.text();
//   result.detectedStep = stepName;

//   auto position = barcode.position();
//   result.corners[0]
//       = cv::Point2f(position.topLeft().x + roiRect.x, position.topLeft().y + roiRect.y);
//   result.corners[1]
//       = cv::Point2f(position.topRight().x + roiRect.x, position.topRight().y + roiRect.y);
//   result.corners[2]
//       = cv::Point2f(position.bottomRight().x + roiRect.x, position.bottomRight().y + roiRect.y);
//   result.corners[3]
//       = cv::Point2f(position.bottomLeft().x + roiRect.x, position.bottomLeft().y + roiRect.y);

//   Logger::info("!!!!!!!復号処理成功!!!!!!");
//   return result;
// }

// QrCodeDetectionResult QrCodeDetector::detect(const cv::Mat& frame)
// {
//   QrCodeDetectionResult result;

//   if(frame.empty()) {
//     Logger::error("QrCodeDetector: 入力フレームが空です。");
//     return result;
//   }

//   // ROI切り出し
//   cv::Rect roiRect = roi & cv::Rect(0, 0, frame.cols, frame.rows);
//   if(roiRect.empty()) {
//     Logger::error("QrCodeDetector: ROIがフレーム内に収まっていません。");
//     return result;
//   }
//   cv::Mat roiFrame = frame(roiRect);

//   // --- Step 1: 生画像 (Raw) でデコード試行 ---
//   ZXing::ImageView ivRaw(roiFrame.data, roiFrame.cols, roiFrame.rows, ZXing::ImageFormat::BGR,
//                          static_cast<int>(roiFrame.step));
//   auto qrCode = ZXing::ReadBarcode(ivRaw, options);
//   if(qrCode.isValid()) {
//     return createResult(qrCode, roiRect, "Raw");
//   }

//   // --- Step 2: シャープ化 + CLAHE (コントラスト強調 & 輪郭強調) でデコード試行 ---
//   cv::Mat imgStep2 = applySharpen(applyCLAHE(roiFrame), 1.5);
//   ZXing::ImageView ivStep2(imgStep2.data, imgStep2.cols, imgStep2.rows, ZXing::ImageFormat::BGR,
//                            static_cast<int>(imgStep2.step));
//   qrCode = ZXing::ReadBarcode(ivStep2, options);
//   if(qrCode.isValid()) {
//     return createResult(qrCode, roiRect, "Sharpen+CLAHE");
//   }

//   // --- Step 3: バイラテラル + CLAHE (エッジ保持ノイズ除去 & コントラスト強調) でデコード試行
//   --- cv::Mat imgStep3 = applyCLAHE(applyBilateral(roiFrame, 5, 50.0, 50.0)); ZXing::ImageView
//   ivStep3(imgStep3.data, imgStep3.cols, imgStep3.rows, ZXing::ImageFormat::BGR,
//                            static_cast<int>(imgStep3.step));
//   qrCode = ZXing::ReadBarcode(ivStep3, options);
//   if(qrCode.isValid()) {
//     return createResult(qrCode, roiRect, "Bilateral+CLAHE");
//   }

//   // --- Step 4: 複合処理 (バイラテラル + CLAHE + シャープ化) でデコード試行 ---
//   cv::Mat imgStep4 = applySharpen(imgStep3, 1.5);
//   ZXing::ImageView ivStep4(imgStep4.data, imgStep4.cols, imgStep4.rows, ZXing::ImageFormat::BGR,
//                            static_cast<int>(imgStep4.step));
//   qrCode = ZXing::ReadBarcode(ivStep4, options);
//   if(qrCode.isValid()) {
//     return createResult(qrCode, roiRect, "Bilateral+CLAHE+Sharpen");
//   }

//   // 検出は成功したが、デコード（復号）フェーズで失敗した場合のログ
//   if(qrCode.format() != ZXing::BarcodeFormat::None) {
//     auto position = qrCode.position();
//     cv::Point2f tl(position.topLeft().x + roiRect.x, position.topLeft().y + roiRect.y);
//     cv::Point2f br(position.bottomRight().x + roiRect.x, position.bottomRight().y + roiRect.y);

//     std::string errorType = ZXing::ToString(qrCode.error().type());
//     std::string errorMsg = qrCode.error().msg();

//     Logger::printfLog(
//         Logger::DEBUG,
//         "QrCodeDetector: [デコード失敗] QRコードの位置は検出できましたが、復号に失敗しました。"
//         " エラー種別: %s (%s) 推定位置(ROI加算後): TL(%d, %d) BR(%d, %d)",
//         errorType.c_str(), errorMsg.c_str(), static_cast<int>(tl.x), static_cast<int>(tl.y),
//         static_cast<int>(br.x), static_cast<int>(br.y));

//     return result;
//   }

//   // そもそもQRコードの位置パターンが見つからなかった場合（検出失敗）
//   Logger::debug("QrCodeDetector: [検出失敗] ROI内にQRコードのパターンが見つかりませんでした。");
//   return result;
// }

/**
 * @file    QrCodeDetector.cpp
 * @brief   QRコード検出処理クラス (WeChatQRCode版)
 * @author HaruArima08 sadomiya-sousi
 */

#include "QrCodeDetector.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/wechat_qrcode.hpp>
#include <vector>

// QrCodeDetector::QrCodeDetector(const cv::Rect& _roi) : roi(_roi)
// {
//   // WeChatQRCodeの初期化
//   // ※環境に合わせてモデルファイルのパス（.prototxt と .caffemodel）を指定してください
//   try {
//     weChatDetector = std::make_unique<cv::wechat_qrcode::WeChatQRCode>(
//         "detect.prototxt", "detect.caffemodel",
//         "sr.prototxt", "sr.caffemodel"
//     );
//   } catch (const cv::Exception& e) {
//     Logger::printfLog(Logger::ERROR, "QrCodeDetector: WeChatQRCodeの初期化に失敗しました: %s",
//     e.what());
//   }

//   validateParameters();
//   LOG_CREATE("QrCodeDetector");
// }

QrCodeDetector::QrCodeDetector(const cv::Rect& _roi) : roi(_roi)
{
  try {
    weChatDetector = std::make_unique<cv::wechat_qrcode::WeChatQRCode>(

        "./../camera_server/models/detect.prototxt", "./../camera_server/models/detect.caffemodel",
        "./../camera_server/models/sr.prototxt", "./../camera_server/models/sr.caffemodel"

        // "/home/katlab/RasPike-ART/sdk/workspace/etrobocon2026/camera_server/models/detect.prototxt",
        // "/home/katlab/RasPike-ART/sdk/workspace/etrobocon2026/camera_server/models/detect.caffemodel",
        // "/home/katlab/RasPike-ART/sdk/workspace/etrobocon2026/camera_server/models/sr.prototxt",
        // "/home/katlab/RasPike-ART/sdk/workspace/etrobocon2026/camera_server/models/sr.caffemodel"
    );
  } catch(const cv::Exception& e) {
    Logger::printfLog(Logger::ERROR, "QrCodeDetector: WeChatQRCodeの初期化に失敗しました: %s",
                      e.what());
  }

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

  if(!weChatDetector) {
    Logger::error("QrCodeDetector: WeChatQRCodeが初期化されていません。");
    return result;
  }

  // ROI切り出し
  cv::Rect roiRect = roi & cv::Rect(0, 0, frame.cols, frame.rows);
  if(roiRect.empty()) {
    Logger::error("QrCodeDetector: ROIがフレーム内に収まっていません。");
    return result;
  }

  // ★修正: 切り出したROIを clone() してメモリを確実に連続化・独立させる
  cv::Mat roiFrame = frame(roiRect).clone();

  // ★追加: チャンネル数が3でない場合（グレースケールや4ch等）はBGRに強制変換する
  if(roiFrame.channels() == 1) {
    cv::cvtColor(roiFrame, roiFrame, cv::COLOR_GRAY2BGR);
  } else if(roiFrame.channels() == 4) {
    cv::cvtColor(roiFrame, roiFrame, cv::COLOR_BGRA2BGR);
  }

  // WeChatQRCodeによる検出とデコードの実行
  // std::vector<cv::Point2f> points;
  std::vector<std::vector<cv::Point2f>> points;

  std::vector<std::string> decodedStrings;

  try {
    decodedStrings = weChatDetector->detectAndDecode(roiFrame, points);
  } catch(const cv::Exception& e) {
    Logger::printfLog(Logger::ERROR, "QrCodeDetector: 検出処理中に例外が発生しました: %s",
                      e.what());
    return result;
  }

  // QrCodeDetectionResult QrCodeDetector::detect(const cv::Mat& frame)
  // {
  //   QrCodeDetectionResult result;

  //   if(frame.empty()) {
  //     Logger::error("QrCodeDetector: 入力フレームが空です。");
  //     return result;
  //   }

  //   if(!weChatDetector) {
  //     Logger::error("QrCodeDetector: WeChatQRCodeが初期化されていません。");
  //     return result;
  //   }

  //   // ROI切り出し
  //   cv::Rect roiRect = roi & cv::Rect(0, 0, frame.cols, frame.rows);
  //   if(roiRect.empty()) {
  //     Logger::error("QrCodeDetector: ROIがフレーム内に収まっていません。");
  //     return result;
  //   }
  //   cv::Mat roiFrame = frame(roiRect);

  //   // WeChatQRCodeによる検出とデコードの実行
  //   std::vector<cv::Point2f> points;
  //   std::vector<std::string> decodedStrings;

  //   try {
  //     decodedStrings = weChatDetector->detectAndDecode(roiFrame, points);
  //   } catch (const cv::Exception& e) {
  //     Logger::printfLog(Logger::ERROR, "QrCodeDetector: 検出処理中に例外が発生しました: %s",
  //     e.what()); return result;
  //   }

  // 検出・デコード成功時
  if(!decodedStrings.empty() && !decodedStrings[0].empty()) {
    result.wasDetected = true;
    result.content = decodedStrings[0];
    result.detectedStep = "WeChatQRCode";

    // points は std::vector<std::vector<cv::Point2f>>
    // なので、1つ目のQRコード(points[0])の各頂点を取り出す
    if(!points.empty() && points[0].size() >= 4) {
      for(int i = 0; i < 4; ++i) {
        result.corners[i] = cv::Point2f(points[0][i].x + roiRect.x, points[0][i].y + roiRect.y);
      }
    }

    Logger::info("!!!!!!!復号処理成功!!!!!!");
    return result;
  }

  // 検出失敗時
  Logger::debug("QrCodeDetector: [検出失敗] ROI内にQRコードのパターンが見つかりませんでした。");
  return result;
}