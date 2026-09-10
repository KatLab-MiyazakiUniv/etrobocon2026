/**
 * @file   QrCodeDetector.h
 * @brief  QRコード検出処理クラス
 * @author HaruArima08
 */

#ifndef QR_CODE_DETECTOR_H
#define QR_CODE_DETECTOR_H

#include <opencv2/core.hpp>
#include <ZXing/ReadBarcode.h>
#include <ZXing/ReaderOptions.h>
#include "CodeDetector.h"
#include "Logger.h"
#include "QrCodeDetectionResult.h"
#include "SystemInfo.h"

class QrCodeDetector : public CodeDetector<QrCodeDetectionResult> {
 public:
  /**
   * @brief コンストラクタ
   * @param roi 注目領域
   */
  explicit QrCodeDetector(const cv::Rect& roi);

  /**
   * @brief デストラクタ
   */
  ~QrCodeDetector();

  /**
   * @brief フレーム内のQRコードを検出・デコードする
   * @param frame 処理対象のフレーム
   * @return 検出結果
   */
  QrCodeDetectionResult detect(const cv::Mat& frame) override;

  /**
   * @brief ROIを検証したうえで設定する
   * @param _roi 設定するROI
   */
  void setValidatedRoi(const cv::Rect& _roi);

  /**
   * @brief アンシャープマスクによるシャープ化
   * @param src 入力画像
   * @param amount シャープ化強度
   * @return 処理後画像
   */
  static cv::Mat applySharpen(const cv::Mat& src, double amount = 1.5);

  /**
   * @brief コントラスト制限付き適応的ヒストグラム均等化 (CLAHE)
   * @param src 入力画像
   * @param clipLimit コントラスト制限値
   * @param tileGridSize グリッドサイズ
   * @return 処理後画像
   */
  static cv::Mat applyCLAHE(const cv::Mat& src, double clipLimit = 3.0,
                            const cv::Size& tileGridSize = cv::Size(8, 8));

  /**
   * @brief バイラテラルフィルタによるエッジ保持平滑化
   * @param src 入力画像
   * @param d ピクセル近傍の直径
   * @param sigmaColor 色空間のシグマ値
   * @param sigmaSpace 座標空間のシグマ値
   * @return 処理後画像
   */
  static cv::Mat applyBilateral(const cv::Mat& src, int d = 5, double sigmaColor = 50.0,
                                double sigmaSpace = 50.0);

 private:
  ZXing::ReaderOptions options;  // ZXingのデコードオプション
  cv::Rect roi;                  // フレーム全体に対するROI

  /**
   * @brief ROIがフレーム内に収まるように補正する
   */
  void validateParameters();

  /**
   * @brief ZXingのデコード結果からQrCodeDetectionResultを生成する
   * @param barcode デコード結果
   * @param roiRect ROI矩形
   * @param stepName 検出に成功した前処理ステップ名
   * @return 生成されたQrCodeDetectionResult
   */
  QrCodeDetectionResult createResult(const ZXing::Barcode& barcode, const cv::Rect& roiRect,
                                     const std::string& stepName = "");
};

#endif  // QR_CODE_DETECTOR_H
