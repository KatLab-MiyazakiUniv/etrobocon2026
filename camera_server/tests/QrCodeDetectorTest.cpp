/**
 * @file   QrCodeDetectorTest.cpp
 * @brief  QRコード検出用の画像処理クラスのテスト
 * @author yutaro-1214
 */

#include <gtest/gtest.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include "QrCodeDetector.h"

#include <opencv2/opencv.hpp>
#include <filesystem>
#include <vector>
#include <string>
#include <regex>
#include <algorithm>
#include "QrCodeDetector.h"

namespace fs = std::filesystem;

// ファイル名末尾のタイムスタンプ（数値）を抽出するヘルパー関数
long long extractTimestamp(const std::string& filename)
{
  // 例: "..._2494623.JPEG" から 2494623 を抽出
  std::regex re(R"(_(\d+)\.(?:jpeg|jpg|png|JPEG|JPG|PNG)$)");
  std::smatch match;
  if(std::regex_search(filename, match, re) && match.size() > 1) {
    return std::stoll(match[1].str());
  }
  return 0;
}

namespace etrobocon2026_test {

  // Hint1.pngの実解像度(135x135)を包含する、フレーム全体を対象とするROI
  static const cv::Rect FULL_FRAME_ROI(0, 0, 1920, 1080);

  TEST(QrCodeDetectorTest, BatchDetectionInTimestampOrder)
  {
    const std::string targetDir = "./../camera_server/datafiles/line_trace/";
    cv::Rect roi(0, 0, 1920, 1080);
    // QrCodeDetector::QrCodeDetector qrCodeDetector(roi);
    QrCodeDetector qrCodeDetector(roi);

    // 1. ディレクトリ内の画像ファイルを収集
    std::vector<fs::path> filePaths;
    for(const auto& entry : fs::directory_iterator(targetDir)) {
      if(entry.is_regular_file()) {
        std::string ext = entry.path().extension().string();
        // 大文字小文字を問わず画像拡張子を判定
        if(ext == ".jpeg" || ext == ".JPEG" || ext == ".jpg" || ext == ".JPG" || ext == ".png") {
          filePaths.push_back(entry.path());
        }
      }
    }

    ASSERT_FALSE(filePaths.empty())
        << "対象ディレクトリに画像ファイルが存在しません: " << targetDir;

    // 2. タイムスタンプ順（時系列昇順）にソート
    std::sort(filePaths.begin(), filePaths.end(), [](const fs::path& a, const fs::path& b) {
      return extractTimestamp(a.filename().string()) < extractTimestamp(b.filename().string());
    });

    int detectedCount = 0;
    int totalCount = static_cast<int>(filePaths.size());

    // 3. 時系列順に連続実行
    for(size_t i = 0; i < filePaths.size(); ++i) {
      const auto& path = filePaths[i];
      cv::Mat frame = cv::imread(path.string());

      if(frame.empty()) {
        std::cout << "[SKIP] 読み込み失敗: " << path.filename().string() << std::endl;
        continue;
      }

      auto result = qrCodeDetector.detect(frame);

      if(result.wasDetected) {
        detectedCount++;
        std::cout << "[SUCCESS] " << path.filename().string() << " -> Content: " << result.content
                  << std::endl;
      } else {
        std::cout << "[FAILED ] " << path.filename().string() << std::endl;
      }
    }

    std::cout << "===== 検出結果サマリー =====" << std::endl;
    std::cout << "検出成功率: " << detectedCount << " / " << totalCount << " ("
              << (static_cast<double>(detectedCount) / totalCount * 100.0) << " %)" << std::endl;

    // 少なくとも1フレーム以上は検出できていることを検証
    EXPECT_GT(detectedCount, 0) << "1度もQRコードを検出・デコードできませんでした。";
  }

  // QRコードを含む画像を入力したときに、QRコードを検出し、
  // 内容「25,35」を正しく取得できることを確認するテスト
  TEST(QrCodeDetectorTest, DetectQrCode)
  {
    cv::Mat frame = cv::imread("camera_server/tests/test_data/Hint1.png");
    // cv::Mat frame
    //     = cv::imread("./../camera_server/datafiles/line_trace/"
    //                  "det_d0_tlx0_tly0_trx0_try0_blx0_bly0_brx0_bry0_rx0_ry0_rw0_rh0_2494623.JPEG");
    ASSERT_FALSE(frame.empty());

    QrCodeDetector detector(FULL_FRAME_ROI);
    int beforeTime = ClockUtil::now();
    auto result = detector.detect(frame);
    int afterTime = ClockUtil::now();
    Logger::printfLog(Logger::DEBUG, "検出処理にかかった時間は%d", afterTime - beforeTime);
    EXPECT_TRUE(result.wasDetected);
    // EXPECT_EQ("25,35", result.content);
  }

  // 空画像を入力したときに、QRコード未検出となり、
  // wasDetectedがfalse、contentが空文字列になることを確認するテスト
  TEST(QrCodeDetectorTest, EmptyFrame)
  {
    QrCodeDetector detector(FULL_FRAME_ROI);

    cv::Mat frame;

    QrCodeDetectionResult result = detector.detect(frame);

    EXPECT_FALSE(result.wasDetected);
    EXPECT_TRUE(result.content.empty());
  }

  // フレームは空でないが、QRコードを含まない画像を入力したときに、QRコード未検出となり、
  // wasDetectedがfalse、contentが空文字列になることを確認するテスト
  TEST(QrCodeDetectorTest, NoQrCode)
  {
    QrCodeDetector detector(FULL_FRAME_ROI);

    cv::Mat frame = cv::Mat::zeros(300, 300, CV_8UC3);

    QrCodeDetectionResult result = detector.detect(frame);

    EXPECT_FALSE(result.wasDetected);
    EXPECT_TRUE(result.content.empty());
  }

  // ROIがQRコードを含まない領域に限定されているとき、QRコード未検出となることを確認するテスト
  TEST(QrCodeDetectorTest, NotDetectedWhenRoiExcludesQrCode)
  {
    cv::Mat frame = cv::imread("camera_server/test_data/Hint1.png");
    ASSERT_FALSE(frame.empty());

    // フレーム左上の小さな領域にはQRコードが含まれない
    QrCodeDetector detector(cv::Rect(0, 0, 10, 10));

    auto result = detector.detect(frame);

    EXPECT_FALSE(result.wasDetected);
  }

  // ROIがフレーム範囲外を指定しているとき、QRコード未検出となることを確認するテスト
  TEST(QrCodeDetectorTest, NotDetectedWhenRoiOutsideFrame)
  {
    cv::Mat frame = cv::imread("camera_server/test_data/Hint1.png");
    ASSERT_FALSE(frame.empty());

    QrCodeDetector detector(cv::Rect(CAM_MAX_WIDTH + 1, CAM_MAX_HEIGHT + 1, 100, 100));

    auto result = detector.detect(frame);

    EXPECT_FALSE(result.wasDetected);
  }

  // setValidatedRoiでROIを更新すると、更新後のROIを用いて検出できることを確認するテスト
  TEST(QrCodeDetectorTest, DetectQrCodeAfterSetRoi)
  {
    cv::Mat frame = cv::imread("camera_server/test_data/Hint1.png");
    ASSERT_FALSE(frame.empty());

    // 最初はQRコードを含まない領域を指定
    QrCodeDetector detector(cv::Rect(0, 0, 10, 10));
    ASSERT_FALSE(detector.detect(frame).wasDetected);

    // フレーム全体を対象とするROIに更新
    detector.setValidatedRoi(FULL_FRAME_ROI);

    auto result = detector.detect(frame);
    EXPECT_TRUE(result.wasDetected);
    EXPECT_EQ("25,35", result.content);
  }
}  // namespace etrobocon2026_test