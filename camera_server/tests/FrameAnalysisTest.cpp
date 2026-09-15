/**
 * @file    FrameAnalysisTest.cpp
 * @brief   ETロボコン実機フレーム画像の全件特徴解析および前処理・多段フォールバック検証テスト
 * @author  HaruArima08 sadomiya-sousi
 */

#include <gtest/gtest.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <vector>
#include <string>
#include <regex>
#include <algorithm>
#include "QrCodeDetector.h"
#include "Logger.h"

namespace fs = std::filesystem;

namespace etrobocon2026_test {

  // 定数定義
  static const cv::Rect FULL_FRAME_ROI(0, 0, 1920, 1080);

  // 統計集計用構造体
  struct MetricStats {
    double minVal = 1e9;
    double maxVal = -1e9;
    double sum = 0.0;
    int count = 0;

    void add(double v)
    {
      if(v < minVal) minVal = v;
      if(v > maxVal) maxVal = v;
      sum += v;
      count++;
    }

    double avg() const { return count > 0 ? (sum / count) : 0.0; }
  };

  // ファイル名からタイムスタンプを抽出
  static long long parseFrameTimestamp(const std::string& filename)
  {
    std::regex re(R"(_(\d+)\.(?:jpeg|jpg|png|JPEG|JPG|PNG)$)");
    std::smatch match;
    if(std::regex_search(filename, match, re) && match.size() > 1) {
      return std::stoll(match[1].str());
    }
    return 0;
  }

  // Laplacian分散によるピント・ボケ評価
  static double calcLaplacianVariance(const cv::Mat& src)
  {
    cv::Mat gray, lap;
    if(src.channels() == 3) {
      cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
    } else {
      gray = src;
    }
    cv::Laplacian(gray, lap, CV_64F);
    cv::Scalar mean, stddev;
    cv::meanStdDev(lap, mean, stddev);
    return stddev.val[0] * stddev.val[0];
  }

  // =========================================================================
  // 前処理パイプライン関数群
  // =========================================================================

  static cv::Mat applySharpen(const cv::Mat& src, double amount = 1.5)
  {
    cv::Mat blurred, sharp;
    cv::GaussianBlur(src, blurred, cv::Size(0, 0), 3);
    cv::addWeighted(src, 1.0 + amount, blurred, -amount, 0, sharp);
    return sharp;
  }

  static cv::Mat applyCLAHE(const cv::Mat& src, double clipLimit = 3.0,
                            const cv::Size& tileGridSize = cv::Size(8, 8))
  {
    cv::Mat result;
    if(src.channels() == 3) {
      cv::Mat lab;
      cv::cvtColor(src, lab, cv::COLOR_BGR2Lab);
      std::vector<cv::Mat> labPlanes;
      cv::split(lab, labPlanes);
      cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(clipLimit, tileGridSize);
      clahe->apply(labPlanes[0], labPlanes[0]);
      cv::merge(labPlanes, lab);
      cv::cvtColor(lab, result, cv::COLOR_Lab2BGR);
    } else {
      cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(clipLimit, tileGridSize);
      clahe->apply(src, result);
    }
    return result;
  }

  static cv::Mat applyBilateralCLAHE(const cv::Mat& src, int d = 5, double sigmaColor = 50.0,
                                     double sigmaSpace = 50.0)
  {
    cv::Mat filtered;
    cv::bilateralFilter(src, filtered, d, sigmaColor, sigmaSpace);
    return applyCLAHE(filtered);
  }

  static cv::Mat applyBilateralCLAHE_Sharpen(const cv::Mat& src, int d = 5,
                                             double sigmaColor = 50.0, double sigmaSpace = 50.0,
                                             double amount = 1.5)
  {
    cv::Mat filtered;
    cv::bilateralFilter(src, filtered, d, sigmaColor, sigmaSpace);
    cv::Mat claheImg = applyCLAHE(filtered);
    return applySharpen(claheImg, amount);
  }

  // =========================================================================
  // テスト本体
  // =========================================================================

  TEST(FrameAnalysisTest, ExecuteFullFrameAnalysisAndPreprocessingEvaluation)
  {
    Logger::init();
    const std::string targetDir = "./../camera_server/datafiles/line_trace/";
    std::vector<fs::path> filePaths;

    if(fs::exists(targetDir)) {
      for(const auto& entry : fs::directory_iterator(targetDir)) {
        if(entry.is_regular_file()) {
          const std::string ext = entry.path().extension().string();
          if(ext == ".jpeg" || ext == ".JPEG" || ext == ".jpg" || ext == ".JPG" || ext == ".png") {
            filePaths.push_back(entry.path());
          }
        }
      }
    }

    if(filePaths.empty()) {
      Logger::printfLog(Logger::WARNING, "[SKIP] 画像ディレクトリが存在しないか空です: %s",
                        targetDir.c_str());
      return;
    }

    std::sort(filePaths.begin(), filePaths.end(), [](const fs::path& a, const fs::path& b) {
      return parseFrameTimestamp(a.filename().string())
             < parseFrameTimestamp(b.filename().string());
    });

    Logger::printfLog(Logger::INFO,
                      "=================================================================");
    Logger::printfLog(Logger::INFO,
                      "  ETロボコン2026 QRコード撮影フレーム全件特徴解析 & 前処理検証");
    Logger::printfLog(Logger::INFO, "  総解析対象フレーム数: %zu", filePaths.size());
    Logger::printfLog(Logger::INFO,
                      "=================================================================");

    QrCodeDetector detector(FULL_FRAME_ROI);

    MetricStats blurStats;
    MetricStats brightnessStats;
    MetricStats contrastStats;

    int successRawFull = 0;
    int successSharpen = 0;
    int successCLAHE = 0;
    int successSharpenCLAHE = 0;
    int successBilateralCLAHE = 0;
    int successBilateralCLAHE_Sharpen = 0;

    int successFallback = 0;
    int fallbackStep1Count = 0;
    int fallbackStep2Count = 0;
    int fallbackStep3Count = 0;

    std::vector<std::string> detailedSuccessList;

    for(size_t i = 0; i < filePaths.size(); ++i) {
      const auto& path = filePaths[i];
      cv::Mat frame = cv::imread(path.string());
      if(frame.empty()) continue;

      const double blur = calcLaplacianVariance(frame);
      blurStats.add(blur);

      cv::Mat gray;
      cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
      cv::Scalar mean, stddev;
      cv::meanStdDev(gray, mean, stddev);
      brightnessStats.add(mean.val[0]);
      contrastStats.add(stddev.val[0]);

      // Raw
      const auto resRaw = detector.detect(frame);
      if(resRaw.wasDetected) {
        successRawFull++;
        detailedSuccessList.push_back("[Raw OK] " + path.filename().string() + " -> "
                                      + resRaw.content
                                      + " (Blur:" + std::to_string(static_cast<int>(blur)) + ")");
      }

      // Sharpen
      const cv::Mat imgSharp = applySharpen(frame, 1.5);
      const auto resSharp = detector.detect(imgSharp);
      if(resSharp.wasDetected) {
        successSharpen++;
        if(!resRaw.wasDetected) {
          detailedSuccessList.push_back("[Sharpen ONLY OK] " + path.filename().string() + " -> "
                                        + resSharp.content
                                        + " (Blur:" + std::to_string(static_cast<int>(blur)) + ")");
        }
      }

      // CLAHE
      const cv::Mat imgCLAHE = applyCLAHE(frame);
      const auto resCLAHE = detector.detect(imgCLAHE);
      if(resCLAHE.wasDetected) {
        successCLAHE++;
        if(!resRaw.wasDetected) {
          detailedSuccessList.push_back("[CLAHE ONLY OK] " + path.filename().string() + " -> "
                                        + resCLAHE.content
                                        + " (Blur:" + std::to_string(static_cast<int>(blur)) + ")");
        }
      }

      // Sharpen + CLAHE
      const cv::Mat imgSharpCLAHE = applySharpen(imgCLAHE, 1.5);
      const auto resSharpCLAHE = detector.detect(imgSharpCLAHE);
      if(resSharpCLAHE.wasDetected) {
        successSharpenCLAHE++;
      }

      // Bilateral + CLAHE
      const cv::Mat imgBilateralCLAHE = applyBilateralCLAHE(frame, 5, 50.0, 50.0);
      const auto resBilateralCLAHE = detector.detect(imgBilateralCLAHE);
      if(resBilateralCLAHE.wasDetected) {
        successBilateralCLAHE++;
        if(!resRaw.wasDetected && !resCLAHE.wasDetected) {
          detailedSuccessList.push_back("[Bilateral+CLAHE ONLY OK] " + path.filename().string()
                                        + " -> " + resBilateralCLAHE.content
                                        + " (Blur:" + std::to_string(static_cast<int>(blur)) + ")");
        }
      }

      // Bilateral + CLAHE + Sharpen
      const cv::Mat imgBilateralCLAHE_Sharpen
          = applyBilateralCLAHE_Sharpen(frame, 5, 50.0, 50.0, 1.5);
      const auto resBilateralCLAHE_Sharpen = detector.detect(imgBilateralCLAHE_Sharpen);
      if(resBilateralCLAHE_Sharpen.wasDetected) {
        successBilateralCLAHE_Sharpen++;
        if(!resRaw.wasDetected && !resSharpCLAHE.wasDetected && !resBilateralCLAHE.wasDetected) {
          detailedSuccessList.push_back("[Bilateral+CLAHE+Sharpen ONLY OK] "
                                        + path.filename().string() + " -> "
                                        + resBilateralCLAHE_Sharpen.content
                                        + " (Blur:" + std::to_string(static_cast<int>(blur)) + ")");
        }
      }

      // フォールバック戦略
      if(resRaw.wasDetected) {
        successFallback++;
        fallbackStep1Count++;
      } else if(resSharpCLAHE.wasDetected) {
        successFallback++;
        fallbackStep2Count++;
      } else if(resBilateralCLAHE.wasDetected) {
        successFallback++;
        fallbackStep3Count++;
      }
    }

    const size_t total = filePaths.size();
    Logger::printfLog(Logger::INFO,
                      "-----------------------------------------------------------------");
    Logger::printfLog(Logger::INFO, "【1. 全フレーム画像特徴の統計評価】");
    Logger::printfLog(
        Logger::INFO,
        "  - ピント/ボケ指標 (Laplacian Variance): 平均: %.2f | 最小: %.2f | 最大: %.2f",
        blurStats.avg(), blurStats.minVal, blurStats.maxVal);
    Logger::printfLog(Logger::INFO, "  - 平均輝度 (0-255): 平均: %.2f | 最小: %.2f | 最大: %.2f",
                      brightnessStats.avg(), brightnessStats.minVal, brightnessStats.maxVal);
    Logger::printfLog(Logger::INFO,
                      "  - コントラスト (輝度標準偏差): 平均: %.2f | 最小: %.2f | 最大: %.2f",
                      contrastStats.avg(), contrastStats.minVal, contrastStats.maxVal);
    Logger::printfLog(Logger::INFO,
                      "-----------------------------------------------------------------");
    Logger::printfLog(Logger::INFO,
                      "【2. 前処理アルゴリズム別 QRコード検出・復号成功率 (全画面ROI)】");
    Logger::printfLog(Logger::INFO, "  A. 前処理なし (Raw)                  : %d / %zu (%.1f%%)",
                      successRawFull, total, (static_cast<double>(successRawFull) / total * 100.0));
    Logger::printfLog(Logger::INFO, "  B. シャープ化                        : %d / %zu (%.1f%%)",
                      successSharpen, total, (static_cast<double>(successSharpen) / total * 100.0));
    Logger::printfLog(Logger::INFO, "  C. コントラスト補正 (CLAHE)          : %d / %zu (%.1f%%)",
                      successCLAHE, total, (static_cast<double>(successCLAHE) / total * 100.0));
    Logger::printfLog(Logger::INFO, "  D. シャープ化 + CLAHE                : %d / %zu (%.1f%%)",
                      successSharpenCLAHE, total,
                      (static_cast<double>(successSharpenCLAHE) / total * 100.0));
    Logger::printfLog(Logger::INFO, "  E. バイラテラル + CLAHE              : %d / %zu (%.1f%%)",
                      successBilateralCLAHE, total,
                      (static_cast<double>(successBilateralCLAHE) / total * 100.0));
    Logger::printfLog(Logger::INFO, "  F. 複合 (Bilateral + CLAHE + Sharpen): %d / %zu (%.1f%%)",
                      successBilateralCLAHE_Sharpen, total,
                      (static_cast<double>(successBilateralCLAHE_Sharpen) / total * 100.0));
    Logger::printfLog(Logger::INFO,
                      "-----------------------------------------------------------------");
    Logger::printfLog(Logger::INFO, "【3. 検出・復号に成功した個別フレーム詳細】");
    if(detailedSuccessList.empty()) {
      Logger::printfLog(Logger::INFO, "  (全フレームで検出失敗)");
    } else {
      for(const auto& s : detailedSuccessList) {
        Logger::printfLog(Logger::INFO, "  %s", s.c_str());
      }
    }
    Logger::printfLog(Logger::INFO,
                      "-----------------------------------------------------------------");
    Logger::printfLog(Logger::INFO, "【4. 実戦向け多段フォールバック戦略のトータル成功率】");
    Logger::printfLog(Logger::INFO, "  - トータル検出成功数: %d / %zu (%.1f%%)", successFallback,
                      total, (static_cast<double>(successFallback) / total * 100.0));
    Logger::printfLog(Logger::INFO, "      Step 1 (Raw) で解決            : %d フレーム",
                      fallbackStep1Count);
    Logger::printfLog(Logger::INFO, "      Step 2 (Sharpen + CLAHE) で解決 : %d フレーム",
                      fallbackStep2Count);
    Logger::printfLog(Logger::INFO, "      Step 3 (Bilateral + CLAHE) で解決: %d フレーム",
                      fallbackStep3Count);
    Logger::printfLog(Logger::INFO,
                      "=================================================================");

    // 必要に応じてログをファイルに出力
    Logger::outputToFile();
  }
}  // namespace etrobocon2026_test