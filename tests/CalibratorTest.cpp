/**
 * @file CalibratorTest.cpp
 * @brief Calibratorクラスのテスト
 * @author okuyama0528
 */

#include "Calibrator.h"
#include <gtest/gtest.h>
#include <iostream>
#include <cstdio>

using namespace std;

namespace etrobocon2025_test {
  // waitForStart()において期待した出力がされており，WarningやErrorが出ていないかテスト
  TEST(CalibratorTest, WaitForStart)
  {
    RealNetworkSystem netSys;
    SocketClient socketClient(netSys);
    Robot robot(socketClient);
    Calibrator calibrator(robot);
    testing::internal::CaptureStdout();  // 標準出力キャプチャ開始
    calibrator.waitForStart();
    string output = testing::internal::GetCapturedStdout();  // キャプチャ終了
    // find("str")はstrが見つからない場合string::nposを返す
    bool actual = output.find("待機中\n") != string::npos;
    EXPECT_TRUE(actual);
  }

  // 左右ボタンでLRコースを選択できるかのテスト
  TEST(CalibratorTest, SetCourse)
  {
    RealNetworkSystem netSys;
    SocketClient socketClient(netSys);
    Robot robot(socketClient);
    Calibrator calibrator(robot);

    testing::internal::CaptureStdout();  // 標準出力キャプチャ開始
    calibrator.selectAndSetCourse();
    string output = testing::internal::GetCapturedStdout();  // キャプチャ終了

    Course actual = robot.getCourse();
    // Leftコースと出力されていた場合

    if(output.find("Calibrator:走行開始 Left Course") != string::npos) {
      EXPECT_EQ(Course::Left, actual);
      // Rightコースと出力されていた場合
    } else if(output.find("Calibrator:走行開始 Right Course") != string::npos) {
      EXPECT_EQ(Course::Right, actual);  // 出力とゲッタの値が等しいかテスト
    } else {
      FAIL();  // 想定外
    }
  }

  // 目標輝度値を取得できるかのテスト
  TEST(CalibratorTest, TargetBrightness)
  {
    RealNetworkSystem netSys;
    SocketClient socketClient(netSys);
    Robot robot(socketClient);
    Calibrator calibrator(robot);

    testing::internal::CaptureStdout();  // 標準出力キャプチャ開始
    calibrator.measureAndSetTargetBrightness();
    string output = testing::internal::GetCapturedStdout();  // キャプチャ終了

    string targetString = "Calibrator:目標輝度値は ";  // 目標輝度値の直前に書かれている文字列

    // 出力された目標輝度値を取得
    int index = output.find(targetString) + targetString.length();
    string expectedStr = output.substr(index);  // 輝度値を取得（文字列）
    int expected = stoi(expectedStr);           // 文字列を整数値に変換

    int actual = robot.getTargetBrightness();  // 実際の輝度値を取得

    EXPECT_EQ(expected, actual);  // 出力とゲッタの値が等しいかテスト
  }
  TEST(CalibratorTest, DecryptionKeyFromFile)
  {
    // --- ディレクトリ作成 ---
    std::filesystem::create_directory("etrobocon2026");

    // --- ファイル作成 ---
    std::ofstream ofs("etrobocon2026/key.txt");
    ofs << "1234";
    ofs.close();

    RealNetworkSystem netSys;
    SocketClient socketClient(netSys);
    Robot robot(socketClient);
    Calibrator calibrator(robot);

    calibrator.inputAndSetDecryptionKey();

    std::string actual = robot.getDecryptionKey();
    EXPECT_EQ("1234", actual);

    // --- 後片付け ---
    std::remove("etrobocon2026/key.txt");
    std::filesystem::remove("etrobocon2026");
  }
}  // namespace etrobocon2025_test