/**
 * @file   MotionParserTest.cpp
 * @brief  MotionParserクラスをテストする
 * @author nishijima515
 */

#include "MotionParser.h"
#include "MockNetworkSystem.h"
#include <gtest/gtest.h>
#include <gtest/internal/gtest-port.h>

using namespace std;

namespace etrobocon2026_test {
  // ファイルパスが存在しない場合のテスト
  TEST(MotionParserTest, NotCreateMotionList)
  {
    MockNetworkSystem mockNetworkSystem;
    SocketClient mockSocketClient(mockNetworkSystem);
    Robot robot(mockSocketClient);
    string csvPath = "../tests/test_data/NonExistent.csv";

    // 標準出力のキャプチャ開始
    testing::internal::CaptureStdout();

    vector<BaseMotion*> actualList = MotionParser::createMotionList(robot, csvPath);

    // 標準出力を取得
    string output = testing::internal::GetCapturedStdout();

    // motionListが空であること
    EXPECT_TRUE(actualList.empty());

    // エラーメッセージの確認
    string expectedMessage = "コマンドファイルを開けませんでした: " + csvPath + "\n";

    EXPECT_NE(std::string::npos, output.find(expectedMessage));
  }

  // ファイルパスが存在するが、コマンドが未実装の場合のテスト
  TEST(MotionParserTest, NoErrorMessageWhenFileExistsButNoImplementation)
  {
    MockNetworkSystem mockNetworkSystem;
    SocketClient mockSocketClient(mockNetworkSystem);
    Robot robot(mockSocketClient);
    string csvPath = "../tests/test_data/Existent.csv";  // 存在するファイル

    // 標準出力キャプチャ開始
    testing::internal::CaptureStdout();

    vector<BaseMotion*> actualList = MotionParser::createMotionList(robot, csvPath);

    // 標準出力取得
    string output = testing::internal::GetCapturedStdout();

    // motionListは空（5/13現在、各動作クラスが未実装なので空になる）
    EXPECT_TRUE(actualList.empty());

    // エラーメッセージが出ていないことを確認する
    EXPECT_EQ(output.find("コマンドファイルを開けませんでした"), std::string::npos);
  }

}  // namespace etrobocon2026_test