/**
 * @file Calibrator.cpp
 * @brief キャリブレーションからスタートまでを担当するクラス
 * @author okuyama0528 sadomiya-sousi
 */

#include "Calibrator.h"

using namespace std;

Calibrator::Calibrator(Robot& _robot) : robot(_robot) {}

void Calibrator::selectAndSetCourse()
{
  robot.getDisplayInstance().showChar('C');  // C:コース
  Course course = Course::Left;
  while(1) {
    // 左ボタンが押されたときRコースがセットされていれば、Lコースをセットする
    if(robot.getButtonInstance().isLeftPressed()) {
      course = (course == Course::Left) ? Course::Right : Course::Left;
      // 画面にLコースが選択されたことを表示
      robot.getDisplayInstance().showChar(course == Course::Left ? 'L' : 'R');
      // ボタンが離されるまで待機
      while(robot.getButtonInstance().isLeftPressed()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    }
    // 右ボタンが押されたときコース選択を終了
    if(robot.getButtonInstance().isRightPressed()) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  robot.setCourse(course);

  Logger::printfLog(Logger::INFO, "Calibrator:走行開始 %s Course",
                    course == Course::Left ? "Left" : "Right");

  robot.getDisplayInstance().scrollText("OK", 50);
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}

void Calibrator::measureAndSetTargetBrightness()
{
  int blackBrightness = -1;
  int whiteBrightness = -1;

  robot.getDisplayInstance().showChar('B');
  // 黒の輝度測定
  // 左ボタンで輝度を取得し、右ボタンで黒の輝度を決定する
  bool isBrightnessMeasuring = false;
  while(1) {
    // 左ボタンが押されたら輝度値の連続取得を開始
    if(robot.getButtonInstance().isLeftPressed() && !isBrightnessMeasuring) {
      isBrightnessMeasuring = true;
      // ボタンが離されるまで待機
      while(robot.getButtonInstance().isLeftPressed()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));  // 10ミリ秒スリープ
      }
    }
    // ループが開始されたら連続的に輝度を取得
    if(isBrightnessMeasuring) {
      blackBrightness = robot.getColorSensorControllerInstance().getReflectance();
      // 現在の輝度値をリアルタイムで表示
      robot.getDisplayInstance().showNumber(blackBrightness);
      // 右ボタンが押されたら確定
      if(robot.getButtonInstance().isRightPressed()) {
        // 黒の輝度を確定したことをディスプレイに表示
        robot.getDisplayInstance().scrollText("OK", 50);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));  // 1秒スリープ
        // ボタンが離されるまで待機
        while(robot.getButtonInstance().isRightPressed()) {
          std::this_thread::sleep_for(std::chrono::milliseconds(10));  // 10ミリ秒スリープ
        }
        break;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));  // 10ミリ秒スリープ
  }
  robot.getDisplayInstance().showChar('W');
  // 白の輝度測定
  // 左ボタンで輝度を取得し、右ボタンで白の輝度を決定する
  isBrightnessMeasuring = false;
  while(1) {
    // 左ボタンが押されたら輝度値の連続取得を開始
    if(robot.getButtonInstance().isLeftPressed() && !isBrightnessMeasuring) {
      isBrightnessMeasuring = true;
      // ボタンが離されるまで待機
      while(robot.getButtonInstance().isLeftPressed()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));  // 10ミリ秒スリープ
      }
    }
    // ループが開始されたら連続的に輝度を取得
    if(isBrightnessMeasuring) {
      whiteBrightness = robot.getColorSensorControllerInstance().getReflectance();
      // 現在の輝度値をリアルタイムで表示
      robot.getDisplayInstance().showNumber(whiteBrightness);
      // 右ボタンが押されたら確定
      if(robot.getButtonInstance().isRightPressed()) {
        // 白の輝度を確定したことをディスプレイに表示
        robot.getDisplayInstance().scrollText("OK", 50);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));  // 1秒スリープ
        // ボタンが離されるまで待機
        while(robot.getButtonInstance().isRightPressed()) {
          std::this_thread::sleep_for(std::chrono::milliseconds(10));  // 10ミリ秒スリープ
        }
        break;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));  // 10ミリ秒スリープ
  }
  int target = (whiteBrightness + blackBrightness) / 2;
  robot.setTargetBrightness(target);

  // 目標輝度をディスプレイに表示
  robot.getDisplayInstance().showNumber(target);
  Logger::printfLog(Logger::INFO, "Calibrator:目標輝度値は %d ", target);
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));  // 1秒スリープ
  robot.getDisplayInstance().showChar(' ');                      // ディスプレイを消灯
}

void Calibrator::getAngleCheckFrame()
{
  robot.getDisplayInstance().showChar('F');
  // 角度調整フレーム取得
  // 中央ボタンでフレームを取得し、右ボタンで決定する
  std::string fileName = "angleCheckFrame.jpg";
  while(1) {
    // 中央ボタンが押されたらフレーム取得
    if(robot.getButtonInstance().isCenterPressed()) {
      // ボタンが離されるまで待機
      while(robot.getButtonInstance().isCenterPressed()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));  // 10ミリ秒スリープ
      }
      Snapshot snapshot(robot, fileName);
      snapshot.run();
      robot.getDisplayInstance().showChar('S');
      std::this_thread::sleep_for(std::chrono::milliseconds(10));  // 10ミリ秒スリープ
    }
    // 右ボタンで終了
    if(robot.getButtonInstance().isRightPressed()) {
      robot.getDisplayInstance().scrollText("OK", 50);
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));  // 1秒スリープ
      // ボタンが離されるまで待機
      while(robot.getButtonInstance().isRightPressed()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));  // 10ミリ秒スリープ
      }
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));  // 10ミリ秒スリープ
  }
  Logger::info("Calibrator:アングル調節完了");
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));  // 1秒スリープ
}

void Calibrator::waitForStart()
{
  Logger::info("Calibrator:待機中");
  while(!robot.getForceSensorInstance().isPressed(PRESS_POWER)) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void Calibrator::setDecryptionKey(const std::string& key)
{
  if(key.length() == 4) {
    robot.setDecryptionKey(key.c_str());
  } else {
    robot.setDecryptionKey("AAAA");
  }
}
void Calibrator::inputAndSetDecryptionKey()
{
  std::string key;

  Logger::info("復号キー読み込み中...");

  std::ifstream ifs;
  ifs.open("etrobocon2026/key.txt", std::ios::in | std::ios::binary);

  if(!ifs.is_open()) {
    Logger::error("key.txt を開けません（存在しない or パスが違う）");
  } else {
    if(!(ifs >> key)) {
      Logger::error("key.txt 読み込み失敗（中身が空 or フォーマット不正）");
    } else if(key.length() != 4) {
      Logger::error("key.txt の内容が4文字ではない");
    } else {
      Logger::info("ファイルから復号キー取得成功");
      setDecryptionKey(key);
      return;
    }
  }
  ifs.close();
}