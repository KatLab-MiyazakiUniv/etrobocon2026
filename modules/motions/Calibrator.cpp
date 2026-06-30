/**
 * @file Calibrator.cpp
 * @brief キャリブレーションからスタートまでを担当するクラス
 * @author okuyama0528 sadomiya-sousi
 */

#include "Calibrator.h"

using namespace std;

Calibrator::Calibrator(Robot& _robot, std::unique_ptr<BaseContinuationCondition> condition)
  : BaseMotion(_robot, std::move(condition))
{
  LOG_CREATE("Calibrator");
}
Calibrator::~Calibrator()
{
  LOG_DESTROY("Calibrator");
}
void Calibrator::executeStep()
{
  inputAndSetDecryptionKey();
  selectAndSetCourse();
  measureAndSetTargetBrightness();
  getAngleCheckFrame();
  waitForStart();
}

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
        ClockUtil::sleep();
      }
    }
    // 右ボタンが押されたときコース選択を終了
    if(robot.getButtonInstance().isRightPressed()) {
      break;
    }
    ClockUtil::sleep();
  }
  robot.setCourse(course);

  Logger::printfLog(Logger::INFO, "Calibrator:走行開始 %s Course",
                    course == Course::Left ? "Left" : "Right");

  robot.getDisplayInstance().scrollText("OK", 50);
  ClockUtil::sleep();
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
        ClockUtil::sleep();
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
        ClockUtil::sleep(1000);
        // ボタンが離されるまで待機
        while(robot.getButtonInstance().isRightPressed()) {
          ClockUtil::sleep();
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
        ClockUtil::sleep();
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
        ClockUtil::sleep();
        // ボタンが離されるまで待機
        while(robot.getButtonInstance().isRightPressed()) {
          ClockUtil::sleep();
        }
        break;
      }
    }
    ClockUtil::sleep();
  }
  int target = (whiteBrightness + blackBrightness) / 2;
  robot.setTargetBrightness(target);

  // 目標輝度をディスプレイに表示
  robot.getDisplayInstance().showNumber(target);
  Logger::printfLog(Logger::INFO, "Calibrator:目標輝度値は %d ", target);
  ClockUtil::sleep(1000);
  robot.getDisplayInstance().showChar(' ');  // ディスプレイを消灯
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
        ClockUtil::sleep();
      }
      Snapshot snapshot(robot, fileName, std::make_unique<RepeatCountCondition>(robot, 1));
      snapshot.run();
      robot.getDisplayInstance().showChar('S');
      ClockUtil::sleep();
    }
    // 右ボタンで終了
    if(robot.getButtonInstance().isRightPressed()) {
      robot.getDisplayInstance().scrollText("OK", 50);
      ClockUtil::sleep(1000);  // ボタンが離されるまで待機
      while(robot.getButtonInstance().isRightPressed()) {
        ClockUtil::sleep();
      }
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));  // 10ミリ秒スリープ
  }
  Logger::info("Calibrator:アングル調節完了");
  ClockUtil::sleep(1000);
}

void Calibrator::waitForStart()
{
  Logger::info("Calibrator:待機中");
  while(!robot.getForceSensorInstance().isPressed(PRESS_POWER)) {
    ClockUtil::sleep();
  }
}

void Calibrator::inputAndSetDecryptionKey()
{
  robot.getDisplayInstance().showChar('K');  // K: Key

  std::string key1;
  std::string key2;

  Logger::info("復号キー読み込み中...");

  std::ifstream ifs("etrobocon2026/key.txt", std::ios::in | std::ios::binary);

  if(!ifs.is_open()) {
    Logger::error("key.txt を開けません");
    robot.getDisplayInstance().scrollText("ERR", 50);
    ClockUtil::sleep(1000);
    return;
  }

  if(!(ifs >> key1)) {
    Logger::error("1行目の復号キー読み込み失敗");
    robot.getDisplayInstance().scrollText("ERR", 50);
    ClockUtil::sleep(1000);
    return;
  }

  if(!(ifs >> key2)) {
    Logger::error("2行目の復号キー読み込み失敗");
    robot.getDisplayInstance().scrollText("ERR", 50);
    ClockUtil::sleep(1000);
    return;
  }

  if(key1.length() != 4 || key2.length() != 4) {
    Logger::error("復号キーは4文字で入力してください");
    robot.getDisplayInstance().scrollText("ERR", 50);
    ClockUtil::sleep(1000);
    return;
  }

  if(key1 != key2) {
    Logger::error("復号キーが一致しません");
    robot.getDisplayInstance().scrollText("ERR", 50);
    ClockUtil::sleep(1000);
    return;
  }

  Logger::printfLog(Logger::INFO, "復号キー: %s", key1.c_str());

  robot.setDecryptionKey(key1.c_str());

  robot.getDisplayInstance().scrollText("OK", 50);
  ClockUtil::sleep(1000);

  ifs.close();
}
