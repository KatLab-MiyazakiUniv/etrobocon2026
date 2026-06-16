
/**
 * @file Calibrator.cpp
 * @brief キャリブレーションからスタートまでを担当するクラス
 * @author okuyama0528 sadomiya-sousi
 */

#include "Calibrator.h"
#include <thread>
#include <chrono>
#include <iostream>

using namespace std;

Calibrator::Calibrator(Robot& _robot)
  : robot(_robot), isLeftCourse(true), targetBrightness(50), fourDigitNumber(0)
{
}

void Calibrator::selectAndSetCourse()
{
  robot.getDisplayInstance().showChar('C');  // C:コース
  while(1) {
    // 左ボタンが押されたときRコースがセットされていれば、Lコースをセットする
    if(robot.getButtonInstance().isLeftPressed() && !isLeftCourse) {
      isLeftCourse = true;
      // 画面にLコースが選択されたことを表示
      robot.getDisplayInstance().showChar('L');
      std::this_thread::sleep_for(std::chrono::milliseconds(10));  // 10ミリ秒スリープ
      // ボタンが離されるまで待機
      while(robot.getButtonInstance().isLeftPressed()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));  // 10ミリ秒スリープ
      }
    }
    // 左ボタンが押されたときLコースがセットされていれば、Rコースをセットする
    else if(robot.getButtonInstance().isLeftPressed() && isLeftCourse) {
      isLeftCourse = false;
      // 画面にRコースが選択されたことを表示
      robot.getDisplayInstance().showChar('R');
      std::this_thread::sleep_for(std::chrono::milliseconds(10));  // 10ミリ秒スリープ
      // ボタンが離されるまで待機
      while(robot.getButtonInstance().isLeftPressed()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));  // 10ミリ秒スリープ
      }
    }
    // 右ボタンが押されたときコース選択を終了
    if(robot.getButtonInstance().isRightPressed()) {
      break;
    }
  }
  const char* course = isLeftCourse ? "Left" : "Right";
  printf("\nWill Run on the %s Course\n", course);
  robot.getDisplayInstance().scrollText("OK", 50);
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));  // 1秒スリープ
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

  targetBrightness = (whiteBrightness + blackBrightness) / 2;
  // 目標輝度をディスプレイに表示
  robot.getDisplayInstance().showNumber(targetBrightness);
  cout << "Target Brightness Value is " << targetBrightness << endl;
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
  cout << "Complete Check Frame." << endl;
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));  // 1秒スリープ
}

void Calibrator::waitForStart()
{
  Logger::info("待機中");
  // ForceSensorが押されるまで待機
  while(!robot.getForceSensorInstance().isPressed(PRESS_POWER)) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));  // 10ミリ秒スリープ
  }
}

// void Calibrator::inputAndSetFourDigitNumber()
// {
//   robot.getDisplayInstance().showChar('P');
//   std::cout << "decryption key (0000-9999): " << endl;
//   int inputVal;
//   while(1) {
//     if(std::cin >> inputVal) {
//       if(inputVal >= 0 && inputVal <= 9999) {
//         fourDigitNumber = inputVal;
//         break;
//       }
//     } else {
//       if(std::cin.eof()) {
//         fourDigitNumber = 0;  // デフォルト値
//         break;
//       }
//       std::cin.clear();
//       std::cin.ignore(numeric_limits<streamsize>::max(), '\n');
//     }
//     cout << "Invalid input. Please enter a 4-digit number (0000-9999): " << endl;
//   }
//   cout << "Input 4-digit number: " << fourDigitNumber << endl;
// }

void Calibrator::inputAndSetFourDigitNumber()
{
  int digit[4] = { 0, 0, 0, 0 };

  for(int i = 0; i < 4; ++i) {
    // ディスプレイに現在の桁のインデックス（1〜4）を表示する
    robot.getDisplayInstance().showChar('1' + i);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));  // 0.5秒表示

    while(1) {
      // 現在の入力値を表示する
      robot.getDisplayInstance().showNumber(digit[i]);

      // 左ボタンが押されたとき、値をデクリメントする
      if(robot.getButtonInstance().isLeftPressed()) {
        digit[i] = (digit[i] + 9) % 10;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));  // チャタリング防止のスリープ
        // ボタンが離されるまで待機
        while(robot.getButtonInstance().isLeftPressed()) {
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
      }

      // 中央ボタンが押されたとき、値をインクリメントする
      if(robot.getButtonInstance().isCenterPressed()) {
        digit[i] = (digit[i] + 1) % 10;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));  // チャタリング防止のスリープ
        // ボタンが離されるまで待機
        while(robot.getButtonInstance().isCenterPressed()) {
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
      }

      // 右ボタンが押されたとき、その桁を確定して次の桁へ
      if(robot.getButtonInstance().isRightPressed()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));  // チャタリング防止のスリープ
        while(robot.getButtonInstance().isRightPressed()) {
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  // 4桁の数値を計算
  fourDigitNumber = digit[0] * 1000 + digit[1] * 100 + digit[2] * 10 + digit[3];

  // 確定した数値を文字列にしてスクロール表示
  char displayStr[16];
  sprintf(displayStr, "%04d", fourDigitNumber);
  robot.getDisplayInstance().scrollText(displayStr, 100);

  cout << "Input 4-digit number: " << fourDigitNumber << endl;
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  robot.getDisplayInstance().showChar(' ');  // ディスプレイを消灯
}

bool Calibrator::getIsLeftCourse()
{
  return isLeftCourse;
}

int Calibrator::getTargetBrightness()
{
  return targetBrightness;
}

int Calibrator::getFourDigitNumber()
{
  return fourDigitNumber;
}