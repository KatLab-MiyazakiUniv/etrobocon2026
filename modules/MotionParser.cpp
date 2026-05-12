/**
 * @file   MotionParser.cpp
 * @brief  動作コマンドファイルを解析するクラス
 * @author nishijima515
 */

#include "MotionParser.h"

#include <unordered_map>

using namespace std;

vector<BaseMotion*> MotionParser::createMotionList(Robot& robot, string& commandFilePath)
{
  // 行番号カウンタ
  int lineNum = 1;
  // 動作インスタンスのリスト
  vector<BaseMotion*> motionList;

  // ファイルを開き、開けなければ空のリストを返す
  ifstream file(commandFilePath);
  if(!file) {
    cout << "コマンドファイルを開けませんでした: " << commandFilePath << endl;
    return motionList;
  }

  // 各行を格納する変数を定義
  string line;

  // fileから1行ずつ文字列として line に読み込む
  // TODO: 各動作クラスが完成したら、以下のコメントを外してswitch-caseを実装する
  while(getline(file, line)) {
    // 文字列 line をストリームに変換
    stringstream ss(line);

    // カンマ区切りでコマンド名とその引数を1つずつ取り出して params に追加
    vector<string> params;
    for(string token; getline(ss, token, SEPARATOR);) {
      params.push_back(move(token));
    }

    // TODO: コマンド名をCOMMAND enumに変換して各Motionオブジェクトを生成する処理
    //       各コマンドに対応するMotionクラスが実装済みになったら順次追加していく
    // COMMAND command = convertCommand(params[0]);
    // switch(command) {
    //   case COMMAND::AR: {
    //     // AR: 角度指定回頭
    //     // params[1]:int 角度[deg], params[2]:double 速度[mm/s], params[3]:string 方向
    //     auto ar = new AngleRotation(robot, std::stoi(params[1]), std::stod(params[2]),
    //                                 convertBool(params[0], params[3]));
    //     motionList.push_back(ar);
    //     break;
    //   }
    //   // ↓ 他のコマンドはここに追加していく
    //   default: {
    //     cout << commandFilePath << ":" << lineNum << " Command " << params[0] << " は未定義です"
    //     << endl; break;
    //   }
    // }

    lineNum++;  // 行番号をインクリメントする
  }

  return motionList;
}

COMMAND MotionParser::convertCommand(const string& str)
{
  // コマンド文字列(string)と、それに対応する列挙型COMMANDのマッピングを定義
  static const unordered_map<string, COMMAND> commandMap = {
    { "EXAMPLE", COMMAND::EXAMPLE },  // 例
  };

  // コマンド文字列に対応するCOMMAND値をマップから取得。なければCOMMAND::NONEを返す
  auto it = commandMap.find(str);
  if(it != commandMap.end()) {
    return it->second;
  } else {
    return COMMAND::NONE;
  }
}

// bool MotionParser::convertBool(const string& command, const string& stringParameter)
// {
//   // 末尾の改行を削除
//   string param = StringOperator::removeEOL(stringParameter);

//   // カメラPIDトラッキング系の停止制御（continueなら継続、stopなら停止）
//   if(command == "DCL" || command == "CDCL" || command == "UDCL" || command == "DTCCL"
//      || command == "CDTCCL") {
//     if(param == "continue") {
//       return false;
//     } else if(param == "stop") {
//       return true;
//     } else {
//       cout << "'continue' か 'stop'を入力してください" << endl;
//       return true;
//     }
//   }

//   //
//   回転動作(AR,IMUR,MCA,BCA)の場合、"clockwise"ならtrue（時計回り）、"anticlockwise"ならfalse（反時計回り）に変換
//   if(command == "AR" || command == "IMUR" || command == "MCA" || command == "BCA"
//      || command == "CRA") {
//     if(param == "clockwise") {
//       return true;
//     } else if(param == "anticlockwise") {
//       return false;
//     } else {
//       cout << "'clockwise' か 'anticlockwise'を入力してください" << endl;
//       return true;
//     }
//   }

//   // エッジ切り替え(EC)の場合、"left"ならtrue（左エッジ）、"right"ならfalse（右エッジ)に変換
//   if(command == "EC") {
//     if(param == "left") {
//       return true;
//     } else if(param == "right") {
//       return false;
//     } else {
//       cout << "'left' か 'right'を入力してください" << endl;
//       return true;
//     }
//   }

//   // IMU設定(IS)の場合、"start"ならtrue（開始）、"stop"ならfalse（停止)に変換
//   if(command == "IS") {
//     if(param == "start") {
//       return true;
//     } else if(param == "stop") {
//       return false;
//     } else {
//       cout << "'start' か 'stop'を入力してください" << endl;
//       return false;
//     }
//   }

//   // ここまでに条件を満たしていなかった場合は、デフォルト値としてtrueを返す
//   cout << "convertBool関数の処理の対象外です: '" << command << endl;
//   return true;
// }

// bool MotionParser::convertRotationModeToBool(const string& stringParameter)
// {
//   // 末尾の改行を削除
//   string param = StringOperator::removeEOL(stringParameter);

//   // "relative"ならfalse（相対角度回頭）、"absolute"ならtrue（絶対角度回頭）に変換
//   if(param == "relative") {
//     return false;
//   } else if(param == "absolute") {
//     return true;
//   } else {
//     cout << "'relative' か 'absolute'を入力してください (入力値: " << param << ")" << endl;
//     return false;  // デフォルトは相対角度回頭
//   }
// }