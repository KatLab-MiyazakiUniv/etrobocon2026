/**
 * @file   MotionParser.cpp
 * @brief  動作コマンドファイルを解析するクラス
 * @author nishijima515
 */

#include "MotionParser.h"

using namespace std;

void MotionParser::createRunCSV(Robot& robot, Area area, bool isLeftCourse)
{
    cout << "実行用コマンドファイルの作成を開始します" << endl;

    string areaPath = "etrobocon2026/datafiles/commands/Area/";
    string motionsPath = "etrobocon2026/datafiles/commands/Motions/";
    string runPath = "etrobocon2026/datafiles/commands/Run/";

    string areaName = areaCommandNames[static_cast<int>(area)];
    string course = isLeftCourse ? "Left" : "Right";

    string commandAreaFilePath = areaPath + areaName + course + ".csv";
    string commandRunFilePath = runPath + "run_" + areaName + course + ".csv";

    // 入力ファイル
    ifstream commandAreaFile(commandAreaFilePath);
    if (!commandAreaFile) {
        cerr << "エリアコマンドファイルを開けませんでした: " << commandAreaFilePath << endl;
        return;
    }

    // 出力ファイル（追記モード）
    ofstream commandRunFile(commandRunFilePath, ios::app);
    if (!commandRunFile) {
        cerr << "実行用コマンドファイルを開けませんでした: " << commandRunFilePath << endl;
        return;
    }

    string areaFileLine;

    while (getline(commandAreaFile, areaFileLine)) {

        // 空行スキップ
        if (areaFileLine.empty()) continue;

        // CSV分割
        stringstream ssArea(areaFileLine);
        vector<string> areaFileParams;
        string token;

        while (getline(ssArea, token, SEPARATOR)) {
            areaFileParams.push_back(token);
        }

        // 列数チェック
        if (areaFileParams.size() != 2) {
            cerr << "エリアコマンドファイルのフォーマットが不正です: " << areaFileLine << endl;
            continue;
        }

        string commandName = areaFileParams[0];
        string commandID   = areaFileParams[1];

        string commandMotionsFilePath = motionsPath + commandName + ".csv";

        // 中間ファイル(Motionごとのコマンドファイル)の読み込み
        ifstream commandMotionsFile(commandMotionsFilePath);
        if (!commandMotionsFile) {
            cerr << "動作コマンドファイルを開けませんでした: " 
                 << commandMotionsFilePath << endl;
            continue;
        }

        string motionsFileLine;

        while (getline(commandMotionsFile, motionsFileLine)) {

            if (motionsFileLine.empty()) continue;

            // CSV分割
            stringstream ssMotions(motionsFileLine);
            vector<string> motionParams;

            while (getline(ssMotions, token, SEPARATOR)) {
                motionParams.push_back(token);
            }

            // ID一致チェック（2列目がID想定）
            if (!motionParams.empty() && motionParams[1] == commandID) {
                commandRunFile << motionsFileLine << "\n";
                break;
            }
        }
    }

    cout << "実行用コマンドファイルの作成が完了しました" << endl;
}



// vector<Motion*> MotionParser::createMotions(Robot& robot, string& commandFilePath,
//                                             int targetBrightness)
// {
//   // 行番号カウンタ
//   int lineNum = 1;
//   // 動作インスタンスのリスト
//   vector<Motion*> motionList;

//   // ファイルを開き、開けなければ空のリストを返す
//   ifstream file(commandFilePath);
//   if(!file) {
//     cout << "コマンドファイルを開けませんでした: " << commandFilePath << endl;
//     return motionList;
//   }

//   // 各行を格納する変数と、区切り文字としてカンマを定義
//   string line;
// //   constexpr char separator = ',';

//   // fileから1行ずつ文字列として line に読み込む
//   while(getline(file, line)) {
//     // 文字列 line をストリームに変換
//     stringstream ss(line);

//     // カンマ区切りでコマンド名とその引数を1つずつ取り出して params に追加
//     vector<string> params;
//     for(string token; getline(ss, token, SEPARATOR);) {
//       params.push_back(move(token));
//     }

//     // コマンド名(paramsの0番目)を対応する動作コマンドに変換
//     COMMAND command = convertCommand(params[0]);

//     // コマンドに応じて対応する動作オブジェクトを生成し、動作リスト（motionList）に追加する処理
//     switch(command) {
//       // AR: 角度指定回頭
//       // [1]:int 角度[deg], [2]:double 速度[mm/s], [3]:string 方向(clockwise or anticlockwise)
//       case COMMAND::AR: {
//         auto ar = new AngleRotation(robot, stoi(params[1]), stod(params[2]),
//                                     convertBool(params[0], params[3]));
//         motionList.push_back(ar);
//         break;
//       }

//       // 未定義コマンド
//       default: {
//         cout << commandFilePath << ":" << lineNum << " Command " << params[0] << " は未定義です"
//              << endl;
//         break;
//       }
//     }

//     lineNum++;  // 行番号をインクリメントする
//   }

//   return motionList;
// }

// COMMAND MotionParser::convertCommand(const string& str)
// {
//   // コマンド文字列(string)と、それに対応する列挙型COMMANDのマッピングを定義
//   static const unordered_map<string, COMMAND> commandMap = {
//     { "AR", COMMAND::AR },         // 角度指定回頭
//     { "IMUR", COMMAND::IMUR },     // IMU角度指定回頭
//     { "IMUMR", COMMAND::IMUMR },   // IMU絶対角度を最小の角度で回頭動作
//     { "DS", COMMAND::DS },         // 指定距離直進
//     { "IDS", COMMAND::IDS },       // IMU角度補正直進
//     { "CS", COMMAND::CS },         // 指定色直進
//     { "DL", COMMAND::DL },         // 指定距離ライントレース
//     { "DCL", COMMAND::DCL },       // 指定距離カメラライントレース
//     { "CDCL", COMMAND::CDCL },     // 色距離指定カメラライントレース
//     { "UDCL", COMMAND::UDCL },     // 超音波距離指定カメラライントレース
//     { "CL", COMMAND::CL },         // 指定色ライントレース
//     { "CDL", COMMAND::CDL },       // 色距離指定ライントレース
//     { "EC", COMMAND::EC },         // エッジ切り替え
//     { "SL", COMMAND::SL },         // スリープ
//     { "SS", COMMAND::SS },         // カメラ撮影動作
//     { "MCA", COMMAND::MCA },       // ミニフィグのカメラ撮影動作
//     { "BCA", COMMAND::BCA },       // 風景・プラレールのカメラ撮影動作
//     { "CRA", COMMAND::CRA },       // カメラ復帰動作
//     { "BTCA", COMMAND::BTCA },     // ボトル2つ目のキャッチ動作
//     { "BLA", COMMAND::BLA },       // ボトルランディング動作
//     { "STOP", COMMAND::STOP },     // 走行体を停止させる動作
//     { "PCIDS", COMMAND::PCIDS },   // 画像ラインを用いた距離直進
//     { "IS", COMMAND::IS },         // IMU設定
//     { "DTCCL", COMMAND::DTCCL },   // 2色指定距離カメラライントレース
//     { "CDTCCL", COMMAND::CDTCCL }  // 2色色指定距離カメラライントレース
//   };

//   // コマンド文字列に対応するCOMMAND値をマップから取得。なければCOMMAND::NONEを返す
//   auto it = commandMap.find(str);
//   if(it != commandMap.end()) {
//     return it->second;
//   } else {
//     return COMMAND::NONE;
//   }
// }

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

//   // 回転動作(AR,IMUR,MCA,BCA)の場合、"clockwise"ならtrue（時計回り）、"anticlockwise"ならfalse（反時計回り）に変換
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
