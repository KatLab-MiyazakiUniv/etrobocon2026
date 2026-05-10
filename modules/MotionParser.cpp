/**
 * @file   MotionParser.cpp
 * @brief  動作コマンドファイルを解析するクラス
 * @author nishijima515
 */

#include "MotionParser.h"

using namespace std;

// stoi/stodの代替関数 (v850クロスコンパイラはlibstdc++が簡略版のため、std::stoi/std::stodが利用不可)
template<typename T>
bool tryParse(const std::string& s);

// int型用の特化：strtolを利用して確実に判定（組み込みのistringstreamの不具合回避）
template<>
bool tryParse<int>(const std::string& s)
{
    const char* p = s.c_str();
    char* end;
    errno = 0; // stdlib.h などのインクルードが必要だが、下部に既にあると想定
    long x = std::strtol(p, &end, 10);
    if (p == end || *end != '\0') return false;
    if (errno == ERANGE || x < INT_MIN || x > INT_MAX) return false;
    return true;
}

// double型用の特化：strtodを利用して確実に判定
template<>
bool tryParse<double>(const std::string& s)
{
    const char* p = s.c_str();
    char* end;
    errno = 0;
    std::strtod(p, &end);
    if (p == end || *end != '\0') return false;
    if (errno == ERANGE) return false;
    return true;
}

const array<string, 2> MotionParser::areaCommandNames = { "LineTrace", "Area2" };

void MotionParser::createRunCSV(Robot& robot, Area area, bool isLeftCourse)
{
    cout << "実行用コマンドファイルの作成を開始します" << endl;

    string areaPath = "../etrobocon2026/datafiles/commands/Area/";
    string motionsPath = "../etrobocon2026/datafiles/commands/Motions/";
    string runPath = "../etrobocon2026/datafiles/commands/Run/";

    string areaName = areaCommandNames[static_cast<int>(area)];
    string course = isLeftCourse ? "Left" : "Right";

    string commandAreaFilePath = areaPath + areaName + course + ".csv";
    string commandRunFilePath = runPath + "run_" + areaName + course + ".csv";

    cout << commandAreaFilePath << endl;
    cout << commandRunFilePath << endl;

    // 文字列の先頭と末尾の空白を削除するラムダ式
    auto trim = [](string& s) {
        size_t start = s.find_first_not_of(" \t");
        if (start == string::npos) {
            s.clear();
            return;
        }
        size_t end = s.find_last_not_of(" \t");
        s = s.substr(start, end - start + 1);
    };

    // 入力ファイルを読み取りモードで開く
    ifstream commandAreaFile(commandAreaFilePath);
    if (!commandAreaFile) {
        cerr << "エリアコマンドファイルを開けませんでした: " << commandAreaFilePath << endl;
        return;
    }

    // 出力ファイルを書き込みモードで開き、初期化する
    ofstream commandRunFile(commandRunFilePath, ios::out | ios::trunc);
    if (!commandRunFile) {
        cerr << "実行用コマンドファイルを開けませんでした: " << commandRunFilePath << endl;
        return;
    }


    // 改行制御フラグ
    bool firstWrite = true;

    string areaFileLine;

    while (getline(commandAreaFile, areaFileLine)) {

        // コメント削除
        size_t areaPos = areaFileLine.find('#');
        if (areaPos != string::npos) {
            areaFileLine = areaFileLine.substr(0, areaPos);
        }

        // trim（行全体）
        trim(areaFileLine);

        // 空行スキップ
        if (areaFileLine.empty()) continue;

        // CSV分割
        stringstream ssArea(areaFileLine);
        vector<string> areaFileParams;
        string token;

        while (getline(ssArea, token, SEPARATOR)) {
            trim(token);
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

            // コメント削除
            size_t motionsPos = motionsFileLine.find('#');
            if (motionsPos != string::npos) {
                motionsFileLine = motionsFileLine.substr(0, motionsPos);
            }

            // trim
            trim(motionsFileLine);

            if (motionsFileLine.empty()) continue;

            // CSV分割
            stringstream ssMotions(motionsFileLine);
            vector<string> motionParams;

            while (getline(ssMotions, token, SEPARATOR)) {
                trim(token);
                motionParams.push_back(token);
            }

            cout << "比較: commandID vs motionID = [ " << commandID << " ] vs [ " << motionParams[1] << " ]" << endl;
            // ID一致チェック（2列目がID想定）
            if (motionParams.size() >= 2 && motionParams[1] == commandID) {

                // 1行目以外は改行してから書き込む
                if (!firstWrite) {
                    commandRunFile << "\n";
                }

                commandRunFile << motionsFileLine;
                firstWrite = false;

                // commandRunFile << motionsFileLine << "\n";
                break;
            }
        }
    }

    cout << "実行用コマンドファイルの作成が完了しました" << endl;

    commandRunFile.close(); // バッファを書き出してファイルを閉じる

    checkType(commandRunFilePath);
}

/**
 * @brief 指定したcsvファイルの各コマンド毎にパラメータの型が正しいかチェックする関数
 * @param commandFilePath 読み込むcsvファイルのファイルパス
 * @return すべてのコマンドの型が一致していればtrue、不一致があればfalse
 */
bool MotionParser::checkType(string& commandFilePath)
{
    // 1. 指定されたファイルパスのファイルを開く
    ifstream file(commandFilePath);
    if (!file) {
        cerr << "実行用コマンドファイルを開けないため、型チェックは行いません: " << commandFilePath << endl;
        return false;
    }

    string line;
    int lineNum = 1;
    bool allValid = true;

    // 文字列の先頭と末尾の空白を削除するラムダ式
    auto trim = [](string& s) {
        size_t start = s.find_first_not_of(" \t");
        if (start == string::npos) {
            s.clear();
            return;
        }
        size_t end = s.find_last_not_of(" \t");
        s = s.substr(start, end - start + 1);
    };

    // 2. csvファイルを1行ずつ読み込む
    while (getline(file, line)) {
        // csvファイル内の「#」以降はコメントとして扱うため削除する
        size_t pos = line.find('#');
        if (pos != string::npos) {
            line = line.substr(0, pos);
        }

        trim(line);
        // 空行はスキップ
        if (line.empty()) {
            lineNum++;
            continue;
        }

        // 3. カンマ区切りでコマンド名とその引数を1つずつ取り出して params に追加
        stringstream ss(line);
        vector<string> params;
        string token;
        while (getline(ss, token, SEPARATOR)) {
            trim(token);
            params.push_back(token);
        }

        if (params.empty()) {
            lineNum++;
            continue;
        }

        string commandName = params[0];
        // 4. params[1] はIDであるため、型チェックのときは考慮しない

        // 5. 各コマンドに対応する関数で定義している引数の型とパラメータの値が一致しているかチェックする
        //
        // =====================================================================
        // 【新しいコマンドを追加するときのガイド】
        //
        //   下の if (commandName == ...) のブロックを追加してください。
        //
        //   ＜ARコマンドを例に＞　AR=去年の角度指定回頭(AngleRotation)
        //   ARコマンドのcsvの書式は以下の通りです:
        //     AR, <ID>, <角度(int)>, <速度(double)>, <方向(string)>
        //   params[0]=コマンド名, params[1]=ID, params[2]以降が実際の引数です
        //   ※ params[1] はIDのため型チェックでは考慮しない
        //
        //   必要な引数の数（ARなら5つ）を params.size() で事前にチェックし、
        //   tryParse<T>() によって整数や小数が正しい形式か例外(throw)を使わずにチェックします。
        // =====================================================================

        if (commandName == "EXAMPLE") {
            char lineBuf[32];
            sprintf(lineBuf, "%d", lineNum);
            
            // EXAMPLE: テスト用コマンド (int パラメータ, double パラメータ, string パラメータ("left" or "right"))
            if (params.size() < 5) {
                cerr << commandFilePath << ":" << lineBuf << "行目 (" << commandName << "): 引数の数が足りません" << endl;
                allValid = false;
            } else {
                if (!tryParse<int>(params[2])) {
                    cerr << commandFilePath << ":" << lineBuf << "行目 (" << commandName << "): 引数の型が不正です (intであるべき場所に文字が含まれています)" << endl;
                    allValid = false;
                }
                if (!tryParse<double>(params[3])) {
                    cerr << commandFilePath << ":" << lineBuf << "行目 (" << commandName << "): 引数の型が不正です (doubleであるべき場所に文字が含まれています)" << endl;
                    allValid = false;
                }
                if (params[4] != "left" && params[4] != "right") {
                    cerr << commandFilePath << ":" << lineBuf << "行目 (" << commandName << "): 'left' か 'right' を指定してください" << endl;
                    allValid = false;
                }
            }
        }
        // ↓ 新しいコマンドはここに追加していく
        // else if (commandName == "コマンド名") { ... }

        lineNum++;
    }

    // 6. 全ての行のチェックが完了したら、最終結果をサマリーとして出力して返す
    if (allValid) {
        cout << "型チェック完了: 「" << commandFilePath << "」に問題はありませんでした" << endl;
    } else {
        cout << "型チェック完了: 「" << commandFilePath << "」に問題が見つかりました。上記のエラーを確認してください" << endl;
    }
    return allValid;
}



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
    //     cout << commandFilePath << ":" << lineNum << " Command " << params[0] << " は未定義です" << endl;
    //     break;
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
    { "EXAMPLE", COMMAND::EXAMPLE },         // 例
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




int MotionParser::stoi(const std::string& str, std::size_t* idx, int base) {
  const char* p = str.c_str();
  char* end;
  errno = 0;
  long x = std::strtol(p, &end, base);
  if (p == end) {
    throw std::invalid_argument("stoi");
  }
  if (errno == ERANGE || x < INT_MIN || x > INT_MAX) {
    throw std::out_of_range("stoi");
  }
  if (idx != nullptr) {
    *idx = static_cast<std::size_t>(end - p);
  }
  return static_cast<int>(x);
}

int MotionParser::stoi(const std::wstring& str, std::size_t* idx, int base) {
  const wchar_t* p = str.c_str();
  wchar_t* end;
  errno = 0;
  long x = std::wcstol(p, &end, base);
  if (p == end) {
    throw std::invalid_argument("stoi");
  }
  if (errno == ERANGE || x < INT_MIN || x > INT_MAX) {
    throw std::out_of_range("stoi");
  }
  if (idx != nullptr) {
    *idx = static_cast<std::size_t>(end - p);
  }
  return static_cast<int>(x);
}

double MotionParser::stod(const std::string& str, std::size_t* idx) {
  const char* p = str.c_str();
  char* end;
  errno = 0;
  double x = std::strtod(p, &end);
  if (p == end) {
    throw std::invalid_argument("stod");
  }
  if (errno == ERANGE) {
    throw std::out_of_range("stod");
  }
  if (idx != nullptr) {
    *idx = static_cast<std::size_t>(end - p);
  }
  return x;
}

double MotionParser::stod(const std::wstring& str, std::size_t* idx) {
  const wchar_t* p = str.c_str();
  wchar_t* end;
  errno = 0;
  double x = std::wcstod(p, &end);
  if (p == end) {
    throw std::invalid_argument("stod");
  }
  if (errno == ERANGE) {
    throw std::out_of_range("stod");
  }
  if (idx != nullptr) {
    *idx = static_cast<std::size_t>(end - p);
  }
  return x;
}