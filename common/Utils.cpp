/**
 * @file   Utils.cpp
 * @brief  便利関数のクラス
 * @author sadomiya-sousi
 */

#include "Utils.h"

std::string Utils::toStrInt(double value)
{
  return std::to_string(static_cast<int>(value));
}


std::tm Utils::getLocalTime()
{

    std::tm globalTime{};
    //UNIX タイムスタンプ
    //引数にnullptrを指定する事で,返り値を受け取る
    std::time_t now = std::time(nullptr);
    //Docker環境では地域標準時は世界協定時
    //明示的に世界協定時
    gmtime_r(&now, &globalTime);

    globalTime.tm_year += 1900;
    globalTime.tm_mon += 1;
    globalTime.tm_hour += 9;

    return globalTime;
}

