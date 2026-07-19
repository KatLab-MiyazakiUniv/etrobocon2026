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