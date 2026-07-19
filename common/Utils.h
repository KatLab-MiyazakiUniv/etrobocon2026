/**
 * @file   Utils.h
 * @brief  便利関数のクラス
 * @author sadomiya-sousi
 */

#include <string>

class Utils {
 public:
  /**
   * @brief 値をint型にキャストして文字列に変換するヘルパー関数
   * @param value 変換する値
   * @return std::string 変換後の文字列
   */
  static std::string toStrInt(double value);

 private:
  Utils();  // インスタンス化禁止
};