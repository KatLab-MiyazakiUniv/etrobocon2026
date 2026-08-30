
/**
 * @file   GatePositionParser.cpp
 * @brief  平文からゲート位置情報を解析するクラス
 * @author migaku2645
 */

#include "GatePositionParser.h"

GatePositionParser::GatePositionParser(const std::string& _plaintext, Robot& _robot)
  : plaintext(_plaintext), robot(_robot)
{
  LOG_CREATE("GatePositionParser");
}

GatePositionParser::~GatePositionParser()
{
  LOG_DESTROY("GatePositionParser");
}

bool GatePositionParser::parse()
{
  if(plaintext.empty()) {
    Logger::error("GatePositionParser: 平文が空です。");
    return false;
  }

  // XY,XY形式
  if(isRedFormat()) {
    return parseRedPosition();
  }

  // XY,XY/XY,XY形式
  if(isBlueYellowFormat()) {
    return parseBlueYellowPosition();
  }

  Logger::error("GatePositionParser: 平文の形式が不正です。");

  return false;
}

bool GatePositionParser::isRedFormat() const
{
  // "/"が存在しない場合は赤色ゲート形式
  return plaintext.find('/') == std::string::npos;
}

bool GatePositionParser::isBlueYellowFormat() const
{
  std::size_t firstSlash = plaintext.find('/');

  if(firstSlash == std::string::npos) {
    return false;
  }

  // "/"が2つ以上存在する場合は不正
  std::size_t secondSlash = plaintext.find('/', firstSlash + 1);

  return secondSlash == std::string::npos;
}

bool GatePositionParser::parseRedPosition()
{
  int position[2];

  if(!parsePosition(plaintext, position)) {
    Logger::error("GatePositionParser: 赤色ゲート位置情報の解析に失敗しました。");
    return false;
  }

  robot.getGatePosition().setPosition(GateColor::RED, position[0], position[1]);

  return true;
}

bool GatePositionParser::parseBlueYellowPosition()
{
  std::size_t slashPosition = plaintext.find('/');

  // "/"を基準に青と黄色を分割
  std::string blueText = plaintext.substr(0, slashPosition);

  std::string yellowText = plaintext.substr(slashPosition + 1);

  int bluePosition[2];
  int yellowPosition[2];

  // 青色ゲート
  if(!parsePosition(blueText, bluePosition)) {
    Logger::error("GatePositionParser: 青色ゲート位置情報の解析に失敗しました。");
    return false;
  }

  // 黄色ゲート
  if(!parsePosition(yellowText, yellowPosition)) {
    Logger::error("GatePositionParser: 黄色ゲート位置情報の解析に失敗しました。");
    return false;
  }

  robot.getGatePosition().setPosition(GateColor::BLUE, bluePosition[0], bluePosition[1]);

  robot.getGatePosition().setPosition(GateColor::YELLOW, yellowPosition[0], yellowPosition[1]);

  return true;
}

bool GatePositionParser::parsePosition(const std::string& text, int positions[2])
{
  std::stringstream ss(text);

  std::string first;
  std::string second;

  // 1つ目の値
  if(!std::getline(ss, first, ',')) {
    Logger::error("GatePositionParser: 1つ目の位置情報を取得できません。");
    return false;
  }

  // 2つ目の値
  if(!std::getline(ss, second)) {
    Logger::error("GatePositionParser: 2つ目の位置情報を取得できません。");
    return false;
  }

  // 余分なカンマがないか確認
  std::string extra;

  if(std::getline(ss, extra, ',')) {
    Logger::error("GatePositionParser: 位置情報に余分な値があります。");
    return false;
  }

  try {
    positions[0] = std::stoi(first);
    positions[1] = std::stoi(second);
  } catch(...) {
    Logger::error("GatePositionParser: 位置情報を整数に変換できません。");
    return false;
  }

  // 2桁の整数か確認
  if(positions[0] < 10 || positions[0] > 99 || positions[1] < 10 || positions[1] > 99) {
    Logger::error("GatePositionParser: 位置情報は2桁の整数で指定してください。");

    return false;
  }

  return true;
}