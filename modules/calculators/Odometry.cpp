/**
 * @file   Odometry.cpp
 * @brief  移動距離と向いている方向からPositionを更新するクラス
 * @author yutaro-1214
 */

#include "Odometry.h"
#include <cmath>
#include "AngleNormalizer.h"
#include "Mileage.h"

Odometry::Odometry(Position& position) : position(position), prevEncoderLeft(0), prevEncoderRight(0)
{
}

void Odometry::reset()
{
  // Positionをリセット
  position.set(0.0, 0.0, 0.0);
}

void Odometry::initialize(int32_t encoderLeft, int32_t encoderRight)
{
  // エンコーダ値をセット
  prevEncoderLeft = encoderLeft;
  prevEncoderRight = encoderRight;
}

void Odometry::update(int32_t encoderLeft, int32_t encoderRight, double heading)
{
  // エンコーダ差分
  int32_t dEncoderLeft = encoderLeft - prevEncoderLeft;
  int32_t dEncoderRight = encoderRight - prevEncoderRight;

  // 差分から移動距離を計算
  double distance = Mileage::calculateMileage(dEncoderRight, dEncoderLeft);

  // 現在値を保存
  prevEncoderLeft = encoderLeft;
  prevEncoderRight = encoderRight;

  // IMU角度を正規化
  heading = AngleNormalizer::normalizeAngle(heading);

  double rad = heading * DEG_TO_RAD;

  // 自己位置更新
  double newX = position.getX() + distance * std::cos(rad);
  double newY = position.getY() + distance * std::sin(rad);

  position.set(newX, newY, heading);
}