/**
 * @file   BaseMotion.cpp
 * @brief  動作の実行処理を共通化するための基底クラス
 * @author takuchi17
 */

#include "ChangeEdge.h"

ChangeEdge::ChangeEdge(Robot& _robot,
                       std::unique_ptr<BaseContinuationCondition> _continuationCondition,
                       std::string _edge)
  : BaseMotion(_robot, std::move(_continuationCondition)), edge(_edge)
{
  LOG_CREATE("ChangeEdge");
}
ChangeEdge::~ChangeEdge()
{
  LOG_DESTROY("ChangeEdge");
}
bool ChangeEdge::canStart()
{
  if(edge != "Left" || edge != "Right") {
    Logger::error("ChangeEdge:Left or Right を入力してください");
    return false;
  }
  return true;
}

void ChangeEdge::executeStep()
{
  if(flag == true) {
    return;
  }

  if(edge == "Left") {
    robot.setEdge(Edge::LeftEdge);
    Logger::info("ChangeEdge:左エッジに変更");
  } else if(edge == "Right") {
    robot.setEdge(Edge::RightEdge);
    Logger::info("ChangeEdge:右エッジに変更");
  }

  flag = true;
}
