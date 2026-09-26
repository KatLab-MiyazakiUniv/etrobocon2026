/**
 * @file   EtRobocon2026.h
 * @brief  全体を制御するクラス
 * @author HaruArima08
 */

#ifndef ETROBOCON2026_H
#define ETROBOCON2026_H

#include <iostream>
#include "Logger.h"

#include <memory>
#include "AreaMaster.h"
#include "ClockUtil.h"
#include "CourseMirror.h"
#include "DijkstraRoutePlanner.h"
#include "DistanceCondition.h"
#include "EtRallyMap.h"
#include "GateRoutePlanner.h"
#include "Logger.h"
#include "MapData.h"
#include "Pid.h"
#include "RealNetworkSystem.h"
#include "Robot.h"
#include "RouteFollower.h"
#include "RouteTypes.h"
#include "SocketClient.h"
#include "Straight.h"
#include "SystemInfo.h"
#include "BatteryController.h"

class EtRobocon2026 {
 public:
  static void start();
};

#endif