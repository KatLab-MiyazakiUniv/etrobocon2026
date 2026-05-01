/**
 * @file   EtRobocon2025.cpp
 * @brief  全体を制御するクラス
 * @author takahashitom
 */

#include "EtRobocon2026.h"
#include "spikeapi.h"
#include "MotorController.h"

void EtRobocon2026::start()
{
  std::cout << "Hello KATLAB" << std::endl;
  MotorController controller;
  // // controller.setRightMotorPower(50);

  // controller.setRightMotorSpeed(720);
  // controller.setLeftMotorSpeed(720);

  // // controller.setLeftMotorPower(50);
  // tslp_tsk(3000000);

  // printf("right:%d\n", controller.getRightMotorPower());
  // printf("left:%d\n", controller.getLeftMotorPower());
  // tslp_tsk(5000000);

  // // controller.resetRightMotorPower();
  // // controller.resetLeftMotorPower();

  // controller.setArmMotorPower(30);
  // tslp_tsk(1000000);
  // controller.holdArmMotor();
  // tslp_tsk(2000000);

  // controller.setArmMotorPower(-30);
  // tslp_tsk(1000000);

  // // controller.stopWheelsMotor();
  // controller.brakeWheelsMotor();
  // controller.stopArmMotor();
  int i;
  for(i = 0; i < 100; i++) {
    if(i == 0) {
      controller.setRightMotorSpeed(-1000);
      controller.setLeftMotorSpeed(-1000);
    } else if(i == 70) {
      controller.setArmMotorPower(30);
    } else if(i == 80) {
      controller.holdArmMotor();
    } else if(i == 90) {
      controller.setArmMotorPower(-30);
    }
    printf("right:%d\n", controller.getRightMotorPower());
    // printf("left:%d\n", controller.getLeftMotorPower());
    tslp_tsk(100000);
  }
  controller.brakeWheelsMotor();
}