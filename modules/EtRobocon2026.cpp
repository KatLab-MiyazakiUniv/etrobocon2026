/**
 * @file   EtRobocon2026.cpp
 * @brief  全体を制御するクラス
 * @author HaruArima08
 */
void EtRobocon2026::start()
{
  std::cout << "Hello KATLAB" << std::endl;

  RelativeRotation rotation(robot, 80.0);

  rotation.prepare();

  while(!rotation.isFinished()) {
    rotation.executeStep();
  }

  rotation.finish();
}