#include <cstdio>
#include <iostream>

void drawGraphWithPopen()
{
  if(!gp) {
    std::cerr << "gnuplotの起動に失敗" << std::endl;
    return;
  }

  // gnuplotでグラフ描画を指定する。
  fprintf(gp, "set datafile separator ','\n");
  fprintf(gp, "set grid\n");
  // 1行目を列名として扱う
  fprintf(gp, "set datafile columnheaders\n");
  // 列名を指定
  fprintf(
      gp,
      "file = './datafiles/logfiles/quick_trun/quickTurnLog1.csv'\n"
      "plot file using 'time':'comand:id' with lines lc rgb 'black' title 'comand:id', "
      "file using 'time':'currentRightMotorPower' with lines lc rgb 'orange' title "
      "'currentRightMotorPower', "
      "file using 'time':'currentRightSpeed' with lines lc rgb 'green' title 'currentRightSpeed', "
      "file using 'time':'baseRightPower' with lines lc rgb 'yellow' title 'baseRightPower', "
      "file using 'time':'currentLeftMotorPower' with lines lc rgb 'purple' title "
      "'currentLeftMotorPower', "
      "file using 'time':'currentLeftSpeed' with lines lc rgb 'cyan' title 'currentLeftSpeed', "
      "file using 'time':'baseLeftPower' with lines lc rgb 'magenta' title 'baseLeftPower', "
      "file using 'time':'turningPower' with lines lc rgb 'web-green' title 'turningPower', "
      "file using 'time':'finalRightMotorPower' with lines lc rgb 'violet' title "
      "'finalRightMotorPower', "
      "file using 'time':'finalLeftMotorPower' with lines lc rgb 'dark-red' title "
      "'finalLeftMotorPower', "
      "\n");

  fflush(gp);
  pclose(gp);
}

int main()
{
  drawGraphWithPopen();
  return 0;
}
