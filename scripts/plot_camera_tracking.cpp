#include <cstdio>    //c言語の関数:popen(),fprintf(),fflu:Telescope live_grepsh(),pclose()
#include <iostream>  //c++の入出力ストリームstd::cerr

void drawGraphWithPopen()
{
  // gnuplotを対話モードで起動
  //popen("termコマンド", "モード") termコマンドを実行する。返り値は*FILEで対話モードなら、そのプロセスとのストリームを返す
  // ファイルストリームは対話モードのプロセスとの通信もできるのか
  //そのファイルストリームを表すのはFILE*というポインタなのか
  FILE* gp = popen("gnuplot -persist", "w");
  if(!gp) {
    std::cerr << "gnuplotの起動に失敗" << std::endl;
    return;
  }

  // gnuplotでグラフ描画を指定する。
  fprintf(gp, "set datafile separator ','\n");
  fprintf(gp, "set grid\n");
  // 1. 1行目を列名として扱う設定を有効にする
  fprintf(gp, "set datafile columnheaders\n");

  // 2. 列名をそのまま指定してプロットする
fprintf(gp,
    "file = './datafiles/logfiles/quick_trun/quickTurnLog1.csv'\n"
    "plot file using 'time':'comand:id' with lines lc rgb 'black' title 'comand:id', "
         "file using 'time':'currentRightMotorPower' with lines lc rgb 'orange' title 'currentRightMotorPower', "
         "file using 'time':'currentRightSpeed' with lines lc rgb 'green' title 'currentRightSpeed', "
         "file using 'time':'baseRightPower' with lines lc rgb 'yellow' title 'baseRightPower', "
         "file using 'time':'currentLeftMotorPower' with lines lc rgb 'purple' title 'currentLeftMotorPower', "
         "file using 'time':'currentLeftSpeed' with lines lc rgb 'cyan' title 'currentLeftSpeed', "
         "file using 'time':'baseLeftPower' with lines lc rgb 'magenta' title 'baseLeftPower', "
         "file using 'time':'turningPower' with lines lc rgb 'web-green' title 'turningPower', "
         "file using 'time':'finalRightMotorPower' with lines lc rgb 'violet' title 'finalRightMotorPower', "
         "file using 'time':'finalLeftMotorPower' with lines lc rgb 'dark-red' title 'finalLeftMotorPower', "
         "file using ($3 == 9999 ? $1 : 1/0) : ($3 == 9999 ? 0 : 1/0) with points pointtype 7 pointsize 2 lc rgb 'red' title 'End Mark' "
         "\n"
  );

// fprintf(gp,
//     "file = './datafiles/logfiles/runlog3.csv'\n"
//     "set datafile separator ','\n"
//     "set grid\n"
//
//     // 一度 columnheaders を無効（unset）にして、純粋な数値の列番号（$1, $21, $27）で確実に処理させる
//     "unset datafile columnheaders\n"
//     // 1行目がヘッダーである場合、データとしてはスキップ（skip 1）させる
//     "plot file skip 1 using 1:2 with lines lc rgb 'black' title 'comand:id', "
//     //      "file skip 1 using 1:($24 == $28 ? 1 : 3) with lines title 'End Mark'\n"
//     "file skip 1 using 1 : ($23 - $26)  with lines title 'diffBasePower', "
//     "file skip 1 using 1 : ($30 - $31) with lines title 'diffFinalPower', "
//     "\n"
//
//   );





// fprintf(gp, R"(
//     # awkスクリプトの区切りを見直し、エラーが起きないように整理
//     file = "< awk -F',' 'NR==1 {print $0 \",period\"; next} {dt=$1-p; p=$1; print $0 \",\" dt}' ./datafiles/logfiles/runlog3.csv"
//
//     # プロットの実行
//     plot file using 1:33 with lines title 'period', \
//          file using ($3 == 9999 ? $1 : 1/0) : ($3 == 9999 ? $33 : 1/0) with points pointtype 7 pointsize 2 lc rgb "red" title 'End Mark'
// )");


  // 1. 画面を 2行×1列 に分割する設定を開始
  // fprintf(gp, "set multiplot layout 2,1\n");
  // 4. マルチプロットモードを終了
  // fprintf(gp, "unset multiplot\n");
  // 2'\n"); ストリームにためたバッファをプロセスに即時、吐き出して空にする
  fflush(gp);
  // パイプを閉じる
  pclose(gp);
}

int main()
{
  drawGraphWithPopen();
  return 0;
}




// printf(gp,
//     "file = './datafiles/logfiles/runlog3.csv'\n"
//     "plot file using 'time':'comand:id' with lines lc rgb 'black' title 'comand:id', "
//          // "file using 'time':'targetSpeed' with lines lc rgb 'blue' title 'targetSpeed', "
//          // "file using 'time':'currentRightMotorPower' with lines lc rgb 'orange' title 'currentRightMotorPower', "
//          // "file using 'time':'currentRightSpeed' with lines lc rgb 'green' title 'currentRightSpeed', "
//          // "file using 'time':'baseRightPower' with lines lc rgb 'yellow' title 'baseRightPower', "
//          // "file using 'time':'currentLeftMotorPower' with lines lc rgb 'purple' title 'currentLeftMotorPower', "
//          // "file using 'time':'currentLeftSpeed' with lines lc rgb 'cyan' title 'currentLeftSpeed', "
//          // "file using 'time':'baseLeftPower' with lines lc rgb 'magenta' title 'baseLeftPower', "
//          // "file using 'time':'afterDetectRightMotorPower' with lines lc rgb 'coral' title 'afterDetectRightMotorPower', "
//          // "file using 'time':'afterDetectLeftMotorPower' with lines lc rgb 'brown' title 'afterDetectLeftMotorPower', "
//          // "file using 'time':'turningPower' with lines lc rgb 'web-green' title 'turningPower', "
//          // "file using 'time':'finalRightMotorPower' with lines lc rgb 'violet' title 'finalRightMotorPower', "
//          // "file using 'time':'finalLeftMotorPower' with lines lc rgb 'dark-red' title 'finalLeftMotorPower', "
//          // "file using 'time':'isReverseTurningPowerAndFinalPower' with lines lc rgb 'gray' title 'isReverseTurningPowerAndFinalPower', "
//          // "file using ($3 == 9999 ? $1 : 1/0) : ($3 == 9999 ? 0 : 1/0) with points pointtype 7 pointsize 2 lc rgb 'red' title 'End Mark'
//          "\n"
//         )
