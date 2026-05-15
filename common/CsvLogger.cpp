/**
 * @file CsvLogger.cpp
 * @brief 走行中の各種値可視化のためのcsvファイル作成に関するクラス
 * @author miyahara046
 */

#include "CsvLogger.h"

char CsvLogger::logs[LOG_BUFFER_SIZE] = "";

int CsvLogger::currentIndex = 0;

// CSVログの初期化
void CsvLogger::init()
{
  logs[0] = '\0';
  currentIndex = 0;
}

// CSVヘッダを書き込む
void CsvLogger::writeHeader()
{
  int remainBuffer =
      LOG_BUFFER_SIZE - currentIndex - 1;

  if(remainBuffer <= 0) {
    return;
  }

  int written =
      snprintf(&logs[currentIndex],
               remainBuffer,
               "time,brightness,rightPwm,leftPwm\n");

  if(written >= remainBuffer) {
    currentIndex = LOG_BUFFER_SIZE - 1;
  } else if(written > 0) {
    currentIndex += written;
  }
}

// 各種値を追加する
void CsvLogger::add(int brightness,
                    int rightPwm,
                    int leftPwm)
{
  int remainBuffer =
      LOG_BUFFER_SIZE - currentIndex - 1;

  if(remainBuffer <= 0) {
    return;
  }

  int written =
      snprintf(&logs[currentIndex],
               remainBuffer,
               "%d,%d,%d\n",
              //  "%d,%d,%d,%d\n",
              //  ClockUtil::now(),
               brightness,
               rightPwm,
               leftPwm
              );

  if(written >= remainBuffer) {
    currentIndex = LOG_BUFFER_SIZE - 1;
  } else if(written > 0) {
    currentIndex += written;
  }
}

// 記録した走行ログをファイル出力する
void CsvLogger::outputToFile()
{

  FILE* outputFile =
      fopen(csvFileName, "w");

  if(outputFile == NULL) {
    printf("cannot open csv file\n");
    return;
  }

  printf("csv file open\n");

  fprintf(outputFile, "%s", logs);

  fclose(outputFile);
}