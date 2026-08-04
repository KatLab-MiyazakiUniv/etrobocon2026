/**
 * @file   BenchmarkCsvWriter.cpp
 * @brief  復号性能の計測結果をCSVへ出力するクラス
 * @author HaruArima08
 */

#include "BenchmarkCsvWriter.h"
#include "Logger.h"
#include <filesystem>

BenchmarkCsvWriter::BenchmarkCsvWriter(const std::string& _filePath) : filePath(_filePath) {}

BenchmarkCsvWriter::~BenchmarkCsvWriter()
{
  close();
}

bool BenchmarkCsvWriter::open()
{
  // 出力先ディレクトリが無い場合に備えて作成しておく
  std::filesystem::path path(filePath);
  if(path.has_parent_path()) {
    std::error_code errorCode;
    std::filesystem::create_directories(path.parent_path(), errorCode);
  }

  ofs.open(filePath, std::ios::out | std::ios::trunc);
  if(!ofs.is_open()) {
    Logger::printfLog(Logger::ERROR, "BenchmarkCsvWriter:open: %sを開けませんでした",
                      filePath.c_str());
    return false;
  }

  ofs << "trial,mode,serverIp,port,encryptedTextLength,iterations,"
      << "totalMicro,serverDecryptMicro,commMicro,pingMicro,success\n";
  return true;
}

void BenchmarkCsvWriter::writeRow(int trial, DecryptMode mode, const std::string& serverIp,
                                  int port, size_t encryptedTextLength, uint32_t iterations,
                                  const DecryptMeasurement& measurement, uint64_t pingMicro)
{
  if(!ofs.is_open()) return;

  ofs << trial << ',' << toDecryptModeName(mode) << ',' << serverIp << ',' << port << ','
      << encryptedTextLength << ',' << iterations << ',' << measurement.totalMicro << ','
      << measurement.serverDecryptMicro << ',' << measurement.commMicro << ',' << pingMicro << ','
      << (measurement.success ? 1 : 0) << '\n';
}

void BenchmarkCsvWriter::close()
{
  if(ofs.is_open()) {
    ofs.flush();
    ofs.close();
  }
}
