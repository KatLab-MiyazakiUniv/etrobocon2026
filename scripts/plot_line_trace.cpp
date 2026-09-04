/**
 * @file plot_line_trace.cpp
 * @brief LineTraceのCSVログから、SVGグラフを生成する
 */

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

  struct Row {
    double time = 0.0;
    std::string id;
    std::map<std::string, double> values;
  };

  std::vector<std::string> splitCsv(const std::string& line)
  {
    std::vector<std::string> fields;
    std::string field;
    bool quoted = false;
    for(size_t i = 0; i < line.size(); ++i) {
      char ch = line[i];
      if(ch == '"') {
        if(quoted && i + 1 < line.size() && line[i + 1] == '"') {
          field += '"';
          ++i;
        } else {
          quoted = !quoted;
        }
      } else if(ch == ',' && !quoted) {
        fields.push_back(field);
        field.clear();
      } else {
        field += ch;
      }
    }
    fields.push_back(field);
    return fields;
  }

  double parseNumber(const std::string& value)
  {
    if(value.empty()) return std::numeric_limits<double>::quiet_NaN();
    try {
      return std::stod(value);
    } catch(...) {
      return std::numeric_limits<double>::quiet_NaN();
    }
  }

  std::vector<Row> loadRows(const std::string& path)
  {
    std::ifstream input(path);
    if(!input) throw std::runtime_error("CSVを開けません: " + path);

    std::string line;
    if(!std::getline(input, line)) throw std::runtime_error("CSVが空です");
    const auto headers = splitCsv(line);
    std::map<std::string, size_t> column;
    for(size_t i = 0; i < headers.size(); ++i) column[headers[i]] = i;

    const std::vector<std::string> required
        = { "time", "comand:id",   "brightness",   "target",     "error",    "rawTurn",
            "turn", "maxoutPower", "maxoutActive", "rightPower", "leftPower" };
    for(const auto& name : required) {
      if(column.count(name) == 0) throw std::runtime_error("CSV列がありません: " + name);
    }

    std::vector<Row> rows;
    while(std::getline(input, line)) {
      const auto fields = splitCsv(line);
      if(fields.size() < headers.size()) continue;
      Row row;
      row.time = parseNumber(fields[column["time"]]);
      row.id = fields[column["comand:id"]];
      for(const auto& name : required) {
        if(name != "comand:id" && name != "time") {
          row.values[name] = parseNumber(fields[column[name]]);
        }
      }
      if(std::isfinite(row.time)) rows.push_back(row);
    }
    if(rows.empty()) throw std::runtime_error("CSVに走行データがありません");
    return rows;
  }

  std::string escapeXml(const std::string& text)
  {
    std::string result;
    for(char ch : text) {
      if(ch == '&')
        result += "&amp;";
      else if(ch == '<')
        result += "&lt;";
      else if(ch == '>')
        result += "&gt;";
      else
        result += ch;
    }
    return result;
  }

  struct Series {
    std::string column;
    std::string label;
    std::string color;
    bool negate = false;
  };

  void drawPanel(std::ostream& out, const std::vector<Row>& rows, double startTime, int top,
                 const std::string& title, const std::vector<Series>& series)
  {
    constexpr int left = 85;
    constexpr int width = 1100;
    constexpr int height = 185;
    double minValue = std::numeric_limits<double>::infinity();
    double maxValue = -std::numeric_limits<double>::infinity();
    for(const auto& item : series) {
      for(const auto& row : rows) {
        double value = row.values.at(item.column) * (item.negate ? -1.0 : 1.0);
        if(std::isfinite(value)) {
          minValue = std::min(minValue, value);
          maxValue = std::max(maxValue, value);
        }
      }
    }
    if(!std::isfinite(minValue) || !std::isfinite(maxValue)) return;
    if(minValue == maxValue) {
      minValue -= 1.0;
      maxValue += 1.0;
    }
    const double padding = (maxValue - minValue) * 0.08;
    minValue -= padding;
    maxValue += padding;
    const double duration = std::max(1.0, rows.back().time - startTime);
    auto x = [&](double time) { return left + (time - startTime) / duration * width; };
    auto y = [&](double value) {
      return top + height - (value - minValue) / (maxValue - minValue) * height;
    };

    out << "<rect x='" << left << "' y='" << top << "' width='" << width << "' height='" << height
        << "' fill='white' stroke='#aaa'/>\n";

    // コマンド区間を交互の背景色で塗り、同じIDの再登場も別区間として表示する。
    size_t sectionStart = 0;
    int sectionIndex = 0;
    for(size_t i = 1; i <= rows.size(); ++i) {
      if(i == rows.size() || rows[i].id != rows[sectionStart].id) {
        const double sectionStartX = x(rows[sectionStart].time);
        const double sectionEndTime = i < rows.size() ? rows[i].time : rows.back().time;
        const double sectionEndX = x(sectionEndTime);
        const char* fill = sectionIndex % 2 == 0 ? "#e8f2ff" : "#fff1df";
        out << "<rect x='" << sectionStartX << "' y='" << top << "' width='"
            << std::max(0.0, sectionEndX - sectionStartX) << "' height='" << height << "' fill='"
            << fill << "' opacity='0.55'/>\n";
        if(top == 75) {
          out << "<text x='" << (sectionStartX + sectionEndX) / 2.0 << "' y='" << top - 9
              << "' text-anchor='middle' font-size='13' font-weight='bold'>"
              << escapeXml(rows[sectionStart].id) << "</text>\n";
          out << "<line x1='" << sectionStartX << "' y1='" << top - 5 << "' x2='" << sectionEndX
              << "' y2='" << top - 5 << "' stroke='#555'/>\n";
        }
        sectionStart = i;
        ++sectionIndex;
      }
    }
    for(int i = 1; i < 4; ++i) {
      int gy = top + height * i / 4;
      out << "<line x1='" << left << "' y1='" << gy << "' x2='" << left + width << "' y2='" << gy
          << "' stroke='#ddd'/>\n";
    }
    out << "<text x='15' y='" << top + 20 << "' font-size='14'>" << escapeXml(title) << "</text>\n";
    out << "<text x='78' y='" << top + 12 << "' text-anchor='end' font-size='11'>" << std::fixed
        << std::setprecision(1) << maxValue << "</text>\n";
    out << "<text x='78' y='" << top + height << "' text-anchor='end' font-size='11'>" << minValue
        << "</text>\n";

    std::string previousId = rows.front().id;
    for(const auto& row : rows) {
      if(row.id != previousId) {
        out << "<line x1='" << x(row.time) << "' y1='" << top << "' x2='" << x(row.time) << "' y2='"
            << top + height << "' stroke='#666' stroke-dasharray='5,4'/>\n";
        previousId = row.id;
      }
    }

    int legendX = left + 10;
    for(const auto& item : series) {
      out << "<polyline fill='none' stroke='" << item.color << "' stroke-width='1.4' points='";
      for(const auto& row : rows) {
        double value = row.values.at(item.column) * (item.negate ? -1.0 : 1.0);
        if(std::isfinite(value)) out << x(row.time) << "," << y(value) << " ";
      }
      out << "'/>\n";
      out << "<line x1='" << legendX << "' y1='" << top + height - 8 << "' x2='" << legendX + 22
          << "' y2='" << top + height - 8 << "' stroke='" << item.color
          << "' stroke-width='2'/><text x='" << legendX + 26 << "' y='" << top + height - 4
          << "' font-size='11'>" << escapeXml(item.label) << "</text>\n";
      legendX += 125;
    }
  }

  void writeSvg(const std::vector<Row>& rows, const std::string& outputPath)
  {
    std::ofstream out(outputPath);
    if(!out) throw std::runtime_error("SVGを書き込めません: " + outputPath);

    double errorSum = 0.0;
    double saturationSum = 0.0;
    for(const auto& row : rows) {
      errorSum += std::abs(row.values.at("error"));
      saturationSum += row.values.at("maxoutActive");
    }
    const double mae = errorSum / rows.size();
    const double saturationRate = saturationSum / rows.size() * 100.0;

    out << "<svg xmlns='http://www.w3.org/2000/svg' width='1240' height='900' "
           "viewBox='0 0 1240 900'>\n"
        << "<rect width='100%' height='100%' fill='#f7f7f7'/>\n"
        << "<g font-family='Noto Sans CJK JP, Yu Gothic, Meiryo, sans-serif' fill='#222'>\n"
        << "<text x='620' y='30' text-anchor='middle' font-size='20'>ライントレース調整: "
           "平均絶対偏差="
        << std::fixed << std::setprecision(2) << mae
        << ", マックスアウト到達率=" << std::setprecision(1) << saturationRate << "%</text>\n";

    const double startTime = rows.front().time;
    drawPanel(out, rows, startTime, 75, "輝度 [%]",
              { { "brightness", "現在輝度", "#1f77b4" }, { "target", "目標輝度", "#ff7f0e" } });
    drawPanel(out, rows, startTime, 280, "輝度偏差 [%]", { { "error", "偏差", "#d62728" } });
    drawPanel(out, rows, startTime, 485, "旋回パワー",
              { { "rawTurn", "制限前", "#999999" },
                { "turn", "制限後", "#2ca02c" },
                { "maxoutPower", "+上限", "#d62728" },
                { "maxoutPower", "-上限", "#d62728", true } });
    drawPanel(
        out, rows, startTime, 690, "モーターパワー",
        { { "rightPower", "右モーター", "#9467bd" }, { "leftPower", "左モーター", "#17becf" } });
    out << "<text x='1185' y='890' text-anchor='end' font-size='12'>経過時間: "
        << (rows.back().time - startTime) / 1000.0 << " 秒</text></g></svg>\n";
  }

}  // namespace

int main(int argc, char* argv[])
{
  if(argc < 2 || argc > 3) {
    std::cerr << "使い方: " << argv[0] << " runlog.csv [output.svg]\n";
    return 1;
  }
  const std::string outputPath = argc == 3 ? argv[2] : "runlog_plot.svg";
  try {
    writeSvg(loadRows(argv[1]), outputPath);
    std::cout << "グラフを出力しました: " << outputPath << std::endl;
    return 0;
  } catch(const std::exception& error) {
    std::cerr << error.what() << std::endl;
    return 1;
  }
}
