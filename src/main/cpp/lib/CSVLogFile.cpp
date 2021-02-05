#include "lib/CSVLogFile.h"
#include "lib/LogFile.h"

#include <string>



    void CSVLogFile::Log(Value value, Values... values) {
    using namespace std::chrono;
    auto timestamp =
        duration_cast<milliseconds>(system_clock::now().time_since_epoch());
    m_logFile << timestamp.count() << ',';

    LogValues(value, values...);
  }

    const std::string CSVLogFile::GetFileName() const { return m_logFile.GetFileName(); }

    void CSVLogFile::LogValues(Value value, Values... values){
    if constexpr (std::is_convertible_v<Value, wpi::StringRef>) {
      m_logFile << '\"' << EscapeDoubleQuotes(value) << '\"';
    } else {
      m_logFile << value;
    }

    if constexpr (sizeof...(values) > 0) {
      m_logFile << ',';
      LogValues(values...);
    } else {
      m_logFile << '\n';
    }
    }


  const std::string CSVLogFile::EscapeDoubleQuotes(wpi::StringRef text) const {
    std::string textString = text.str();
    for (std::string::size_type i = 0; i < text.size(); i++) {
      if (text[i] == '\"') {
        i++;
        textString.insert(i, "\"");
      }
    }
    return textString;
  }