#include "lib/CSVLogFile.h"

template <typename Value, typename... Values>
CSVLogFile::CSVLogFile(wpi::StringRef filePrefix, Value columnHeading, Values... columnHeadings): m_logFile(filePrefix, "csv") {
m_logFile << "\"Timestamp (ms)\",";
LogValues(columnHeading, columnHeadings...);
}

template <typename Value, typename... Values>
void CSVLogFile::Log(Value value, Values... values) {
    using namespace std::chrono;
    auto timestamp =
            duration_cast<milliseconds>(system_clock::now().time_since_epoch());
    m_logFile << timestamp.count() << ',';

    LogValues(value, values...);
}

std::string CSVLogFile::GetFileName() { return m_logFile.GetFileName(); }

template <typename Value, typename... Values>
void CSVLogFile::LogValues(Value value, Values... values) {
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

std::string CSVLogFile::EscapeDoubleQuotes(wpi::StringRef text) {
    std::string textString = text.str();
    for (std::string::size_type i = 0; i < text.size(); i++) {
        if (text[i] == '\"') {
            i++;
            textString.insert(i, "\"");
        }
    }
    return textString;
}