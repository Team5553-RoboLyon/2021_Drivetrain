/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <algorithm>
#include <chrono>
#include <string>
#include <type_traits>

#include <wpi/StringRef.h>

#include "lib/LogFile.h"

/**
 * A CSVLogFile writes values to a csv file
 *
 * For the CSVLogFile to write log informations, you must call Log()
 * periodically.
 */
class CSVLogFile {
 public:
  /**
   * Instantiate a LogFile passing in its prefix and its column headings.
   *
   * If you want the file to be saved in a existing directory, you can add
   * its path before the file prefix. Exemple : to save the file in a usb stick
   * on the roborio ("/media/sda1/") : LogFile("/media/sda1/log").
   *
   * @param filePrefix     The prefix of the LogFile.
   * @param columnHeading  Title of 1st CSVLogFile column.
   * @param columnHeadings Titles of other CSVLogFile columns.
   */
  template <typename Value, typename... Values>
    CSVLogFile(wpi::StringRef filePrefix, Value columnHeading,
             Values... columnHeadings)
      : m_logFile(filePrefix, "csv") {
    m_logFile << "\"Timestamp (ms)\",";
    LogValues(columnHeading, columnHeadings...);
  }
  /**
   * Print a new line of values in the CSVLogFile.
   *
   * @param value 1st value to log in the file.
   * @param values Other values to log in the file in the order.
   */
  template <typename Value, typename... Values>
  void Log(Value value, Values... values);

  /**
   * Get the name the file.
   *
   * @return The name of the file.
   */
  const std::string GetFileName() const;

 private:
  /**
   * Print a new line of values in the CSVLogFile without timestamp.
   *
   * @param value 1st value to log in the file.
   * @param values Other values to log in the file in the order.
   */
  template <typename Value, typename... Values>
  void LogValues(Value value, Values... values);

  /**
   * Escape double quotes in a text by duplicating them.
   *
   * @param text Text to escape.
   * @return The text with all its double quotes escaped.
   */
  const std::string EscapeDoubleQuotes(wpi::StringRef text) const;

  LogFile m_logFile;
};
