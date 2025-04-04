#pragma once

#include "Parameters.h"

/**
 * @brief Set up the logger with the specified log level
 * 
 * Configures the global logger to use the specified log level.
 * Valid levels are: TRACE, DEBUG, INFO, WARN, ERROR.
 * If an invalid value is provided, the default level INFO will be set.
 * 
 * @param logLevel The log level to set
 */
void setupLogger(const std::string& logLevel);

/**
 * @brief Parse command line parameters
 * 
 * Parses command line arguments and populates the Parameters structure.
 * Handles options for socket path, log level, frequency, and timeout.
 * 
 * @param argc Argument count from main
 * @param argv Argument vector from main
 * @param params Parameters structure to populate
 * @return true if parsing was successful, false otherwise
 */
bool parseParameters(int argc, char* argv[], Parameters& params);