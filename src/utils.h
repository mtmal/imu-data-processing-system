#pragma once

#include "Parameters.h"

/**
 * @brief Function that sets the logging level for the program.
 * @param logLevel The new logging level to set. Possible values are TRACE, DEBUG, INFO, WARN, ERROR.
 *                 If an invalid value is provided, the default level INFO will be set. 
 */
void setupLogger(const std::string& logLevel);

/**
 * @brief Reads parameters from the command line arguments.
 * @param argc The number of command line arguments
 * @param argv The command line arguments
 * @param params The parameters structure to fill
 * @return true if the parameters were parsed successfully, false otherwise
 */
bool parseParameters(int argc, char* argv[], Parameters& params);