
#ifndef LOG_HPP
#define LOG_HPP

/// @brief Simple logging functions for the simulator
/// @param tag Tag string to identify the log source
/// @param format printf-style format string
/// @param ... Additional arguments for the format string

void log(const char* tag, const char* format, ...); 
/// @brief Simple warning logging functions for the simulator
/// @param tag Tag string to identify the log source
/// @param format printf-style format string
/// @param ... Additional arguments for the format string
void logw(const char* tag, const char* format, ...); 

#endif // LOG_HPP