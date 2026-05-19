/**
 * Copyright 2022-2026 T2T Inc. All rights reserved.
 */
#pragma once

typedef enum {
  TTLOG_LEVEL_DEBUG = 0,
  TTLOG_LEVEL_INFO,
  TTLOG_LEVEL_WARN,
  TTLOG_LEVEL_ERROR,
} TT_LogLevel;

#if (PICO_TRACE) // Only enable debug print when PICO_TRACE is defined
void ttlog(const int level, const char *filepath, const int lineno,
           const char *fmt, ...);
// #warning "PICO_TRACE is enabled, debug print is active. This may impact performance. Define PICO_TRACE=0 to disable."
#else
#define ttlog(level, module, lineno, fmt, ...)
// #warning "PICO_TRACE is disabled, debug print is inactive. Define PICO_TRACE=1 to enable."
#endif

#define DBG(fmt, ...)                                                          \
  do {                                                                         \
    ttlog(TTLOG_LEVEL_DEBUG, __FILE__, __LINE__, fmt, ##__VA_ARGS__);          \
  } while (0)

#define INFO(fmt, ...)                                                         \
  do {                                                                         \
    ttlog(TTLOG_LEVEL_INFO, __FILE__, __LINE__, fmt, ##__VA_ARGS__);           \
  } while (0)

#define WARN(fmt, ...)                                                         \
  do {                                                                         \
    ttlog(TTLOG_LEVEL_WARN, __FILE__, __LINE__, fmt, ##__VA_ARGS__);           \
  } while (0)

#define ERR(fmt, ...)                                                          \
  do {                                                                         \
    ttlog(TTLOG_LEVEL_ERROR, __FILE__, __LINE__, fmt, ##__VA_ARGS__);          \
  } while (0)

#define CONSOLE_MESSAGE_SEPARATOR "\t"

#define CONSOLE_PREFIX_CHAR_MSG "!"
#define CONSOLE_PREFIX_CHAR_DUMP "^"
#define CONSOLE_PREFIX_CHAR_COMMENT "#"
#define CONSOLE_PREFIX_CHAR_PROPERTY "@"
#define CONSOLE_PREFIX_CHAR_SENSOR_UPDATE "%"