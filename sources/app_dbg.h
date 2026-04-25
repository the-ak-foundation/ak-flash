#ifndef __APP_DBG_H__
#define __APP_DBG_H__

#include <stdint.h>

/* Log info */
#define APP_DBG_EN          1
#define LOG_TIME_EN		0
#define LOG_FUNCTION_EN 0
#define LOG_LINE_EN		0
#define LOG_LEVEL_EN	1

#if APP_DBG_EN
#define APP_DBG(level, format, ...) \
	do { \
	app_dbg_print(level, __func__, __LINE__, format, ##__VA_ARGS__); \
	} while (0)
#else
#define APP_DBG(level, format, ...)
#endif

enum def_debug_level {
	DEBUG_LEVEL_NONE = 0,
	DEBUG_LEVEL_ERROR, // Log only error messages
	DEBUG_LEVEL_WARN,  // Log warning and error messages
	DEBUG_LEVEL_INFO,  // Log info messages
	DEBUG_LEVEL_DEBUG  //  All debug log error, warn, info
};

void app_dbg_set_level(enum def_debug_level level);
void app_dbg_print(enum def_debug_level level, const char* function, int line, const char* format, ...);

#endif /* __APP_DBG_H__ */
