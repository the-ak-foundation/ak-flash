#include "app_dbg.h"
#include <stdarg.h>
#include <stdio.h>
#include <time.h>

static enum def_debug_level current_debug_level = DEBUG_LEVEL_INFO;

static const char* debug_level_to_string(enum def_debug_level level) {
	switch (level) {
	case DEBUG_LEVEL_ERROR:
		return "ERROR";
	case DEBUG_LEVEL_WARN:
		return "WARN";
	case DEBUG_LEVEL_INFO:
		return "INFO";
	case DEBUG_LEVEL_DEBUG:
		return "DEBUG";
	default:
		return "UNKNOWN";
	}
}

static const char* debug_level_to_color(enum def_debug_level level) {
	switch (level) {
	case DEBUG_LEVEL_ERROR:
		return "\x1b[31m";
	case DEBUG_LEVEL_WARN:
		return "\x1b[33m";
	case DEBUG_LEVEL_INFO:
		return "\x1b[32m";
	case DEBUG_LEVEL_DEBUG:
		return "\x1b[36m";
	default:
		return "\x1b[0m";
	}
}

void app_dbg_set_level(enum def_debug_level level) {
	current_debug_level = level;
}

void app_dbg_print(enum def_debug_level level, const char* function, int line, const char* format, ...) {
	(void)function;
	(void)line;

	if (level <= DEBUG_LEVEL_NONE || level > DEBUG_LEVEL_DEBUG) {
		return;
	}

	if (current_debug_level < level) {
		return;
	}

	if (format != NULL) {
		if ((format[0] == '\n' && format[1] == '\0') || format[0] == '\r') {
			va_list raw_args;
			va_start(raw_args, format);
			vprintf(format, raw_args);
			va_end(raw_args);
			fflush(stdout);
			return;
		}
	}

	fputs(debug_level_to_color(level), stdout);

#if LOG_TIME_EN
	{
		time_t now = time(NULL);
		struct tm* local_time = localtime(&now);
		if (local_time != NULL) {
			printf("[%02d:%02d:%02d] ", local_time->tm_hour, local_time->tm_min, local_time->tm_sec);
		}
	}
#endif

#if LOG_LEVEL_EN
	printf("[%s] ", debug_level_to_string(level));
#endif

#if LOG_FUNCTION_EN && LOG_LINE_EN
	printf("[%s:%d] ", function, line);
#elif LOG_FUNCTION_EN
	printf("[%s] ", function);
#elif LOG_LINE_EN
	printf("[line:%d] ", line);
#endif

	fputs("\x1b[0m", stdout);
	va_list args;
	va_start(args, format);
	vprintf(format, args);
	va_end(args);

	fputs("\x1b[0m", stdout);
}
