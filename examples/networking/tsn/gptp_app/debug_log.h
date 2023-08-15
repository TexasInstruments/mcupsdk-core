#ifndef __DEBUG_LOG_H__
#define __DEBUG_LOG_H__

typedef void (*on_console_out)(const char *str, ...);

#if defined(SITARA)
#define LINE_FEED "\r\n"
#else
#define LINE_FEED "\n"
#endif

extern on_console_out s_drv_console_out;
#define DPRINT(str,...) s_drv_console_out(str, ##__VA_ARGS__)

int log_to_buffer(bool flush, const char *str);
int direct_log(bool flush, const char *str);
#ifdef TSN_USE_LOG_BUFFER
#define LOG_OUTPUT log_to_buffer
#else
#define LOG_OUTPUT direct_log
#endif

int debug_log_init(on_console_out console_out);
void debug_log_deinit(void);

#endif
