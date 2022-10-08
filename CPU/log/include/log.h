#pragma once

#include <nanoprintf.h>

#ifdef __cplusplus
extern "C" {
#endif

void loggingInit(void);

int DEBUG(const char *fmt, ...);
int INFO(const char *fmt, ...);
int WARN(const char *fmt, ...);
int ERROR(const char *fmt, ...);

void putchar_(int c, void *ctx = NULL);

#ifdef __cplusplus
}
#endif