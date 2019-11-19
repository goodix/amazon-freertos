#ifndef __DBG_DEFINE_H__
#define __DBG_DEFINE_H__

#include "custom_config.h"
#include <stdio.h>

#ifdef CFG_DBG_PRINTF

#define DEBUG_PRINTF_BASE(format, ...)        printf(format, ##__VA_ARGS__)
#define DEBUG_PRINTF(format, ...)             DEBUG_PRINTF_BASE(format, ##__VA_ARGS__)

#ifdef CFG_APP_PRINTF
#define APP_PRINTF(format, ...)               DEBUG_PRINTF_BASE("[APP]"format"\r\n", ##__VA_ARGS__)
#else
#define APP_PRINTF(format, ...)
#endif

#else   // CFG_DBG_PRINTF

#define DEBUG_PRINTF_BASE(type, format, ...)
#define DEBUG_PRINTF(format, ...)

#define APP_PRINTF(format, ...)

#endif // CFG_DBG_PRINTF

#endif // __DBG_DEFINE_H__
