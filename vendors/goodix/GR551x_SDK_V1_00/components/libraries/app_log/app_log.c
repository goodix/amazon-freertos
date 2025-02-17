/**
 *****************************************************************************************
 *
 * @file app_log.c
 *
 * @brief App Log Implementation.
 *
 *****************************************************************************************
 * @attention
  #####Copyright (c) 2019 GOODIX
  All rights reserved.
 *****************************************************************************************
 */

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "app_log.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

/*
 * DEFINE
 *****************************************************************************************
 */
#if APP_LOG_COLOR_ENABLE
/**@brief CSI(Control Sequence Introducer/Initiator) sign more information on https://en.wikipedia.org/wiki/ANSI_escape_code. */
#define CSI_START                      "\033["
#define CSI_END                        "\033[0m"

/**@brief Output log front color */
#define F_BLACK                        "30;"
#define F_RED                          "31;"
#define F_GREEN                        "32;"
#define F_YELLOW                       "33;"
#define F_BLUE                         "34;"
#define F_MAGENTA                      "35;"
#define F_CYAN                         "36;"
#define F_WHITE                        "37;"

/**@brief Output log background color */
#define B_NULL
#define B_BLACK                        "40;"
#define B_RED                          "41;"
#define B_GREEN                        "42;"
#define B_YELLOW                       "43;"
#define B_BLUE                         "44;"
#define B_MAGENTA                      "45;"
#define B_CYAN                         "46;"
#define B_WHITE                        "47;"

/**@brief Output log fonts style */
#define S_BOLD                         "1m"
#define S_UNDERLINE                    "4m"
#define S_BLINK                        "5m"
#define S_NORMAL                       "22m"

/**@brief Output log default color definition: [front color] + [background color] + [show style] */
#ifndef APP_LOG_COLOR_ERROR
#define APP_LOG_COLOR_ERROR               (F_YELLOW B_NULL S_NORMAL)
#endif

#ifndef APP_LOG_COLOR_WARNING
#define APP_LOG_COLOR_WARNING             (F_CYAN B_NULL S_NORMAL)
#endif

#ifndef APP_LOG_COLOR_INFO
#define APP_LOG_COLOR_INFO                (F_GREEN B_NULL S_NORMAL)
#endif

#ifndef APP_LOG_COLOR_DEBUG
#define APP_LOG_COLOR_DEBUG               (F_WHITE B_NULL S_NORMAL)
#endif

#endif

/*
 * STRUCTURES
 *****************************************************************************************
 */
/**@brief App log environment variable. */
struct app_log_env_t
{
    app_log_init_t       app_log_init;  /**< App log initialization variables. */
    bool                 is_filter_set; /**< App log filter is set or not. */
    app_log_trans_func_t trans_func;    /**< App log transmit function. */
    app_log_flush_func_t flush_func;    /**< App log flush function. */
};

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static uint8_t     s_log_encode_buf[APP_LOG_LINE_BUF_SIZE];  /**< App log data encode buffer. */

static const char *s_log_svt_lvl_output_info[] =            /**< App log severity level outpout information. */
{
    [APP_LOG_LVL_ERROR]   = "APP_E: ",
    [APP_LOG_LVL_WARNING] = "APP_W: ",
    [APP_LOG_LVL_INFO]    = "APP_I: ",
    [APP_LOG_LVL_DEBUG]   = "APP_D: ",
};

#if APP_LOG_COLOR_ENABLE
static const char *s_log_color_output_info[] =             /**< App log level outpout color information. */
{
    [APP_LOG_LVL_ERROR]   = APP_LOG_COLOR_ERROR,
    [APP_LOG_LVL_WARNING] = APP_LOG_COLOR_WARNING,
    [APP_LOG_LVL_INFO]    = APP_LOG_COLOR_INFO,
    [APP_LOG_LVL_DEBUG]   = APP_LOG_COLOR_DEBUG,
};
#endif

static struct app_log_env_t  s_app_log_env;                  /**< App log environment variable. */

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief App log string copy.
 *
 * @param[in] wr_idx:     Write index of app log buffer.
 * @param[in] p_log_buff: Pointer to app log cache buffer.
 * @param[in] p_log_data: Pointer to app log data.
 *
 * @return Length of copy.
 *****************************************************************************************
 */
static uint16_t app_log_strcpy(uint16_t wr_idx, uint8_t *p_log_buff, const char *p_log_data)
{
    uint16_t cpy_length = 0;

    if (!p_log_buff || !p_log_data)
    {
        return cpy_length;
    }

    while (*p_log_data != 0)
    {
        if ((wr_idx + cpy_length) < APP_LOG_LINE_BUF_SIZE)
        {
            p_log_buff[wr_idx + cpy_length] = *p_log_data++;
            cpy_length++;
        }
        else
        {
            break;
        }
    }

    return cpy_length;
}

/**
 *****************************************************************************************
 * @brief Check app log format is set or not.
 *
 * @param[in] level: App log level.
 * @param[in] fmt:   Format.
 *
 * @return Result of check.
 *****************************************************************************************
 */
static bool app_log_is_fmt_set(uint8_t level, uint8_t fmt)
{
    if (s_app_log_env.app_log_init.fmt_set[level] & fmt)
    {
        return true;
    }
    else
    {
        return false;
    }

}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
bool app_log_init(app_log_init_t *p_log_init, app_log_trans_func_t trans_func, app_log_flush_func_t flush_func)
{
    if ( NULL == trans_func)
    {
        return false;
    }

    if (NULL == p_log_init)
    {
        s_app_log_env.is_filter_set = false;
        memset(&s_app_log_env.app_log_init, 0, sizeof(app_log_init_t));
    }
    else if ( p_log_init->filter.level <= APP_LOG_LVL_DEBUG)
    {
        s_app_log_env.is_filter_set = true;
        memcpy(&s_app_log_env.app_log_init, p_log_init, sizeof(app_log_init_t));
    }
    else
    {
        return false;
    }

    s_app_log_env.trans_func    = trans_func;
    s_app_log_env.flush_func    = flush_func;

    return true;
}

void app_log_output(uint8_t level, const char *tag, const char *file, const char *func, const long line, const char *format, ...)
{
    uint16_t log_length     = 0;
    uint8_t  newline_length = strlen(APP_LOG_NEWLINE_SIGN);
    int      fmt_result     = 0;
    char     line_num[APP_LOG_LINE_NB_LEN_MAX + 1]  = { 0 };
    va_list  ap;

    if (level > s_app_log_env.app_log_init.filter.level && s_app_log_env.is_filter_set)
    {
        return;
    }

#if APP_LOG_TAG_ENABLE
    if (!strstr(tag, s_app_log_env.app_log_init.filter.tag))
    {
        return;
    }
#endif

    va_start(ap, format);

    APP_LOG_LOCK();

#if APP_LOG_COLOR_ENABLE
    // Encode CSI start sign and color info.
    log_length += app_log_strcpy(log_length, s_log_encode_buf, CSI_START);
    log_length += app_log_strcpy(log_length, s_log_encode_buf, s_log_color_output_info[level]);
#endif

    // Encode level info.
    if (app_log_is_fmt_set(level, APP_LOG_FMT_LVL))
    {
        log_length += app_log_strcpy(log_length, s_log_encode_buf, s_log_svt_lvl_output_info[level]);
    }

#if APP_LOG_TAG_ENABLE
    // Encode tag info.
    if (app_log_is_fmt_set(level, APP_LOG_FMT_TAG))
    {
        log_length += app_log_strcpy(log_length, s_log_encode_buf, tag);
        log_length += app_log_strcpy(log_length, s_log_encode_buf, " ");
    }
#endif

    // Encode file directory name , function name and lune number info.
    if (app_log_is_fmt_set(level, APP_LOG_FMT_DIR | APP_LOG_FMT_FUNC | APP_LOG_FMT_LINE))
    {
        log_length += app_log_strcpy(log_length, s_log_encode_buf, "(");

        if (app_log_is_fmt_set(level, APP_LOG_FMT_DIR))
        {
            log_length += app_log_strcpy(log_length, s_log_encode_buf, file);

            if (app_log_is_fmt_set(level, APP_LOG_FMT_FUNC))
            {
                log_length += app_log_strcpy(log_length, s_log_encode_buf, " ");
            }
            else if (app_log_is_fmt_set(level, APP_LOG_FMT_LINE))
            {
                log_length += app_log_strcpy(log_length, s_log_encode_buf, ":");
            }
        }

        if (app_log_is_fmt_set(level, APP_LOG_FMT_FUNC))
        {
            log_length += app_log_strcpy(log_length, s_log_encode_buf, func);

            if (app_log_is_fmt_set(level, APP_LOG_FMT_LINE))
            {
                log_length += app_log_strcpy(log_length, s_log_encode_buf, " Line:");
            }
        }

        if (app_log_is_fmt_set(level, APP_LOG_FMT_LINE))
        {
            snprintf(line_num, APP_LOG_LINE_NB_LEN_MAX, "%ld", line);
            log_length += app_log_strcpy(log_length, s_log_encode_buf, line_num);
        }

        log_length += app_log_strcpy(log_length, s_log_encode_buf, ") ");
    }

    // Encode other log data to buffer. '\0' must be added in the end by vsnprintf. */
    fmt_result = vsnprintf((char *)s_log_encode_buf + log_length, APP_LOG_LINE_BUF_SIZE - log_length, format, ap);

    va_end(ap);

    //  Calculate log length
    if ((fmt_result > -1) && (log_length + fmt_result) <= APP_LOG_LINE_BUF_SIZE)
    {
        log_length += fmt_result;
    }
    else
    {
        log_length = APP_LOG_LINE_BUF_SIZE;
    }

#if APP_LOG_COLOR_ENABLE
    if (log_length + (sizeof(CSI_END) - 1) + newline_length > APP_LOG_LINE_BUF_SIZE)
    {
        log_length  = APP_LOG_LINE_BUF_SIZE;
        // Reserve some space for CSI end sign.
        log_length -= sizeof(CSI_END) - 1;
#else
    if (log_length + newline_length > APP_LOG_LINE_BUF_SIZE)
    {
        log_length = APP_LOG_LINE_BUF_SIZE;
#endif
        log_length -= newline_length;
    }

#if APP_LOG_COLOR_ENABLE
    // Encode CSI end sign.
    log_length += app_log_strcpy(log_length, s_log_encode_buf, CSI_END);
#endif

    // Encode newline sign.
    log_length += app_log_strcpy(log_length, s_log_encode_buf, APP_LOG_NEWLINE_SIGN);

    s_app_log_env.trans_func(s_log_encode_buf, log_length);

    APP_LOG_UNLOCK();
}

void app_log_raw_info(const char *format, ...)
{
    int      fmt_result = 0;
    uint16_t log_length = 0;
    va_list  ap;

    va_start(ap, format);

    APP_LOG_LOCK();

    fmt_result = vsnprintf((char *)s_log_encode_buf, APP_LOG_LINE_BUF_SIZE, format, ap);

    if ((fmt_result > -1) && (fmt_result) <= APP_LOG_LINE_BUF_SIZE)
    {
        log_length = fmt_result;
    }
    else
    {
        log_length = APP_LOG_LINE_BUF_SIZE;
    }

    s_app_log_env.trans_func(s_log_encode_buf, log_length);

    APP_LOG_UNLOCK();
		app_log_flush();
}

void app_log_hex_dump(uint8_t *p_data, uint16_t length)
{
    uint16_t log_length  = 0;
    uint16_t convert_idx = 0;
    char     dump_str[8] = {0};

    APP_LOG_LOCK();

    for (convert_idx = 0; convert_idx < length; convert_idx++)
    {
        if (log_length >= APP_LOG_LINE_BUF_SIZE)
        {
            log_length = APP_LOG_LINE_BUF_SIZE;
            break;
        }

        if (p_data[convert_idx] < ' ')
        {
            s_log_encode_buf[log_length] = '.';
            log_length++;
        }
        else
        {
            snprintf(dump_str, 8, "%02X ", p_data[convert_idx]);
            log_length += app_log_strcpy(log_length, s_log_encode_buf, dump_str);
        }
    }

    s_app_log_env.trans_func(s_log_encode_buf, log_length);

    APP_LOG_UNLOCK();
}

void app_log_flush(void)
{
    if (s_app_log_env.flush_func)
    {
        s_app_log_env.flush_func();
    }
}

#if IO_REDIRECT == 0
#if defined(__CC_ARM)

struct __FILE
{
    int handle;
};

FILE __stdout;
FILE __stdin;

int fputc(int ch, FILE *file)
{
    s_app_log_env.trans_func((uint8_t *)&ch, 1);

    return 1;
}

#elif defined(__GNUC__)

int _write(int file, const char *buf, int len)
{
    int tx_len = 0;

    while (tx_len < len)
    {
        s_app_log_env.trans_func((uint8_t *)buf, 1);
        buf++;
        tx_len++;
    }
    return tx_len;
}

#elif defined(__ICCARM__)

size_t __write(int handle, const unsigned char *buf, size_t size)
{
    size_t len = 0;

    while (len < size)
    {
        s_app_log_env.trans_func((uint8_t *)buf, 1);
        buf++;
        len++;
    }
    return len;
}

#endif /* defined(__CC_ARM) */
#endif

