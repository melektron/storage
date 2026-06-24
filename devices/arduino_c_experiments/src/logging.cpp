/**
 * @file logging.cpp
 * @author melektron (matteo@elektron.work)
 * @brief logging functions for printing via built-in logging channel
 * @version 0.1
 * @date 2025-02-07
 * @copyright melektron (c) 2025 - now
 */

#include <stdint.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <Arduino.h>
#include <el/ansi_colors.h>
#include "logging.hpp"
#include <stdio.h>


int logging::info(const char *_fmt, ...)
{
    va_list args;
    va_start(args, _fmt);
    fputs("I: ", stdout);
    int ret = vprintf(_fmt, args);
    puts(EL_ANSI_RESET);
    va_end(args);
    return ret;
}

int logging::warn(const char *_fmt, ...)
{
    va_list args;
    va_start(args, _fmt);
    fputs(EL_ANSI_YELLOW "W: ", stdout);
    int ret = vprintf(_fmt, args);
    puts(EL_ANSI_RESET);
    va_end(args);
    return ret;
}

int logging::error(const char *_fmt, ...)
{
    va_list args;
    va_start(args, _fmt);
    fputs(EL_ANSI_RED "E: ", stdout);
    int ret = vprintf(_fmt, args);
    puts(EL_ANSI_RESET);
    va_end(args);
    return ret;
}

int logging::verb(const char *_fmt, ...)
{
    va_list args;
    va_start(args, _fmt);
    fputs(EL_ANSI_PURPLE "V: ", stdout);
    int ret = vprintf(_fmt, args);
    puts(EL_ANSI_RESET);
    va_end(args);
    return ret;
}

void logging::binary(const uint8_t * const _data, size_t _n)
{
    static constexpr size_t BYTES_PER_LINE = 8;
    const uint8_t *read_ptr = _data;

    fputs(EL_ANSI_GREEN "B: \e[4m   +\xE2\x96\x95", stdout);
    size_t lines = _n / BYTES_PER_LINE + 1;
    for (size_t b = 0; b < BYTES_PER_LINE; b++)
    {
        printf(" %2x", b);
    }
    puts("\e[24m");

    for (size_t l = 0; l < lines; l++)
    {
        printf("   %+4x\xE2\x96\x95", (read_ptr - _data));
        for (size_t b = 0; b < BYTES_PER_LINE; b++)
        {
            if (read_ptr >= (_data + _n))
                goto done;

            printf("%+3x", *read_ptr);
            read_ptr++;
        }
        puts("");
    }
done:
    puts(EL_ANSI_RESET);

}

void logging::flush()
{
    fflush(stdout);
}