/**
 * @file logging.hpp
 * @author melektron (matteo@elektron.work)
 * @brief logging functions for printing via built-in logging channel
 * @version 0.1
 * @date 2025-02-07
 * @copyright melektron (c) 2025 - now
 */

#pragma once

#include <stdint.h>
#include <stdlib.h>

namespace logging
{
    int info(const char *_fmt, ...);
    int warn(const char *_fmt, ...);
    int error(const char *_fmt, ...);
    int verb(const char *_fmt, ...);
    void binary(const uint8_t * const _data, size_t _n);
    void flush();
} // namespace logging
 