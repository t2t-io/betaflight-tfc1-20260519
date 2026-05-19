/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "build/debug_pin.h"
#include "build/debug_print.h"

#define MAX_LOG_BUFFER_SIZE (1024)

// filepath, like `./src/main/sensors/initialisation.c`
#define FILEPATH_PREFIX (6) // Remove "./src/" prefix from file path for better readability in logs, adjust if the prefix changes

extern int stdio_printf(const char *format, ...);

static char m_buffer[MAX_LOG_BUFFER_SIZE];

void debugInit(void) 
{
    // NOOP
}

void ttlog(const int level, const char *filepath, const int lineno, const char *fmt, ...) 
{
    va_list args;
    const char *filename = strlen(filepath) > FILEPATH_PREFIX ? filepath + FILEPATH_PREFIX : filepath;
    va_start(args, fmt);
    memset(m_buffer, 0, sizeof(m_buffer));
    vsnprintf(m_buffer, sizeof(m_buffer), fmt, args);
    va_end(args);
    stdio_printf(CONSOLE_PREFIX_CHAR_MSG CONSOLE_MESSAGE_SEPARATOR
                "%d" CONSOLE_MESSAGE_SEPARATOR
                "%s:%d" CONSOLE_MESSAGE_SEPARATOR,
                level, filename, lineno);
    stdio_printf("%s\n", m_buffer);
}
