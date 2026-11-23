/** @file host_check.h
 *
 * @brief Tiny PASS/FAIL helper shared by the PC host tests.
 */

#ifndef HOST_CHECK_H
#define HOST_CHECK_H

#include <stdio.h>

static int g_host_pass = 0;
static int g_host_fail = 0;

#define HOST_CHECK(id, cond, desc)                                         \
    do                                                                     \
    {                                                                      \
        int b_ok_ = (cond) ? 1 : 0;                                        \
        if (b_ok_) { g_host_pass++; } else { g_host_fail++; }              \
        printf("[HOST] %-8s %s  %s\n", (id), b_ok_ ? "PASS" : "FAIL",     \
               (desc));                                                    \
    } while (0)

#define HOST_SUMMARY()                                                     \
    (printf("[HOST] SUMMARY %d passed, %d failed\n", g_host_pass,          \
            g_host_fail), (0 == g_host_fail) ? 0 : 1)

#endif /* HOST_CHECK_H */
