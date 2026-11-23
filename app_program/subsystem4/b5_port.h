/**
 * @file b5_port.h
 * @brief Thin OS / SDK abstraction: delay, microsecond clock, critical section.
 *
 * Keeps every RTOS-specific call in one place so the rest of Buddy 5 is
 * portable and the pure logic can be unit-tested on a PC (B5_HOST_TEST).
 *
 * IS25 integration (26 Oct): the team firmware does not link the Pico SDK
 * hardware libraries, so the target branch uses micro T-Kernel only:
 * tk_dly_tsk, DI/EI and the RP2040 1 MHz timer register (TIMERAWL).
 */
#ifndef B5_PORT_H
#define B5_PORT_H

#include <stdint.h>
#include "b5_config.h"

#ifdef B5_HOST_TEST

typedef uint32_t B5_Irq_t;
static inline B5_Irq_t B5_Port_Enter(void) { return 0u; }
static inline void B5_Port_Exit(B5_Irq_t state) { (void)state; }
static inline void B5_Port_DelayMs(uint32_t ms) { (void)ms; }
static inline uint32_t B5_Port_NowUs(void) { return 0u; }
#define B5_LOG(fmt, ...) ((void)0)

#else /* target: micro T-Kernel on the RP2040 */

#include <tk/tkernel.h>
#include <tm/tmonitor.h>

/* IS25 (17 Nov): one console line per obstacle step, so a car run shows
 * where the avoidance stops.  tm_printf never waits for the USB host. */
#define B5_LOG(fmt, ...) ((void)tm_printf((UB *)(fmt), __VA_ARGS__))

typedef UINT B5_Irq_t;

/* Masks interrupts on THIS core only.  Every B5 shared word has a single
 * writer (TSK_SCAN), so this only keeps the masked read-modify-writes of
 * the status word whole against the echo interrupt on the same core. */
static inline B5_Irq_t B5_Port_Enter(void)
{
    UINT intsts;
    DI(intsts);
    return intsts;
}

static inline void B5_Port_Exit(B5_Irq_t state)
{
    UINT intsts = state;
    EI(intsts);
}

/* Blocks only the calling task; other tasks keep running. */
static inline void B5_Port_DelayMs(uint32_t ms)
{
    (void)tk_dly_tsk((RELTIM)ms);
}

/* RP2040 free-running 1 MHz timer, low 32 bits. */
static inline uint32_t B5_Port_NowUs(void)
{
    return (uint32_t)in_w(TIMER_TIMERAWL);
}

/* Short busy wait (trigger pulse), shorter than one kernel tick. */
static inline void B5_Port_BusyWaitUs(uint32_t us)
{
    const uint32_t start = B5_Port_NowUs();

    while ((B5_Port_NowUs() - start) < us)
    {
        /* spin */
    }
}

#endif /* B5_HOST_TEST */

#endif /* B5_PORT_H */
