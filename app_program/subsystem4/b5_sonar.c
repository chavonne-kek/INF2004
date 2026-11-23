/**
 * @file b5_sonar.c
 * @brief Ultrasonic ranging.
 *
 * ISR <-> task hand-over uses ONE 16-bit word written by a single store:
 *   bits 0..13  echo pulse width in microseconds (max 16383 us ~ 282 cm)
 *   bits 14..15 handshake state (IDLE / ARMED / DONE)
 * The task arms it, the ISR completes it, the task reads and re-idles it.
 * Each side only ever stores the WHOLE word, so there is no read-modify-write
 * on shared packed data and no race.
 *
 * IS25 integration (26 Oct): Pico SDK GPIO calls replaced by libbsp; the
 * echo edges arrive through subsystem 2's shared GPIO interrupt
 * (subsystem2_gpio_attach); FAULT_NO_ECHO in the team fault mask follows
 * B5_FAULT_NO_ECHO; the last range is kept for TSK_CTRL and telemetry.
 */
#include "b5_sonar.h"
#include "b5_config.h"
#include "b5_port.h"
#include "b5_types.h"
#include "subsystem4.h"
#include "../subsystem2/subsystem2.h"

/* Edge flags, same values as the Pico SDK's GPIO_IRQ_EDGE_*. */
#define GPIO_IRQ_EDGE_FALL  (0x4u)
#define GPIO_IRQ_EDGE_RISE  (0x8u)

#define WATCHDOG_TICK_REG      (0x4005802Cu)
#define WATCHDOG_TICK_ENABLE   (1u << 9)
#define WATCHDOG_TICK_RUNNING  (1u << 10)
#define WATCHDOG_TICK_CYCLES   (12u)    /* clk_ref 12 MHz -> 1 us tick */

#define ECHO_US_BITS        (14u)
#define ECHO_US_MASK        ((1u << ECHO_US_BITS) - 1u)
#define ECHO_STATE_IDLE     (0u)
#define ECHO_STATE_ARMED    (1u)
#define ECHO_STATE_DONE     (2u)
#define US_PER_CM_ROUNDTRIP (58u)
#define TRIGGER_PULSE_US    (10u)

#define ECHO_WORD(state, us)   ((uint16_t)(((state) << ECHO_US_BITS) | ((us) & ECHO_US_MASK)))
#define ECHO_WORD_STATE(w)     ((uint16_t)((w) >> ECHO_US_BITS))
#define ECHO_WORD_US(w)        ((uint16_t)((w) & ECHO_US_MASK))

static volatile uint16_t s_echoWord = ECHO_WORD(ECHO_STATE_IDLE, 0u);

/* ISR-private: written and read only inside the ISR. */
static uint16_t s_riseUs16;
static bool     s_riseSeen;

/* Task-private. 20 consecutive misses -> 5 bits needed; one byte of storage. */
static uint8_t  s_missCount;

/* IS25: last range, written only by TSK_SCAN, read by TSK_CTRL / S5. */
static volatile uint8_t s_lastCm = B5_DIST_NO_ECHO;

/* IS25: time of the last trigger, for the 60 ms measurement cycle. */
static uint32_t s_lastTriggerUs;

void B5_Sonar_OnGpioEdge(uint32_t gpioPin, uint32_t events)
{
    if (gpioPin != B5_PIN_SONAR_ECHO)
    {
        return;
    }

    if ((events & GPIO_IRQ_EDGE_RISE) != 0u)
    {
        s_riseUs16 = (uint16_t)B5_Port_NowUs();
        s_riseSeen = true;
    }

    if (((events & GPIO_IRQ_EDGE_FALL) != 0u) && s_riseSeen &&
        (ECHO_WORD_STATE(s_echoWord) == ECHO_STATE_ARMED))
    {
        /* 16-bit subtraction handles timer wrap; pulses are < 65 ms. */
        uint16_t width = (uint16_t)((uint16_t)B5_Port_NowUs() - s_riseUs16);

        if (width > ECHO_US_MASK)
        {
            width = (uint16_t)ECHO_US_MASK;
        }
        s_echoWord = ECHO_WORD(ECHO_STATE_DONE, width);   /* single store */
        s_riseSeen = false;
    }
}

/* IS25: adapter from subsystem 2's shared GPIO interrupt. */
static void EchoEdge(bool high, uint32_t timeUs)
{
    (void)timeUs;
    B5_Sonar_OnGpioEdge(B5_PIN_SONAR_ECHO,
                        high ? GPIO_IRQ_EDGE_RISE : GPIO_IRQ_EDGE_FALL);
}

/* IS25: the 1 MHz timer counts only if the TIMER block is out of reset and
 * the watchdog tick runs; start them if the kernel has not. */
static void EnsureUsTimer(void)
{
    clr_w(RESETS_RESET, RESETS_RESET_TIMER);
    while ((in_w(RESETS_RESET_DONE) & RESETS_RESET_TIMER) == 0u)
    {
        /* wait */
    }
    if ((in_w(WATCHDOG_TICK_REG) & WATCHDOG_TICK_RUNNING) == 0u)
    {
        out_w(WATCHDOG_TICK_REG, WATCHDOG_TICK_ENABLE | WATCHDOG_TICK_CYCLES);
    }
}

/* Called from subsystem4_init() (phase 1), before the GPIO interrupt is on. */
void B5_Sonar_Init(void)
{
    EnsureUsTimer();

    (void)gpio_set_pin(B5_PIN_SONAR_TRIG, GPIO_MODE_OUT);
    (void)gpio_set_val(B5_PIN_SONAR_TRIG, 0u);
    (void)gpio_set_pin(B5_PIN_SONAR_ECHO, GPIO_MODE_IN);   /* module drives it low */

    (void)subsystem2_gpio_attach(B5_PIN_SONAR_ECHO, EchoEdge);
}

uint8_t B5_Sonar_LastCm(void)
{
    return s_lastCm;
}

static void NoteResult(bool gotEcho)
{
    if (gotEcho)
    {
        if (s_missCount >= B5_SONAR_NOECHO_FAULT_COUNT)
        {
            subsystem4_fault_clear(FAULT_NO_ECHO);   /* IS25: team mask */
        }
        s_missCount = 0u;
        B5_Status_ClearFault(B5_FAULT_NO_ECHO);
    }
    else if (s_missCount < B5_SONAR_NOECHO_FAULT_COUNT)
    {
        s_missCount++;
        if (s_missCount >= B5_SONAR_NOECHO_FAULT_COUNT)
        {
            B5_Status_RaiseFault(B5_FAULT_NO_ECHO);
            subsystem4_fault_raise(FAULT_NO_ECHO);  /* IS25: team mask */
        }
    }
    else
    {
        /* already latched */
    }
}

uint8_t B5_Sonar_MeasureCm(void)
{
    uint16_t word = ECHO_WORD(ECHO_STATE_IDLE, 0u);
    uint8_t result = B5_DIST_NO_ECHO;
    uint32_t waitedMs;

    /* IS25: HC-SR04 datasheet "use over 60 ms measurement cycle" (Week 6
     * NFR7, Tribunal 1): wait until 60 ms after the previous trigger, so a
     * late echo of the last ping is never taken for this one. */
    while ((B5_Port_NowUs() - s_lastTriggerUs) <
           (B5_SONAR_MIN_CYCLE_MS * 1000u))
    {
        B5_Port_DelayMs(1u);
    }

    s_lastTriggerUs = B5_Port_NowUs();
    s_echoWord = ECHO_WORD(ECHO_STATE_ARMED, 0u);

    (void)gpio_set_val(B5_PIN_SONAR_TRIG, 1u);
    B5_Port_BusyWaitUs(TRIGGER_PULSE_US);
    (void)gpio_set_val(B5_PIN_SONAR_TRIG, 0u);

    for (waitedMs = 0u; waitedMs < B5_SONAR_TIMEOUT_MS; waitedMs++)
    {
        B5_Port_DelayMs(1u);
        word = s_echoWord;
        if (ECHO_WORD_STATE(word) == ECHO_STATE_DONE)
        {
            break;
        }
    }
    s_echoWord = ECHO_WORD(ECHO_STATE_IDLE, 0u);

    if (ECHO_WORD_STATE(word) == ECHO_STATE_DONE)
    {
        uint32_t cm = ((uint32_t)ECHO_WORD_US(word) + (US_PER_CM_ROUNDTRIP / 2u)) /
                      US_PER_CM_ROUNDTRIP;

        if (cm < 1u)
        {
            cm = 1u;
        }
        if (cm < B5_DIST_MAX_CM)
        {
            result = (uint8_t)cm;
        }
    }

    /* IS25: a far reading (>= 250 cm) is still an echo; only a missing
     * echo pulse counts toward the NO_ECHO fault (open rooms are normal). */
    NoteResult(ECHO_WORD_STATE(word) == ECHO_STATE_DONE);
    s_lastCm = result;
    B5_Port_DelayMs(B5_SONAR_GUARD_MS);
    return result;
}

static uint8_t Median3(uint8_t a, uint8_t b, uint8_t c)
{
    uint8_t lo = (a < b) ? a : b;
    uint8_t hi = (a < b) ? b : a;

    return (c < lo) ? lo : ((c > hi) ? hi : c);
}

uint8_t B5_Sonar_MeasureMedianCm(uint8_t reps)
{
    uint8_t first = B5_Sonar_MeasureCm();

    if (reps < 3u)
    {
        return first;
    }
    else
    {
        uint8_t second = B5_Sonar_MeasureCm();
        uint8_t third = B5_Sonar_MeasureCm();

        return Median3(first, second, third);   /* NO_ECHO=255 sorts as far */
    }
}
