/** @file drv_encoder.c
 *
 * @brief Encoder counting and the shared GPIO bank-0 interrupt.
 *
 * The RP2040 has one GPIO interrupt (IO_IRQ_BANK0) per core, so every
 * GPIO edge source shares this handler (Week 6 shared-resource matrix).
 * The handler reads the 1 MHz timer once, then for each attached pin with
 * a pending edge it clears the edge and calls that pin's callback.  It
 * does no other processing.
 *
 * Encoder counters are 16-bit and wrap.  Each is written only by this
 * handler (one writer, one aligned store), and TSK_MOTION reads it once
 * per cycle and takes the unsigned 16-bit difference, which is correct
 * across the wrap (Tribunal 1, Version C).
 *
 * Phase A only (Tribunal 1 decision): direction comes from the commanded
 * direction.  Both signal pins of each encoder's Grove port are watched
 * and the wheel is counted on the pin that carries the pulses (see
 * drv_encoder.h), so the white/yellow wire order does not matter.
 * ENC_USE_PHASE_B (drv_encoder.h) is the fallback if the phase-A test
 * fails: on each phase-A edge, phase B's level gives the direction
 * (quadrature), and the counter counts up or down.
 */

#include "subsystem2.h"
#include "drv_encoder.h"

/* UART0 (CRASH_REPORT=1 or CONSOLE=uart) drives GP0/GP1, which the left
 * encoder uses since 26 Oct.  Stop the build instead of losing a wheel. */
#if defined(TM_CONSOLE_UART) && (TM_CONSOLE_UART != 0) && \
    ((S2_PIN_ENC_L_A <= 1u) || (S2_PIN_ENC_R_A <= 1u) || \
     (S2_PIN_ENC_L_B <= 1u) || (S2_PIN_ENC_R_B <= 1u))
#error "Encoder on GP0/GP1 (Grove 1): build with CRASH_REPORT=0 CONSOLE=usb_cdc"
#endif


#define GPIO_IRQ_NO            13u         /* IO_IRQ_BANK0.                 */
#define GPIO_IRQ_PRIORITY      2
#define GPIO_MAX_ATTACHED      6u          /* 4 encoder pins + HC-SR04 echo. */
#define GPIO_PINS_PER_REG      8u
#define GPIO_BITS_PER_PIN      4u
#define GPIO_EDGE_LOW          0x4u        /* Bit 2 of a pin's nibble.     */
#define GPIO_EDGE_HIGH         0x8u        /* Bit 3 of a pin's nibble.     */
#define GPIO_EDGES             (GPIO_EDGE_LOW | GPIO_EDGE_HIGH)

/* Encoder pins, as indexes into g_pin_edges[]. */
#define ENC_PIN_L_A            0u
#define ENC_PIN_L_B            1u
#define ENC_PIN_R_A            2u
#define ENC_PIN_R_B            3u
#define ENC_PINS               4u

/* A wheel moves to its other pin when that pin has ENC_SWITCH_EDGES more
 * edges than the one in use.  A pin with no encoder on it never leads;
 * with both phases wired the two stay within one edge of each other. */
#define ENC_SWITCH_EDGES       8

/* IO_BANK0 interrupt registers (RP2040 datasheet 2.19.6). */
#define IO_INTR(n)             (IO_BANK0_BASE + 0x0F0u + ((n) * 4u))
#define IO_PROC0_INTE(n)       (IO_BANK0_BASE + 0x100u + ((n) * 4u))
#define IO_PROC0_INTS(n)       (IO_BANK0_BASE + 0x120u + ((n) * 4u))
#define IO_PROC1_INTE(n)       (IO_BANK0_BASE + 0x130u + ((n) * 4u))
#define IO_PROC1_INTS(n)       (IO_BANK0_BASE + 0x150u + ((n) * 4u))
#define SIO_CPUID              (SIO_BASE + 0x000u)
#define TIMER_RAW_LOW          TIMER_TIMERAWL

typedef struct
{
    uint8_t              pin;
    subsystem2_gpio_cb_t cb;
} gpio_slot_t;

static gpio_slot_t       g_slots[GPIO_MAX_ATTACHED];
static uint8_t           g_slot_count = 0u;
static bool              gb_irq_enabled = false;
static uint32_t          g_irq_core     = 0u;     /* 0 = core 0, 1 = core 1. */

static uint8_t const     g_pin_gpio[ENC_PINS] =
{
    S2_PIN_ENC_L_A, S2_PIN_ENC_L_B, S2_PIN_ENC_R_A, S2_PIN_ENC_R_B
};

/* Written by the ISR only (one writer); read by tasks on either core. */
static volatile uint16_t g_count[2] = {0u, 0u};
static volatile uint32_t g_irq_count = 0u;
static volatile uint32_t g_pin_edges[ENC_PINS] = {0u, 0u, 0u, 0u};
static volatile uint8_t  g_active[2] = {ENC_PIN_L_A, ENC_PIN_R_A};

static void gpio_isr(UINT intno);
#if ENC_USE_PHASE_B
static void encoder_left_cb(bool b_high, uint32_t time_us);
static void encoder_right_cb(bool b_high, uint32_t time_us);
static uint16_t step(uint16_t count, bool b_a_high, uint8_t pin_b,
                     bool b_invert);
#else
static void edge_l_a(bool b_high, uint32_t time_us);
static void edge_l_b(bool b_high, uint32_t time_us);
static void edge_r_a(bool b_high, uint32_t time_us);
static void edge_r_b(bool b_high, uint32_t time_us);
static void count_edge(uint8_t wheel, uint8_t pin);
#endif
static void input_pull_up(uint8_t pin);


/*!
 * @brief Configure the encoder inputs and attach them to the handler.
 */
ER
drv_encoder_init (void)
{
    ER      result = E_OK;
    uint8_t index  = 0u;

    for (index = 0u; index < ENC_PINS; index++)
    {
        input_pull_up(g_pin_gpio[index]);
    }

#if ENC_USE_PHASE_B
    result = drv_encoder_attach(S2_PIN_ENC_L_A, encoder_left_cb);

    if (E_OK == result)
    {
        result = drv_encoder_attach(S2_PIN_ENC_R_A, encoder_right_cb);
    }
#else
    {
        static subsystem2_gpio_cb_t const callbacks[ENC_PINS] =
        {
            edge_l_a, edge_l_b, edge_r_a, edge_r_b
        };

        for (index = 0u; (index < ENC_PINS) && (E_OK == result); index++)
        {
            result = drv_encoder_attach(g_pin_gpio[index], callbacks[index]);
        }
    }
#endif

    return (result);
}

/*!
 * @brief Attach a pin to the shared GPIO interrupt (both edges).  Only
 *        before drv_encoder_enable_irq(), i.e. in subsystemN_init().
 */
ER
drv_encoder_attach (uint8_t pin, subsystem2_gpio_cb_t cb)
{
    ER result = E_OK;

    if (gb_irq_enabled)
    {
        result = E_OBJ;         /* Too late: attach in subsystemN_init(). */
    }
    else if ((g_slot_count >= GPIO_MAX_ATTACHED) || (NULL == cb) ||
             (pin >= (uint8_t)GPIO_NUM))
    {
        result = E_PAR;
    }
    else
    {
        g_slots[g_slot_count].pin = pin;
        g_slots[g_slot_count].cb  = cb;
        g_slot_count++;
    }

    return (result);
}

/*!
 * @brief Register the handler and enable both edges of every attached pin
 *        on the CALLING core.  Called by TSK_MOTION (core 1), so encoder
 *        and echo edges are handled on core 1 (the NVIC is per core).
 */
ER
drv_encoder_enable_irq (void)
{
    T_DINT  dint   = {0};
    ER      result = E_OK;
    uint8_t slot   = 0u;

    dint.intatr = TA_HLNG;
    dint.inthdr = (FP)gpio_isr;     /* Kernel handler type for C code. */
    result = tk_def_int(GPIO_IRQ_NO, &dint);

    if (E_OK == result)
    {
        g_irq_core = in_w(SIO_CPUID);

        for (slot = 0u; slot < g_slot_count; slot++)
        {
            uint32_t pin   = g_slots[slot].pin;
            uint32_t reg   = pin / GPIO_PINS_PER_REG;
            uint32_t shift = (pin % GPIO_PINS_PER_REG) * GPIO_BITS_PER_PIN;

            out_w(IO_INTR(reg), GPIO_EDGES << shift);   /* Clear stale. */

            if (0u == g_irq_core)
            {
                set_w(IO_PROC0_INTE(reg), GPIO_EDGES << shift);
            }
            else
            {
                set_w(IO_PROC1_INTE(reg), GPIO_EDGES << shift);
            }
        }

        gb_irq_enabled = true;
        EnableInt(GPIO_IRQ_NO, GPIO_IRQ_PRIORITY);
    }

    return (result);
}

/*!
 * @brief Raw 16-bit edge count of one wheel (wraps).
 */
uint16_t
drv_encoder_count (uint8_t wheel)
{
    return (g_count[(ENC_RIGHT == wheel) ? ENC_RIGHT : ENC_LEFT]);
}

/*!
 * @brief Number of GPIO interrupts handled (diagnostics).
 */
uint32_t
drv_encoder_irq_count (void)
{
    return (g_irq_count);
}

/*!
 * @brief Edges seen on one encoder pin since start-up (diagnostics).
 *
 * @param[in] gpio GPIO number (GP0..GP3).
 *
 * @return Edge count, or 0 if the pin is not an encoder pin.
 */
uint32_t
drv_encoder_pin_edges (uint8_t gpio)
{
    uint32_t edges = 0u;
    uint8_t  index = 0u;

    for (index = 0u; index < ENC_PINS; index++)
    {
        if (gpio == g_pin_gpio[index])
        {
            edges = g_pin_edges[index];
        }
    }

    return (edges);
}

/*!
 * @brief GPIO number a wheel is being counted on (diagnostics).
 */
uint8_t
drv_encoder_active_gpio (uint8_t wheel)
{
    uint8_t index = g_active[(ENC_RIGHT == wheel) ? ENC_RIGHT : ENC_LEFT];

    return (g_pin_gpio[(index < ENC_PINS) ? index : 0u]);
}

/*!
 * @brief Shared GPIO bank-0 interrupt handler.
 */
static void
gpio_isr (UINT intno)
{
    uint32_t time_us = in_w(TIMER_RAW_LOW);
    uint32_t levels  = in_w(GPIO_IN);
    uint8_t  slot    = 0u;

    (void)intno;
    g_irq_count++;

    for (slot = 0u; slot < g_slot_count; slot++)
    {
        uint32_t pin     = g_slots[slot].pin;
        uint32_t reg     = pin / GPIO_PINS_PER_REG;
        uint32_t shift   = (pin % GPIO_PINS_PER_REG) * GPIO_BITS_PER_PIN;
        uint32_t pending = (0u == g_irq_core) ? in_w(IO_PROC0_INTS(reg))
                                              : in_w(IO_PROC1_INTS(reg));

        pending = (pending >> shift) & GPIO_EDGES;

        if (0u != pending)
        {
            out_w(IO_INTR(reg), pending << shift);     /* Clear the edge. */
            g_slots[slot].cb(0u != (levels & (1u << pin)), time_us);
        }
    }
}

#if ENC_USE_PHASE_B
/*!
 * @brief Left encoder edge: one store to a counter nobody else writes.
 */
static void
encoder_left_cb (bool b_high, uint32_t time_us)
{
    (void)time_us;
    g_pin_edges[ENC_PIN_L_A] = g_pin_edges[ENC_PIN_L_A] + 1u;
    g_count[ENC_LEFT] = step(g_count[ENC_LEFT], b_high, S2_PIN_ENC_L_B,
                             (0 != ENC_INVERT_LEFT));
}

/*!
 * @brief Right encoder edge.
 */
static void
encoder_right_cb (bool b_high, uint32_t time_us)
{
    (void)time_us;
    g_pin_edges[ENC_PIN_R_A] = g_pin_edges[ENC_PIN_R_A] + 1u;
    g_count[ENC_RIGHT] = step(g_count[ENC_RIGHT], b_high, S2_PIN_ENC_R_B,
                              (0 != ENC_INVERT_RIGHT));
}

/*!
 * @brief Next counter value for one phase-A edge: +1 forward, -1 backward
 *        (A and B differ after an edge in one direction and are equal in
 *        the other).
 */
static uint16_t
step (uint16_t count, bool b_a_high, uint8_t pin_b, bool b_invert)
{
    bool b_b_high  = (0u != (in_w(GPIO_IN) & (1u << pin_b)));
    bool b_forward = ((b_a_high != b_b_high) != b_invert);

    return ((uint16_t)(b_forward ? (count + 1u) : (count - 1u)));
}
#else
/*!
 * @brief One edge on one of a wheel's two port pins (phase A only).
 *
 * The wheel counts the pin in use.  The other pin takes over once it
 * leads by ENC_SWITCH_EDGES; the edges it gained meanwhile are added at
 * that moment, so the wheel count is always the busier pin's count.
 */
static void
count_edge (uint8_t wheel, uint8_t pin)
{
    uint8_t  active = g_active[wheel];
    uint32_t edges  = g_pin_edges[pin] + 1u;

    g_pin_edges[pin] = edges;

    if (pin == active)
    {
        g_count[wheel] = (uint16_t)(g_count[wheel] + 1u);
    }
    else
    {
        uint32_t lead = edges - g_pin_edges[active];   /* Wrap-safe. */

        if ((int32_t)lead >= ENC_SWITCH_EDGES)
        {
            g_active[wheel] = pin;
            g_count[wheel]  = (uint16_t)(g_count[wheel] + lead);
        }
    }
}

static void
edge_l_a (bool b_high, uint32_t time_us)
{
    (void)b_high;
    (void)time_us;
    count_edge(ENC_LEFT, ENC_PIN_L_A);
}

static void
edge_l_b (bool b_high, uint32_t time_us)
{
    (void)b_high;
    (void)time_us;
    count_edge(ENC_LEFT, ENC_PIN_L_B);
}

static void
edge_r_a (bool b_high, uint32_t time_us)
{
    (void)b_high;
    (void)time_us;
    count_edge(ENC_RIGHT, ENC_PIN_R_A);
}

static void
edge_r_b (bool b_high, uint32_t time_us)
{
    (void)b_high;
    (void)time_us;
    count_edge(ENC_RIGHT, ENC_PIN_R_B);
}
#endif

/*!
 * @brief Make a pin a digital input with the pull-up on.
 */
static void
input_pull_up (uint8_t pin)
{
    (void)gpio_set_pin(pin, GPIO_MODE_IN);
    clr_w(GPIO((UW)pin), GPIO_PDE);    /* UW: register address maths. */
    set_w(GPIO((UW)pin), GPIO_PUE);
}

/*** end of file ***/
