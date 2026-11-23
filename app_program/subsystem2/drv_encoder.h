/** @file drv_encoder.h
 *
 * @brief Wheel encoders (phase A, both edges) and the one shared GPIO
 *        bank-0 interrupt handler, owned by subsystem 2.  Other pins (the
 *        HC-SR04 echo) are attached with drv_encoder_attach() before the
 *        interrupt is enabled.
 */

#ifndef DRV_ENCODER_H
#define DRV_ENCODER_H

#include <stdint.h>
#include <stdbool.h>

#include "subsystem2.h"

#define ENC_LEFT               0u
#define ENC_RIGHT              1u

/* Phase A only (Tribunal 1): the counters only count up and the direction
 * comes from the command.  Each encoder has its own Grove port (left
 * Grove 1 = GP0/GP1, right Grove 2 = GP2/GP3), and BOTH signal pins of the
 * port are watched: the pin that carries phase A is found by itself (the
 * first one to lead the other by ENC_SWITCH_EDGES edges), so it does not
 * matter whether the encoder output is on the white or the yellow wire
 * (build log 26 Oct).  If both phases are wired, one of them is counted.
 *
 * Set ENC_USE_PHASE_B to 1 for quadrature (Week 6 A2 fallback): then phase
 * A must be on S2_PIN_ENC_x_A and phase B on S2_PIN_ENC_x_B, the counters
 * count up when the wheel turns forward and down when it turns backward,
 * and TSK_MOTION takes the direction from them.
 * TODO(Kai Xuan): 1 if the phase-A test fails NFR3. */
#define ENC_USE_PHASE_B        0

/* With phase B: if a wheel counts down while driving forward, flip it. */
#define ENC_INVERT_LEFT        0
#define ENC_INVERT_RIGHT       1   /* Mirrored motor on the right side. */

ER       drv_encoder_init(void);
ER       drv_encoder_attach(uint8_t pin, subsystem2_gpio_cb_t cb);
ER       drv_encoder_enable_irq(void);
uint16_t drv_encoder_count(uint8_t wheel);
uint32_t drv_encoder_irq_count(void);

/* Diagnostics: edges seen on one encoder pin since start-up (0 if the pin
 * is not an encoder pin), and the GPIO a wheel is being counted on. */
uint32_t drv_encoder_pin_edges(uint8_t gpio);
uint8_t  drv_encoder_active_gpio(uint8_t wheel);

#endif /* DRV_ENCODER_H */

/*** end of file ***/
