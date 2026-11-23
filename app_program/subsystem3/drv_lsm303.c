/** @file drv_lsm303.c
 *
 * @brief LSM303D driver (ST datasheet DocID023312), the team's IMU module,
 *        with the LSM303DLHC (GY-511, DocID018771) kept as a fallback.
 *
 * Nov 2026: the module on the car is an LSM303D, not an LSM303DLHC.  The
 * two chips are NOT register compatible, and the old driver (DLHC only:
 * accelerometer at 0x19, magnetometer at 0x1E) could never find an
 * LSM303D, which is what every "FAULT IMU" / "IMU not found" log showed.
 *
 *   LSM303D   one I2C address for both sensors: 0x1D (SA0 high, most
 *             breakouts) or 0x1E (SA0 low).  WHO_AM_I (0x0F) = 0x49.
 *             Accelerometer: CTRL1 0x20 = 0x6F (100 Hz, BDU, X/Y/Z),
 *             CTRL2 0x21 = 0xC8 (50 Hz anti-alias filter against motor
 *             vibration, +-4 g = 0.122 mg/LSB), 16-bit data at 0x28.
 *             Magnetometer: CTRL5 0x24 = 0x70 (high resolution, 50 Hz),
 *             CTRL6 0x25 = 0x20 (+-4 gauss), CTRL7 0x26 = 0x00
 *             (continuous), data at 0x08, little-endian X, Y, Z.
 *   LSM303DLHC accelerometer 0x19 (100 Hz, +-4 g, high resolution),
 *             magnetometer 0x1E (IRA_REG_M 0x0A = 'H'), data X, Z, Y
 *             big-endian.  Only used if no LSM303D answers.
 * Multi-byte reads set bit 7 of the register address (auto-increment) on
 * both chips.  drv_lsm303_open() finds out which chip is fitted and every
 * read uses that chip's map, so the rest of subsystem 3 sees the same
 * milli-g and magnetometer counts either way.
 *
 * I2C0 on GP16/GP17, driven directly and polled, the same way as the
 * Pico SDK's i2c_write_blocking() / i2c_read_blocking() (17 Nov).  The
 * kernel's interrupt-driven "iica" driver is no longer used: on the car
 * its first transfer timed out (-50 = E_TMOUT) right after the chip had
 * acknowledged its address, and its multi-byte read never sends a STOP
 * and acknowledges the last byte, which leaves the sensor in the middle
 * of a read.  A teammate's driver that uses the Pico SDK reads the same
 * module type on a Maker Pi Pico; this follows that code path, at the
 * same 100 kHz, with the pull-ups on.  Each transfer gives up after
 * IIC_TIMEOUT_US, so a missing sensor never blocks TSK_IMU for long.
 *
 * Bus recovery: the module is powered from the Robo Pico's 3V3, so it
 * keeps power when the Pico resets (RESET button, watchdog).  A reset in
 * the middle of a read can leave the sensor holding SDA low, and then no
 * transfer works until its power is cycled.  Every drv_lsm303_open()
 * therefore first takes GP16/GP17 as plain GPIO and clocks SCL until SDA
 * is released, then sends a STOP (NXP UM10204 3.1.16 "bus clear").  The
 * same bit-banged lines give drv_lsm303_diagnose() its wiring check.
 * Only TSK_IMU uses I2C0, and the I2C0 controller is idle between its
 * transfers, so taking the pins for a few hundred microseconds is safe.
 *
 * No long hangs: open first checks with a bit-banged address byte that
 * the chip answers, every transfer first checks that both lines are high,
 * and a transfer that still does not finish gives up after
 * IIC_TIMEOUT_US (20 ms for one byte) and resets the controller.  The kernel driver
 * used to wait 1000 ms (car tests 29 Oct and 17 Nov).
 */

#include "subsystem3.h"
#include "drv_lsm303.h"


#define AUTO_INCREMENT         0x80u   /* MSB of the sub-address.         */
#define XYZ_BYTES              6u

/* LSM303D (one address for both sensors). */
#define D_ADDR_SA0_HIGH        0x1Du
#define D_ADDR_SA0_LOW         0x1Eu
#define D_WHO_AM_I             0x0Fu
#define D_WHO_AM_I_VALUE       0x49u
#define D_OUT_X_L_M            0x08u
#define D_CTRL1                0x20u
#define D_CTRL1_100HZ_BDU_XYZ  0x6Fu   /* AODR 0110, BDU, AZEN AYEN AXEN. */
#define D_CTRL2                0x21u
#define D_CTRL2_50HZ_AA_4G     0xC8u   /* ABW 11 (50 Hz), AFS 001 (4 g).  */
#define D_CTRL5                0x24u
#define D_CTRL5_MAG_HR_50HZ    0x70u   /* M_RES 11, M_ODR 100 (50 Hz).    */
#define D_CTRL6                0x25u
#define D_CTRL6_MAG_4GAUSS     0x20u
#define D_CTRL7                0x26u
#define D_CTRL7_MAG_CONT       0x00u
#define D_OUT_X_L_A            0x28u
#define D_UG_PER_LSB           122     /* +-4 g: 0.122 mg/LSB.            */
#define D_UG_PER_MG            1000

/* LSM303DLHC (GY-511): two addresses. */
#define DLHC_ACC_ADDR          0x19u
#define DLHC_MAG_ADDR          0x1Eu
#define DLHC_CTRL_REG1_A       0x20u
#define DLHC_CTRL_REG1_100HZ   0x57u   /* ODR 100 Hz, X/Y/Z enabled.      */
#define DLHC_CTRL_REG4_A       0x23u
#define DLHC_CTRL_REG4_4G_HR   0x98u   /* BDU, +-4 g, high resolution.    */
#define DLHC_OUT_X_L_A         0x28u
#define DLHC_ACC_SHIFT         4       /* 12-bit data, left-justified.    */
#define DLHC_MG_PER_LSB        2       /* +-4 g, high resolution.         */
#define DLHC_CRA_REG_M         0x00u
#define DLHC_CRA_30HZ          0x14u
#define DLHC_CRB_REG_M         0x01u
#define DLHC_CRB_1_3_GAUSS     0x20u
#define DLHC_MR_REG_M          0x02u
#define DLHC_MR_CONTINUOUS     0x00u
#define DLHC_OUT_X_H_M         0x03u   /* Order: X, Z, Y; high byte first. */
#define DLHC_IRA_REG_M         0x0Au
#define DLHC_IRA_VALUE         0x48u   /* 'H'.                            */

/* Bit-banged I2C on the same pins (bus clear and wiring check only). */
#define BB_SDA                 S3_PIN_SDA
#define BB_SCL                 S3_PIN_SCL
#define BB_HALF_BIT_US         5u      /* 100 kHz.                        */
#define BB_CLEAR_CLOCKS        9u      /* Enough to finish any byte.      */
#define BB_ADDR_BITS           8u

/* PADS_BANK0: GPIO n pad control is at 0x4001C004 + 4*n.
 * SCHMITT | PULL-UP | DRIVE 2 mA | INPUT ENABLE. */
#define BB_PAD_BASE            0x4001C004u
#define BB_PAD(n)              (BB_PAD_BASE + (4u * (uint32_t)(n)))
#define BB_PAD_PULLUP          0x5Au
#define BB_PAD_NOPULL          0x52u

/* I2C0 (RP2040 datasheet 4.3.17), polled like the Pico SDK's hardware_i2c.
 * clk_sys is 125 MHz (kernel/sysdepend/pico_rp2040/cpu_clock.c).  100 kHz:
 * period 1250 clocks, low 3/5 = 750, high 500; spike filter low/16; SDA
 * hold 300 ns = 38 clocks (the SDK's i2c_set_baudrate() formulas). */
#define IIC_BASE               0x40044000u
#define IIC_CON                (IIC_BASE + 0x00u)
#define IIC_TAR                (IIC_BASE + 0x04u)
#define IIC_DATA_CMD           (IIC_BASE + 0x10u)
#define IIC_FS_SCL_HCNT        (IIC_BASE + 0x1Cu)
#define IIC_FS_SCL_LCNT        (IIC_BASE + 0x20u)
#define IIC_INTR_MASK          (IIC_BASE + 0x30u)
#define IIC_RAW_INTR_STAT      (IIC_BASE + 0x34u)
#define IIC_RX_TL              (IIC_BASE + 0x38u)
#define IIC_TX_TL              (IIC_BASE + 0x3Cu)
#define IIC_CLR_INTR           (IIC_BASE + 0x40u)
#define IIC_CLR_TX_ABRT        (IIC_BASE + 0x54u)
#define IIC_CLR_STOP_DET       (IIC_BASE + 0x60u)
#define IIC_ENABLE             (IIC_BASE + 0x6Cu)
#define IIC_TXFLR              (IIC_BASE + 0x74u)
#define IIC_RXFLR              (IIC_BASE + 0x78u)
#define IIC_SDA_HOLD           (IIC_BASE + 0x7Cu)
#define IIC_TX_ABRT_SOURCE     (IIC_BASE + 0x80u)
#define IIC_DMA_CR             (IIC_BASE + 0x88u)
#define IIC_FS_SPKLEN          (IIC_BASE + 0xA0u)
/* Fast-mode speed field, master, slave off, repeated start, TX_EMPTY only
 * once a byte has left the shift register (as i2c_init()). */
#define IIC_CON_MASTER         0x165u
#define IIC_FS_HCNT_100K       500u
#define IIC_FS_LCNT_100K       750u
#define IIC_FS_SPKLEN_100K     46u
#define IIC_SDA_HOLD_300NS     38u
#define IIC_CMD_READ           (1u << 8)
#define IIC_CMD_STOP           (1u << 9)
#define IIC_CMD_RESTART        (1u << 10)
#define IIC_INT_TX_EMPTY       (1u << 4)
#define IIC_INT_TX_ABRT        (1u << 6)
#define IIC_INT_STOP_DET       (1u << 9)
#define IIC_ENABLE_ON          1u
#define IIC_ENABLE_ABORT       2u
#define IIC_TX_FIFO_DEPTH      16u
#define IIC_TIMEOUT_US         20000u  /* Per byte; a byte takes 0.09 ms. */
#define IIC_STATUS             (IIC_BASE + 0x70u)
#define IIC_ENABLE_STATUS      (IIC_BASE + 0x9Cu)
#define IIC_REPORT_FIRST       6u      /* Failures printed in full ...    */
#define IIC_REPORT_EVERY       100u    /* ... then one in this many.      */

static uint8_t g_chip     = LSM303_CHIP_NONE;
static uint8_t g_acc_addr = 0u;     /* Accelerometer (and LSM303D) address. */
static uint8_t g_mag_addr = 0u;

static ER   write_reg(uint8_t addr, uint8_t reg, uint8_t value);
static ER   read_regs(uint8_t addr, uint8_t reg, uint8_t * p_buf, uint8_t len);
static ER   open_d(uint8_t addr);
static ER   open_dlhc(void);
static bool lines_high(void);
static bool bb_answers(uint8_t addr);
static void bus_clear(void);
static void bb_take_pins(void);
static void iic_setup(void);
static ER   iic_write(uint8_t addr, uint8_t const * p_src, uint8_t len,
                      bool b_nostop);
static ER   iic_read(uint8_t addr, uint8_t * p_dst, uint8_t len,
                     bool b_nostop);
static void iic_recover(void);
static void iic_report(char const * p_what, uint8_t addr, uint8_t index,
                       uint8_t len, uint32_t start, uint32_t abort);
static bool iic_late(uint32_t start);
static void bb_give_pins(void);
static void bb_wait(void);
static void bb_low(uint8_t pin);
static void bb_release(uint8_t pin);
static bool bb_high(uint8_t pin);
static bool bb_probe(uint8_t sda, uint8_t scl, uint8_t addr);
static void bb_stop(uint8_t sda, uint8_t scl);


/*!
 * @brief Open the I2C device (once), find the chip and configure it.  Safe
 *        to call again to re-probe a sensor that dropped out (Buddy 4's
 *        "re-probe once a second" idea, 26 Oct).
 *
 * Order: LSM303D at 0x1D, LSM303D at 0x1E, then LSM303DLHC (0x19 + 0x1E).
 * Each candidate is first checked with a bit-banged address byte, so a
 * missing chip costs microseconds, not a transfer timeout.
 *
 * @return E_OK, or E_IO / the I2C error (nothing answered).
 */
ER
drv_lsm303_open (void)
{
    ER result = E_IO;

    g_chip = LSM303_CHIP_NONE;

    bus_clear();        /* Ends with I2C0 reset and in charge. */

    if (bb_answers(D_ADDR_SA0_HIGH))
    {
        result = open_d(D_ADDR_SA0_HIGH);
    }

    if ((E_OK != result) && bb_answers(D_ADDR_SA0_LOW))
    {
        result = open_d(D_ADDR_SA0_LOW);
    }

    if ((E_OK != result) && bb_answers(DLHC_ACC_ADDR))
    {
        result = open_dlhc();
    }

    return (result);
}

/*!
 * @brief Which chip drv_lsm303_open() found (LSM303_CHIP_*).
 */
uint8_t
drv_lsm303_chip (void)
{
    return (g_chip);
}

/*!
 * @brief Printable name of the chip found, with its address.
 */
char const *
drv_lsm303_chip_name (void)
{
    char const * p_name = "none";

    if (LSM303_CHIP_D == g_chip)
    {
        p_name = (D_ADDR_SA0_HIGH == g_acc_addr) ? "LSM303D at 0x1D"
                                                 : "LSM303D at 0x1E";
    }
    else if (LSM303_CHIP_DLHC == g_chip)
    {
        p_name = "LSM303DLHC (GY-511) at 0x19/0x1E";
    }
    else
    {
        /* Nothing found. */
    }

    return (p_name);
}

/*!
 * @brief Read the accelerometer in milli-g (sensor axes).
 */
ER
drv_lsm303_read_accel (lsm303_vec_t * p_mg)
{
    uint8_t raw[XYZ_BYTES];
    ER      result = E_IO;

    if (LSM303_CHIP_D == g_chip)
    {
        result = read_regs(g_acc_addr,
                           (uint8_t)(D_OUT_X_L_A | AUTO_INCREMENT),
                           raw, XYZ_BYTES);

        if (E_OK == result)
        {
            /* 16-bit little-endian pairs, 0.122 mg per LSB at +-4 g. */
            int32_t x = (int16_t)(uint16_t)(raw[0] | (raw[1] << 8));
            int32_t y = (int16_t)(uint16_t)(raw[2] | (raw[3] << 8));
            int32_t z = (int16_t)(uint16_t)(raw[4] | (raw[5] << 8));

            p_mg->x = (int16_t)((x * D_UG_PER_LSB) / D_UG_PER_MG);
            p_mg->y = (int16_t)((y * D_UG_PER_LSB) / D_UG_PER_MG);
            p_mg->z = (int16_t)((z * D_UG_PER_LSB) / D_UG_PER_MG);
        }
    }
    else if (LSM303_CHIP_DLHC == g_chip)
    {
        result = read_regs(g_acc_addr,
                           (uint8_t)(DLHC_OUT_X_L_A | AUTO_INCREMENT),
                           raw, XYZ_BYTES);

        if (E_OK == result)
        {
            /* Little-endian pairs; arithmetic shift keeps the sign. */
            p_mg->x = (int16_t)(((int16_t)(uint16_t)(raw[0] | (raw[1] << 8)) >>
                                 DLHC_ACC_SHIFT) * DLHC_MG_PER_LSB);
            p_mg->y = (int16_t)(((int16_t)(uint16_t)(raw[2] | (raw[3] << 8)) >>
                                 DLHC_ACC_SHIFT) * DLHC_MG_PER_LSB);
            p_mg->z = (int16_t)(((int16_t)(uint16_t)(raw[4] | (raw[5] << 8)) >>
                                 DLHC_ACC_SHIFT) * DLHC_MG_PER_LSB);
        }
    }
    else
    {
        /* Not open: E_IO. */
    }

    return (result);
}

/*!
 * @brief Read the magnetometer, raw counts (sensor axes).  Only ratios
 *        are used (heading), so the two chips' different scales do not
 *        matter.
 */
ER
drv_lsm303_read_mag (lsm303_vec_t * p_raw)
{
    uint8_t raw[XYZ_BYTES];
    ER      result = E_IO;

    if (LSM303_CHIP_D == g_chip)
    {
        result = read_regs(g_mag_addr,
                           (uint8_t)(D_OUT_X_L_M | AUTO_INCREMENT),
                           raw, XYZ_BYTES);

        if (E_OK == result)
        {
            /* Little-endian pairs in the order X, Y, Z. */
            p_raw->x = (int16_t)(uint16_t)(raw[0] | (raw[1] << 8));
            p_raw->y = (int16_t)(uint16_t)(raw[2] | (raw[3] << 8));
            p_raw->z = (int16_t)(uint16_t)(raw[4] | (raw[5] << 8));
        }
    }
    else if (LSM303_CHIP_DLHC == g_chip)
    {
        result = read_regs(g_mag_addr, DLHC_OUT_X_H_M, raw, XYZ_BYTES);

        if (E_OK == result)
        {
            /* Big-endian pairs in the order X, Z, Y. */
            p_raw->x = (int16_t)(uint16_t)((raw[0] << 8) | raw[1]);
            p_raw->z = (int16_t)(uint16_t)((raw[2] << 8) | raw[3]);
            p_raw->y = (int16_t)(uint16_t)((raw[4] << 8) | raw[5]);
        }
    }
    else
    {
        /* Not open: E_IO. */
    }

    return (result);
}

/*!
 * @brief Wiring check after the IMU did not answer: line levels, and
 *        whether it answers with SDA and SCL swapped.  Prints one line.
 */
void
drv_lsm303_diagnose (void)
{
    bool b_sda    = false;
    bool b_scl    = false;
    bool b_normal = false;
    bool b_swap   = false;

    bb_take_pins();
    b_sda = bb_high(BB_SDA);
    b_scl = bb_high(BB_SCL);

    if (b_sda && b_scl)
    {
        b_normal = bb_probe(BB_SDA, BB_SCL, D_ADDR_SA0_HIGH) ||
                   bb_probe(BB_SDA, BB_SCL, D_ADDR_SA0_LOW) ||
                   bb_probe(BB_SDA, BB_SCL, DLHC_ACC_ADDR);

        if (!b_normal)
        {
            b_swap = bb_probe(BB_SCL, BB_SDA, D_ADDR_SA0_HIGH) ||
                     bb_probe(BB_SCL, BB_SDA, D_ADDR_SA0_LOW) ||
                     bb_probe(BB_SCL, BB_SDA, DLHC_ACC_ADDR);
            bb_stop(BB_SDA, BB_SCL);
        }
    }

    bb_give_pins();

    tm_printf((UB *)"S3: I2C check: SDA (GP%d) %s, SCL (GP%d) %s; IMU %s\n",
              (int)BB_SDA, b_sda ? "high" : "LOW",
              (int)BB_SCL, b_scl ? "high" : "LOW",
              (!b_sda || !b_scl) ? "-> a line is held low: check the IMU "
                                   "module's 3V3/GND and the Grove 4 cable" :
              b_normal ? "answers on the bus, but not as an LSM303D "
                         "(WHO_AM_I) or LSM303DLHC: other chip?" :
              b_swap ? "answers with SDA/SCL SWAPPED -> swap the white and "
                       "yellow wires at the module" :
              "does not answer -> check VCC (3V3), GND, SDA = GP16, "
              "SCL = GP17 (the S3 check image scans every Grove pair)");
}

/*!
 * @brief Point the bit-bang I2C at any pin pair, not just the configured one.
 */
static void
bb_set_pins (uint8_t sda, uint8_t scl)
{
    out_w(GPIO_OE_CLR,  (1u << sda) | (1u << scl));
    out_w(GPIO_OUT_CLR, (1u << sda) | (1u << scl));
    out_w(GPIO_CTRL(sda), GPIO_CTRL_FUNCSEL_SIO);
    out_w(GPIO_CTRL(scl), GPIO_CTRL_FUNCSEL_SIO);

    /* A pair the kernel never configured has no pull-up, so SDA floats low
     * and every address reads as ACK.  Two pairs did exactly that on the
     * first sweep (4/5, 6/7, 12/13, 18/19, 26/27 - all false).  Pull both
     * lines up first, exactly as a real bus would be idling. */
    out_w(BB_PAD(sda), BB_PAD_PULLUP);
    out_w(BB_PAD(scl), BB_PAD_PULLUP);
    bb_wait();
}

/*!
 * @brief Probe every plausible Grove pair for a known IMU address.
 *
 * The board's port labels are not the RP2040 pins.  Rather than trust them,
 * try each pair and report where a device answers (2 Nov: "everything is
 * connected and looks correct" but nothing was on GP16/GP17).
 *
 * GP8..GP11 (motors), GP14/GP15 (servo) and GP23..GP25/GP29 (radio) are
 * deliberately left alone.
 */
void
drv_lsm303_sweep (void)
{
    static uint8_t const s_pair[][2] =
    {
        /* GP20/GP21 are the Robo Pico START/RESET buttons, not a Grove
         * port: probing them gave a false hit at every address and fake
         * START/RESET presses (17 Nov, on the car). */
        { 0u,  1u }, { 2u,  3u }, { 4u,  5u }, { 6u,  7u },
        { 12u, 13u }, { 16u, 17u }, { 18u, 19u }, { 26u, 27u }
    };
    static uint8_t const s_addr[] =
    {
        0x19u, 0x1Eu, 0x1Du, 0x68u, 0x69u, 0x6Bu
    };
    uint8_t  i    = 0u;
    uint8_t  j    = 0u;
    uint32_t hits = 0u;

    for (i = 0u; i < (uint8_t)(sizeof(s_pair) / sizeof(s_pair[0])); i++)
    {
        uint8_t a = s_pair[i][0];
        uint8_t b = s_pair[i][1];
        bool    b_hit = false;

        bb_set_pins(a, b);
        bb_release(a);
        bb_release(b);
        bb_wait();

        /* Both lines must idle high before an ACK means anything. */
        if (!bb_high(a) || !bb_high(b))
        {
            out_w(BB_PAD(a), BB_PAD_NOPULL);
            out_w(BB_PAD(b), BB_PAD_NOPULL);
            continue;
        }

        /* Reserved address 0x7F must never acknowledge.  If it does, SCL is
         * not clocking and every address would look like a device - which is
         * exactly what GP18/19 and GP26/27 did (ADC pins, 12 false hits). */
        if (bb_probe(a, b, 0x7Fu) || bb_probe(b, a, 0x7Fu))
        {
            tm_printf((UB *)"S3: SWEEP GP%d/GP%d not a usable bus (ignored)\n",
                      (int)a, (int)b);
            out_w(BB_PAD(a), BB_PAD_NOPULL);
            out_w(BB_PAD(b), BB_PAD_NOPULL);
            continue;
        }

        for (j = 0u; j < (uint8_t)(sizeof(s_addr) / sizeof(s_addr[0])); j++)
        {
            if (bb_probe(a, b, s_addr[j]) || bb_probe(b, a, s_addr[j]))
            {
                tm_printf((UB *)"S3: SWEEP device 0x%02X on SDA GP%d / SCL GP%d\n",
                          (int)s_addr[j], (int)a, (int)b);
                hits++;
                b_hit = true;
            }
        }

        out_w(BB_PAD(a), BB_PAD_NOPULL);
        out_w(BB_PAD(b), BB_PAD_NOPULL);

        (void)b_hit;
    }

    if (0u == hits)
    {
        tm_printf((UB *)"S3: SWEEP every Grove pair -> no IMU anywhere "
                  "(module dead, or 3V3/GND not reaching it)\n");
    }

    bb_take_pins();     /* back to the configured pair ... */
    bb_give_pins();     /* ... and to a freshly reset I2C0 */
}

/*!
 * @brief Probe every 7-bit I2C address 0x08..0x77 and name anything found.
 *
 * The driver finds an LSM303D (0x1D or 0x1E) or an LSM303DLHC (0x19 +
 * 0x1E).  A different module (MPU-6050 at 0x68, ...) would look exactly
 * like a wiring fault.  This prints what is actually on the bus so the chip
 * is a fact, not an assumption (2 Nov; LSM303D names Nov 2026).
 */
void
drv_lsm303_scan (void)
{
    uint8_t addr  = 0u;
    uint8_t found = 0u;

    /* The app hands the pins back to the hardware I2C peripheral when it
     * finishes with them, so SIO owns nothing here.  Without this the pad
     * ignores every bit-banged edge and EVERY address reads as NACK - the
     * "no device anywhere" result was this bug, not a dead module (2 Nov). */
    bb_take_pins();

    /* Is anything OUT THERE at all?  A powered I2C module carries its own
     * pull-up resistors, so with the Pico's internal pulls removed SDA and
     * SCL still read high.  Reading low means the module is not powered, or
     * its wires are not connected - no multimeter needed (2 Nov). */
    out_w(BB_PAD(BB_SDA), BB_PAD_NOPULL);
    out_w(BB_PAD(BB_SCL), BB_PAD_NOPULL);
    bb_wait();
    bb_wait();

    tm_printf((UB *)"S3: module pull-ups: SDA GP%d %s, SCL GP%d %s -> %s\n",
              (int)BB_SDA, bb_high(BB_SDA) ? "HIGH" : "low",
              (int)BB_SCL, bb_high(BB_SCL) ? "HIGH" : "low",
              (bb_high(BB_SDA) && bb_high(BB_SCL)) ?
              "module IS powered and wired (chip not answering = dead chip)" :
              "NO pull-up: module has no power, or a wire is not connected");

    out_w(BB_PAD(BB_SDA), BB_PAD_PULLUP);
    out_w(BB_PAD(BB_SCL), BB_PAD_PULLUP);
    bb_wait();

    /* Loopback: can the Pico pull each line low and see it?  If a pad is
     * damaged (open driver) the bus never clocks and every device stays
     * silent - which is indistinguishable from a dead module unless tested. */
    bb_low(BB_SDA);
    bb_wait();
    {
        bool b_sda_ok = !bb_high(BB_SDA);

        bb_release(BB_SDA);
        bb_low(BB_SCL);
        bb_wait();
        {
            bool b_scl_ok = !bb_high(BB_SCL);

            bb_release(BB_SCL);
            bb_wait();
            tm_printf((UB *)"S3: drive test SDA GP%d %s, SCL GP%d %s -> %s\n",
                      (int)BB_SDA, b_sda_ok ? "ok" : "STUCK HIGH",
                      (int)BB_SCL, b_scl_ok ? "ok" : "STUCK HIGH",
                      (b_sda_ok && b_scl_ok) ?
                      "car pins can drive the bus (fault is not the Pico)" :
                      "CAR PIN DAMAGED - it cannot pull the line low");
        }
    }

    /* Positive control: 0x7F is reserved and must never acknowledge.  If it
     * does, the bus is not being driven properly and no result below is
     * trustworthy. */
    tm_printf((UB *)"S3: sanity 0x7F %s\n",
              bb_probe(BB_SDA, BB_SCL, 0x7Fu) ?
              "ACKed -> RESULTS INVALID" : "NACKs (scan is trustworthy)");

    tm_printf((UB *)"S3: I2C scan SDA GP%d / SCL GP%d ...\n",
              (int)BB_SDA, (int)BB_SCL);

    for (addr = 0x08u; addr <= 0x77u; addr++)
    {
        if (bb_probe(BB_SDA, BB_SCL, addr))
        {
            UB const * p_name = (UB const *)"unknown";

            if (0x19u == addr)
            {
                p_name = (UB const *)"LSM303DLHC accel (GY-511) - supported";
            }
            else if (0x1Eu == addr)
            {
                p_name = (UB const *)"LSM303D (SA0 low) or LSM303DLHC mag "
                                     "- supported";
            }
            else if (0x1Du == addr)
            {
                p_name = (UB const *)"LSM303D (SA0 high) - supported";
            }
            else if ((0x68u == addr) || (0x69u == addr))
            {
                p_name = (UB const *)"MPU-6050 / MPU-9250";
            }
            else if (0x6Bu == addr)
            {
                p_name = (UB const *)"LSM6DS3 / LSM6DSO";
            }

            tm_printf((UB *)"S3: I2C device at 0x%02X  %s\n", (int)addr,
                      p_name);
            found++;
        }
        else if (bb_probe(BB_SCL, BB_SDA, addr))     /* white/yellow swapped */
        {
            tm_printf((UB *)"S3: I2C device at 0x%02X with SDA/SCL SWAPPED - "
                      "move the white and yellow wires back\n", (int)addr);
            found++;
        }
    }

    if (0u == found)
    {
        tm_printf((UB *)"S3: I2C scan found NOTHING - power, GND, or SDA/SCL "
                  "swapped\n");
    }
    else
    {
        tm_printf((UB *)"S3: I2C scan found %d device(s)\n", (int)found);
    }

    bb_give_pins();     /* hand the bus back to the hardware I2C driver */
}

/*!
 * @brief True if SDA and SCL are both high (bus idle).  Read from the pad
 *        inputs, which work whichever function owns the pins.
 */
static bool
lines_high (void)
{
    uint32_t const both = (1u << BB_SDA) | (1u << BB_SCL);

    return ((in_w(GPIO_IN) & both) == both);
}

/*!
 * @brief True if a chip acknowledges this address (bit-banged, about
 *        0.2 ms; never waits on a line).
 */
static bool
bb_answers (uint8_t addr)
{
    bool b_ack = false;

    bb_take_pins();

    if (bb_high(BB_SDA) && bb_high(BB_SCL))
    {
        b_ack = bb_probe(BB_SDA, BB_SCL, addr);
    }

    bb_give_pins();

    return (b_ack);
}

/*!
 * @brief Identify and configure an LSM303D at addr.
 */
static ER
open_d (uint8_t addr)
{
    uint8_t ident  = 0u;
    ER      result = read_regs(addr, D_WHO_AM_I, &ident, 1u);

    if ((E_OK == result) && (D_WHO_AM_I_VALUE != ident))
    {
        result = E_IO;  /* Something else answered (e.g. a DLHC's 0x1E). */
    }

    if (E_OK == result)
    {
        result = write_reg(addr, D_CTRL2, D_CTRL2_50HZ_AA_4G);
    }

    if (E_OK == result)
    {
        result = write_reg(addr, D_CTRL1, D_CTRL1_100HZ_BDU_XYZ);
    }

    if (E_OK == result)
    {
        result = write_reg(addr, D_CTRL5, D_CTRL5_MAG_HR_50HZ);
    }

    if (E_OK == result)
    {
        result = write_reg(addr, D_CTRL6, D_CTRL6_MAG_4GAUSS);
    }

    if (E_OK == result)
    {
        result = write_reg(addr, D_CTRL7, D_CTRL7_MAG_CONT);
    }

    if (E_OK == result)
    {
        g_chip     = LSM303_CHIP_D;
        g_acc_addr = addr;
        g_mag_addr = addr;
    }

    return (result);
}

/*!
 * @brief Identify and configure an LSM303DLHC (GY-511).
 */
static ER
open_dlhc (void)
{
    uint8_t ident  = 0u;
    ER      result = write_reg(DLHC_ACC_ADDR, DLHC_CTRL_REG1_A,
                               DLHC_CTRL_REG1_100HZ);

    if (E_OK == result)
    {
        result = write_reg(DLHC_ACC_ADDR, DLHC_CTRL_REG4_A,
                           DLHC_CTRL_REG4_4G_HR);
    }

    if (E_OK == result)
    {
        result = write_reg(DLHC_MAG_ADDR, DLHC_CRA_REG_M, DLHC_CRA_30HZ);
    }

    if (E_OK == result)
    {
        result = write_reg(DLHC_MAG_ADDR, DLHC_CRB_REG_M, DLHC_CRB_1_3_GAUSS);
    }

    if (E_OK == result)
    {
        result = write_reg(DLHC_MAG_ADDR, DLHC_MR_REG_M, DLHC_MR_CONTINUOUS);
    }

    if (E_OK == result)
    {
        result = read_regs(DLHC_MAG_ADDR, DLHC_IRA_REG_M, &ident, 1u);

        if ((E_OK == result) && (DLHC_IRA_VALUE != ident))
        {
            result = E_IO;
        }
    }

    if (E_OK == result)
    {
        g_chip     = LSM303_CHIP_DLHC;
        g_acc_addr = DLHC_ACC_ADDR;
        g_mag_addr = DLHC_MAG_ADDR;
    }

    return (result);
}

/*!
 * @brief Free a bus held by a sensor stuck in the middle of a byte.
 */
static void
bus_clear (void)
{
    uint8_t clock = 0u;

    bb_take_pins();

    for (clock = 0u; (clock < BB_CLEAR_CLOCKS) && (!bb_high(BB_SDA));
         clock++)
    {
        bb_low(BB_SCL);
        bb_wait();
        bb_release(BB_SCL);
        bb_wait();
    }

    bb_stop(BB_SDA, BB_SCL);
    bb_give_pins();
}

/*!
 * @brief GP16/GP17 as plain open-drain GPIO (SIO), both released.  The
 *        pads keep the driver's settings (input on, pull-ups on).
 */
static void
bb_take_pins (void)
{
    out_w(GPIO_OE_CLR, (1u << BB_SDA) | (1u << BB_SCL));
    out_w(GPIO_OUT_CLR, (1u << BB_SDA) | (1u << BB_SCL));
    out_w(GPIO_CTRL(BB_SDA), GPIO_CTRL_FUNCSEL_SIO);
    out_w(GPIO_CTRL(BB_SCL), GPIO_CTRL_FUNCSEL_SIO);
    bb_wait();
}

/*!
 * @brief Give GP16/GP17 back to I2C0, with the controller reset first.
 *
 * Car test 17 Nov: after the bit-banged probe the controller kept the
 * command in its FIFO and never started (STATUS 0x02: not active, one
 * entry in the TX FIFO, both lines high), for every transfer, 5 ms each.
 * It had followed the bit-banged START on the pins but not the end of the
 * transfer, so it took the bus for busy (another master).  The Pico SDK
 * never bit-bangs, which is why the teammate's driver worked.  Resetting
 * I2C0 while the pins are still plain GPIO and idle-high, and only then
 * selecting the I2C function, hands it a bus it sees as free.
 */
static void
bb_give_pins (void)
{
    out_w(GPIO_OE_CLR, (1u << BB_SDA) | (1u << BB_SCL));
    iic_setup();        /* Reset, configure, then FUNCSEL = I2C. */
}

/*!
 * @brief Half a bit time on the 1 MHz timer.
 */
static void
bb_wait (void)
{
    uint32_t start = in_w(TIMER_TIMERAWL);

    while ((in_w(TIMER_TIMERAWL) - start) < BB_HALF_BIT_US)
    {
        /* Busy wait: a few microseconds. */
    }
}

/*!
 * @brief Pull a line low (output enable; the output level is 0).
 */
static void
bb_low (uint8_t pin)
{
    out_w(GPIO_OE_SET, 1u << pin);
}

/*!
 * @brief Let a line go high through its pull-up.
 */
static void
bb_release (uint8_t pin)
{
    out_w(GPIO_OE_CLR, 1u << pin);
}

/*!
 * @brief Level of a line.
 */
static bool
bb_high (uint8_t pin)
{
    return (0u != (in_w(GPIO_IN) & (1u << pin)));
}

/*!
 * @brief Send START + address (write) and read the ACK, then STOP.
 *
 * @return True if a device acknowledged.
 */
static bool
bb_probe (uint8_t sda, uint8_t scl, uint8_t addr)
{
    uint8_t byte  = (uint8_t)(addr << 1u);     /* R/W bit 0: write. */
    uint8_t bit   = 0u;
    bool    b_ack = false;

    bb_release(sda);
    bb_release(scl);
    bb_wait();
    bb_low(sda);                    /* START: SDA falls while SCL is high. */
    bb_wait();
    bb_low(scl);
    bb_wait();

    for (bit = 0u; bit < BB_ADDR_BITS; bit++)
    {
        if (0u != (byte & (0x80u >> bit)))
        {
            bb_release(sda);
        }
        else
        {
            bb_low(sda);
        }

        bb_wait();
        bb_release(scl);
        bb_wait();
        bb_low(scl);
    }

    bb_release(sda);                /* ACK bit: the device pulls SDA low. */
    bb_wait();
    bb_release(scl);
    bb_wait();
    b_ack = !bb_high(sda);
    bb_low(scl);
    bb_wait();
    bb_stop(sda, scl);

    return (b_ack);
}

/*!
 * @brief STOP: SDA rises while SCL is high.  Leaves both lines released.
 */
static void
bb_stop (uint8_t sda, uint8_t scl)
{
    bb_low(scl);
    bb_wait();
    bb_low(sda);
    bb_wait();
    bb_release(scl);
    bb_wait();
    bb_release(sda);
    bb_wait();
}

/*!
 * @brief Write one register: payload {reg, value} to the slave address.
 */
static ER
write_reg (uint8_t addr, uint8_t reg, uint8_t value)
{
    uint8_t data[2];
    ER      result = E_OK;

    data[0] = reg;
    data[1] = value;

    if (!lines_high())
    {
        result = E_IO;      /* A line is held low: nothing can answer. */
    }
    else
    {
        result = iic_write(addr, data, 2u, false);
    }

    return (result);
}

/*!
 * @brief Read len registers starting at reg (repeated-start transfer).
 */
static ER
read_regs (uint8_t addr, uint8_t reg, uint8_t * p_buf, uint8_t len)
{
    uint8_t sub    = reg;
    ER      result = E_OK;

    if (!lines_high())
    {
        result = E_IO;      /* A line is held low: nothing can answer. */
    }
    else
    {
        /* Register address, no STOP, then a repeated START and the read
         * (i2c_write_blocking(.., true) + i2c_read_blocking(.., false)). */
        result = iic_write(addr, &sub, 1u, true);

        if (E_OK == result)
        {
            result = iic_read(addr, p_buf, len, false);
        }
    }

    return (result);
}

/* Set by a transfer that ended without a STOP: the next one starts with a
 * repeated START (the SDK's i2c->restart_on_next). */
static bool g_iic_restart = false;

/*!
 * @brief Reset I2C0 and set it up as a 100 kHz master on GP16/GP17 with
 *        the pull-ups on (i2c_init() + gpio_set_function() +
 *        gpio_pull_up()).  Interrupts stay masked: everything is polled.
 */
static void
iic_setup (void)
{
    set_w(RESETS_RESET, RESETS_RESET_I2C0);
    clr_w(RESETS_RESET, RESETS_RESET_I2C0);

    while (0u == (in_w(RESETS_RESET_DONE) & RESETS_RESET_I2C0))
    {
        /* A few clock cycles. */
    }

    out_w(IIC_ENABLE, 0u);
    out_w(IIC_CON, IIC_CON_MASTER);
    out_w(IIC_TX_TL, 0u);
    out_w(IIC_RX_TL, 0u);
    out_w(IIC_DMA_CR, 0u);
    out_w(IIC_INTR_MASK, 0u);       /* The kernel's ISR never fires. */
    out_w(IIC_FS_SCL_HCNT, IIC_FS_HCNT_100K);
    out_w(IIC_FS_SCL_LCNT, IIC_FS_LCNT_100K);
    out_w(IIC_FS_SPKLEN, IIC_FS_SPKLEN_100K);
    out_w(IIC_SDA_HOLD, IIC_SDA_HOLD_300NS);
    out_w(IIC_ENABLE, IIC_ENABLE_ON);

    out_w(BB_PAD(BB_SDA), BB_PAD_PULLUP);
    out_w(BB_PAD(BB_SCL), BB_PAD_PULLUP);
    out_w(GPIO_CTRL(BB_SDA), GPIO_CTRL_FUNCSEL_I2C);
    out_w(GPIO_CTRL(BB_SCL), GPIO_CTRL_FUNCSEL_I2C);

    g_iic_restart = false;
}

/*!
 * @brief True once IIC_TIMEOUT_US have passed since start (1 MHz timer).
 */
static bool
iic_late (uint32_t start)
{
    return ((in_w(TIMER_TIMERAWL) - start) >= IIC_TIMEOUT_US);
}

/*!
 * @brief After a timeout: abort (the controller sends a STOP), then start
 *        again from reset so the next transfer finds an idle bus.
 */
static void
iic_recover (void)
{
    uint32_t start = in_w(TIMER_TIMERAWL);

    out_w(IIC_ENABLE, IIC_ENABLE_ON | IIC_ENABLE_ABORT);

    while ((0u != (in_w(IIC_ENABLE) & IIC_ENABLE_ABORT)) && !iic_late(start))
    {
        /* The abort finishes within one byte time. */
    }

    bus_clear();        /* Ends with I2C0 reset (bb_give_pins). */
}

/*!
 * @brief Print what the I2C0 controller and the two lines were doing when a
 *        transfer failed (car test 17 Nov: the SDK reads the chip, this
 *        driver timed out).  The first IIC_REPORT_FIRST failures, then one
 *        in IIC_REPORT_EVERY, so a missing sensor does not flood the console.
 */
static void
iic_report (char const * p_what, uint8_t addr, uint8_t index, uint8_t len,
            uint32_t start, uint32_t abort)
{
    static uint32_t count = 0u;

    count++;

    if ((count <= IIC_REPORT_FIRST) || (0u == (count % IIC_REPORT_EVERY)))
    {
        uint32_t lines = in_w(GPIO_IN);

        tm_printf((UB *)"S3: I2C %s #%u: addr 0x%02X byte %u of %u, %u us, "
                  "status 0x%02X raw 0x%04X abort 0x%X en 0x%X txflr %u "
                  "rxflr %u, SDA %s SCL %s\n",
                  p_what, (UINT)count, (UINT)addr, (UINT)index,
                  (UINT)len, (UINT)(in_w(TIMER_TIMERAWL) - start),
                  (UINT)in_w(IIC_STATUS), (UINT)in_w(IIC_RAW_INTR_STAT),
                  (UINT)abort, (UINT)in_w(IIC_ENABLE_STATUS),
                  (UINT)in_w(IIC_TXFLR), (UINT)in_w(IIC_RXFLR),
                  (0u != (lines & (1u << BB_SDA))) ? "high" : "LOW",
                  (0u != (lines & (1u << BB_SCL))) ? "high" : "LOW");
    }
}

/*!
 * @brief i2c_write_blocking_internal(): one byte at a time, waiting until
 *        it has left the shift register, then for the STOP.
 *
 * @return E_OK, E_IO (not acknowledged) or E_TMOUT.
 */
static ER
iic_write (uint8_t addr, uint8_t const * p_src, uint8_t len, bool b_nostop)
{
    uint32_t start   = in_w(TIMER_TIMERAWL);
    uint32_t reason  = 0u;
    bool     b_abort = false;
    bool     b_late  = false;
    uint8_t  i       = 0u;
    ER       result  = E_OK;

    out_w(IIC_ENABLE, 0u);
    out_w(IIC_TAR, (UW)addr);
    out_w(IIC_ENABLE, IIC_ENABLE_ON);

    for (i = 0u; (i < len) && !b_abort; i++)
    {
        bool b_first = (0u == i);
        bool b_last  = ((len - 1u) == i);
        UW   cmd     = (UW)p_src[i];

        start = in_w(TIMER_TIMERAWL);   /* The limit is per byte. */

        if (b_first && g_iic_restart)
        {
            cmd |= IIC_CMD_RESTART;
        }

        if (b_last && !b_nostop)
        {
            cmd |= IIC_CMD_STOP;
        }

        out_w(IIC_DATA_CMD, cmd);

        /* TX_EMPTY once the byte has left the shift register.  A NACK
         * flushes the FIFO, which sets it too on the chip; TX_ABRT is
         * checked as well so an abort is never mistaken for a hang. */
        while (0u == (in_w(IIC_RAW_INTR_STAT) &
                      (IIC_INT_TX_EMPTY | IIC_INT_TX_ABRT)))
        {
            if (iic_late(start))
            {
                /* Look once more: this task may have been pre-empted
                 * after the byte had already gone (car, 17 Nov). */
                b_late = (0u == (in_w(IIC_RAW_INTR_STAT) &
                                 (IIC_INT_TX_EMPTY | IIC_INT_TX_ABRT)));
                break;
            }
        }

        if (!b_late)
        {
            reason = in_w(IIC_TX_ABRT_SOURCE);

            if (0u != reason)
            {
                (void)in_w(IIC_CLR_TX_ABRT);    /* Read clears it. */
                b_abort = true;
            }

            if (b_abort || (b_last && !b_nostop))
            {
                while (0u == (in_w(IIC_RAW_INTR_STAT) & IIC_INT_STOP_DET))
                {
                    if (iic_late(start))
                    {
                        b_late = (0u == (in_w(IIC_RAW_INTR_STAT) &
                                         IIC_INT_STOP_DET));
                        break;
                    }
                }

                if (!b_late)
                {
                    (void)in_w(IIC_CLR_STOP_DET);
                }
            }
        }

        b_abort = b_abort || b_late;
    }

    if (b_late)
    {
        result = E_TMOUT;
        iic_report("write TIMEOUT", addr, i, len, start, reason);
        iic_recover();
        b_nostop = false;
    }
    else if (b_abort)
    {
        result = E_IO;      /* Address or data not acknowledged. */
        iic_report("write NACK", addr, i, len, start, reason);
        b_nostop = false;   /* The controller has sent a STOP. */
    }
    else
    {
        /* Every byte acknowledged. */
    }

    g_iic_restart = b_nostop;

    return (result);
}

/*!
 * @brief i2c_read_blocking_internal(): one read command per byte; STOP
 *        on the last, so the controller does not acknowledge it.
 *
 * @return E_OK, E_IO (not acknowledged) or E_TMOUT.
 */
static ER
iic_read (uint8_t addr, uint8_t * p_dst, uint8_t len, bool b_nostop)
{
    uint32_t start   = in_w(TIMER_TIMERAWL);
    bool     b_abort = false;
    bool     b_late  = false;
    uint8_t  i       = 0u;
    ER       result  = E_OK;

    out_w(IIC_ENABLE, 0u);
    out_w(IIC_TAR, (UW)addr);
    out_w(IIC_ENABLE, IIC_ENABLE_ON);

    for (i = 0u; (i < len) && !b_abort; i++)
    {
        bool b_first = (0u == i);
        bool b_last  = ((len - 1u) == i);
        UW   cmd     = IIC_CMD_READ;

        start = in_w(TIMER_TIMERAWL);   /* The limit is per byte. */

        if (b_first && g_iic_restart)
        {
            cmd |= IIC_CMD_RESTART;
        }

        if (b_last && !b_nostop)
        {
            cmd |= IIC_CMD_STOP;
        }

        while ((IIC_TX_FIFO_DEPTH - in_w(IIC_TXFLR)) == 0u)
        {
            if (iic_late(start))
            {
                b_late = ((IIC_TX_FIFO_DEPTH - in_w(IIC_TXFLR)) == 0u);
                break;
            }
        }

        if (!b_late)
        {
            out_w(IIC_DATA_CMD, cmd);

            while (0u == in_w(IIC_RXFLR))
            {
                if (0u != (in_w(IIC_RAW_INTR_STAT) & IIC_INT_TX_ABRT))
                {
                    (void)in_w(IIC_CLR_TX_ABRT);
                    b_abort = true;
                    break;
                }

                if (iic_late(start))
                {
                    /* Car, 17 Nov: "read TIMEOUT ... rxflr 1": the byte
                     * had arrived while this task was pre-empted. */
                    b_late = (0u == in_w(IIC_RXFLR));
                    break;
                }
            }
        }

        b_abort = b_abort || b_late;

        if (!b_abort)
        {
            p_dst[i] = (uint8_t)in_w(IIC_DATA_CMD);
        }
    }

    if (b_late)
    {
        result = E_TMOUT;
        iic_report("read TIMEOUT", addr, i, len, start, 0u);
        iic_recover();
        b_nostop = false;
    }
    else if (b_abort)
    {
        result = E_IO;
        iic_report("read NACK", addr, i, len, start, 0u);
        b_nostop = false;
    }
    else
    {
        /* All bytes read. */
    }

    g_iic_restart = b_nostop;

    return (result);
}

/*** end of file ***/
