/** @file test_host.c
 *
 * @brief PC tests for subsystem 3 as integrated on 26 Oct: Zi Siang's
 *        terrain maths (imu.c) driven through the adapter (imu_glue.c).
 *
 * The drive model is independent of his own generator: it produces what
 * a real LSM303 reports (specific force: nose up -> forward reading
 * rises, speeding up -> forward reading rises, level -> +1 g on Z), and
 * the odometry exactly as subsystem 2 publishes it (5 mm/s speed steps,
 * whole-millimetre distance).  So these tests also check the sign
 * convention between the real sensor and his model.
 *
 * Nov 2026 (lecturer's change): the collision tests H-IM06..08 and the
 * 2 Nov pitch pickup H-IM18 are retired; H-IM20..25 test the lift and
 * overturn stop, and every other test also checks for no false lift.
 */

#include <stdlib.h>
#include <math.h>

#include "../../host_common/host_check.h"
#include "imu_math.h"
#include "imu.h"
#include "imu_glue.h"

typedef double (*ground_fn_t)(double x_mm);

typedef struct
{
    imu_glue_t glue;
    uint8_t    events;
    int16_t    min_pitch_q;
    int16_t    max_pitch_q;
    uint8_t    motions;         /* Bit per subsystem3_motion_t seen. */
    bool       b_active_seen;
} run_t;

static double g_hump_len = 0.0;    /* Ramp length, mm.  */
static double g_hump_deg = 0.0;
static double g_wedge_deg = 0.0;

static double
noise (double sigma)
{
    double u1 = (rand() + 1.0) / ((double)RAND_MAX + 2.0);
    double u2 = (rand() + 1.0) / ((double)RAND_MAX + 2.0);

    return (sigma * sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2));
}

static double
ground_hump (double x)
{
    /* Flat 300 mm, ramp up, flat top 60 mm, ramp down, flat. */
    double start = 300.0;
    double ramp  = g_hump_len;

    if ((x > start) && (x <= start + ramp)) { return (g_hump_deg); }
    if ((x > start + ramp + 60.0) && (x <= start + (2.0 * ramp) + 60.0))
    {
        return (-g_hump_deg);
    }
    return (0.0);
}

static double
ground_flat (double x)
{
    (void)x;
    return (0.0);
}

static double
ground_wedge (double x)
{
    return ((x > 200.0) ? g_wedge_deg : 0.0);
}

static void
run_reset (run_t * p_run)
{
    imu_init();
    imu_glue_init(&p_run->glue);
    p_run->events        = 0u;
    p_run->min_pitch_q   = 127;
    p_run->max_pitch_q   = -128;
    p_run->motions       = 0u;
    p_run->b_active_seen = false;
}

/* One 10 ms step through glue + imu.c, exactly as imu_task.c does it. */
static void
step (run_t * p_run, imu_glue_in_t const * p_in, bool b_turning)
{
    IMU_SAMPLE          sample;
    IMU_STATUS          imu;
    subsystem3_status_t status;

    imu_glue_sample(&p_run->glue, p_in, &sample);
    imu_update(&sample);
    imu_snapshot(&imu);
    p_run->events |= imu_glue_events(&p_run->glue, &imu, imu_hump_count());
    imu_glue_status(&imu, b_turning, false, S3_HEADING_NONE, &status);

    if (status.pitch_q < p_run->min_pitch_q) { p_run->min_pitch_q = status.pitch_q; }
    if (status.pitch_q > p_run->max_pitch_q) { p_run->max_pitch_q = status.pitch_q; }
    p_run->motions |= (uint8_t)(1u << status.motion);
    p_run->b_active_seen = p_run->b_active_seen || status.b_hump_active;
}

/* Drive a ground profile at a given speed (optionally braking or
 * accelerating at accel_mm_s2 after brake_at_mm, optionally turning). */
static void
drive (run_t * p_run, ground_fn_t ground, double speed_mm_s,
       double length_mm, double noise_g, int brake_at_mm,
       double accel_mm_s2, int turn)
{
    double x      = 0.0;
    double v      = speed_mm_s;
    double v_prev = v;
    double dist_q = 0.0;

    while (x < length_mm)
    {
        imu_glue_in_t in;
        double        pitch = ground(x) * M_PI / 180.0;
        double        a_fwd = 0.0;
        double        new_q = 0.0;

        if ((brake_at_mm > 0) && (x > brake_at_mm))
        {
            v += accel_mm_s2 * 0.01;
            if (v < 20.0)  { v = 20.0; }
            if (v > 300.0) { v = 300.0; }
        }

        a_fwd  = (v - v_prev) / 0.01 / 9806.65;
        v_prev = v;

        /* What the LSM303 reports (specific force), in robot axes. */
        in.fwd_mg  = (int16_t)lround(1000.0 * (sin(pitch) + a_fwd +
                                               noise(noise_g)));
        in.left_mg = (int16_t)lround(1000.0 * noise(noise_g));
        in.up_mg   = (int16_t)lround(1000.0 * cos(pitch));

        /* What subsystem 2 publishes. */
        in.left_mm_s  = (int16_t)(5 * lround((v - (turn ? 50 : 0)) / 5.0));
        in.right_mm_s = (int16_t)(5 * lround((v + (turn ? 50 : 0)) / 5.0));
        x    += v * 0.01 * cos(pitch);
        new_q = floor(x);
        in.delta_mm     = (int16_t)(new_q - dist_q);
        in.b_odom_valid = true;
        dist_q = new_q;

        step(p_run, &in, imu_glue_turning(in.left_mm_s, in.right_mm_s));
    }
}

/* From standstill (0.5 s), accelerate at a_mm_s2 to 250 mm/s, cruise 1 s. */
static void
launch (run_t * p_run, double a_mm_s2)
{
    double        v      = 0.0;
    double        dist   = 0.0;
    double        dist_q = 0.0;
    int           cruise = 0;
    imu_glue_in_t still  = {0, 0, 1000, 0, 0, 0, true};

    /* Standstill first: fills the encoder window with zeros. */
    for (int i = 0; i < 50; i++) { step(p_run, &still, false); }

    while (cruise < 100)
    {
        imu_glue_in_t in;
        double        v_new = v + (a_mm_s2 * 0.01);

        if (v_new >= 250.0)
        {
            v_new = 250.0;
            cruise++;
        }

        in.fwd_mg  = (int16_t)lround(1000.0 * ((v_new - v) / 0.01 / 9806.65));
        in.left_mg = 0;
        in.up_mg   = 1000;
        in.left_mm_s  = (int16_t)(5 * lround(v_new / 5.0));
        in.right_mm_s = in.left_mm_s;
        dist  += v_new * 0.01;
        in.delta_mm     = (int16_t)(floor(dist) - dist_q);
        in.b_odom_valid = true;
        dist_q = floor(dist);
        v      = v_new;

        step(p_run, &in, false);
    }
}

/* Hold one constant input for n samples. */
static void
hold (run_t * p_run, int16_t fwd_mg, int16_t left_mg, int16_t speed, int n)
{
    imu_glue_in_t in = {fwd_mg, left_mg, 1000, speed, speed,
                        (int16_t)(speed / 100), true};

    for (int i = 0; i < n; i++)
    {
        step(p_run, &in, false);
    }
}

/* Lift by hand, in the world frame: the car stands (or drives with the
 * wheels turning at speed) for 1.5 s, then a vertical acceleration a_mg
 * for push_n samples, the same braking to hold it, then held in the air;
 * the hand may tip the nose up to tip_deg during the push.  50 mg rms
 * vibration on every axis (motors on).  Returns the sample at which
 * GLUE_EVT_LIFTED first appeared, counted from the start of the push
 * (-1 = never). */
static int
lift (run_t * p_run, double a_mg, int push_n, double tip_deg, int16_t speed)
{
    int found = -1;

    for (int i = 0; i < 150 + (4 * push_n); i++)
    {
        imu_glue_in_t in;
        double        vert = 1000.0;
        double        tip  = 0.0;
        int           k    = i - 150;

        if ((k >= 0) && (k < push_n))           { vert += a_mg; }
        else if ((k >= push_n) && (k < 2 * push_n)) { vert -= a_mg; }
        if (k >= push_n)  { tip = tip_deg; }
        else if (k >= 0)  { tip = tip_deg * k / push_n; }
        tip *= M_PI / 180.0;

        in.fwd_mg  = (int16_t)lround((vert * sin(tip)) + 1000.0 * noise(0.05));
        in.left_mg = (int16_t)lround(1000.0 * noise(0.05));
        in.up_mg   = (int16_t)lround((vert * cos(tip)) + 1000.0 * noise(0.05));
        in.left_mm_s  = speed;
        in.right_mm_s = speed;
        in.delta_mm   = (int16_t)(speed / 100);
        in.b_odom_valid = true;
        step(p_run, &in, false);

        if ((found < 0) && (0u != (p_run->events & GLUE_EVT_LIFTED)))
        {
            found = k;
        }
    }

    return (found);
}

/* Roll the car over sideways to roll_deg in n samples, hold it there for
 * hold_n samples, then roll it back (up_sign -1: module mounted upside
 * down, so upright reads -1 g on Z). */
static void
roll_over (run_t * p_run, double roll_deg, int n, int hold_n, int up_sign)
{
    imu_glue_in_t in = {0, 0, (int16_t)(1000 * up_sign), 0, 0, 0, true};

    for (int i = 0; i < 150; i++) { step(p_run, &in, false); }   /* arm */

    for (int i = 0; i <= n + hold_n + n; i++)
    {
        double r = (i <= n) ? (roll_deg * i / n) :
                   (i <= n + hold_n) ? roll_deg :
                   (roll_deg * (n + hold_n + n - i) / n);

        r *= M_PI / 180.0;
        in.left_mg = (int16_t)lround(1000.0 * sin(r));
        in.up_mg   = (int16_t)lround(1000.0 * cos(r) * up_sign);
        step(p_run, &in, false);
    }
}

int
main (void)
{
    int   ok = 1;
    run_t run;

    srand(2004u);

    /* H-IM01: maths helpers against libm (heading still uses atan2). */
    {
        double err_atan = 0.0;
        double err_asin = 0.0;
        double err_sqrt = 0.0;

        for (double a = -M_PI; a <= M_PI; a += 0.001)
        {
            double e = fabs(imu_atan2f((float)sin(a) * 3.0f,
                                       (float)cos(a) * 3.0f) - a);

            if (e > M_PI) { e = fabs(e - 2.0 * M_PI); }
            err_atan = fmax(err_atan, e);
        }

        for (double s = -1.0; s <= 1.0; s += 0.0005)
        {
            err_asin = fmax(err_asin, fabs(imu_asinf((float)s) - asin(s)));
        }

        for (double v = 0.0; v < 5000.0; v += 0.37)
        {
            err_sqrt = fmax(err_sqrt, fabs(imu_sqrtf((float)v) - sqrt(v)) /
                                      (sqrt(v) + 1e-3));
        }

        printf("        max error: atan2 %.3f deg, asin %.3f deg, sqrt %.1e\n",
               err_atan * 180.0 / M_PI, err_asin * 180.0 / M_PI, err_sqrt);
        HOST_CHECK("H-IM01", (err_atan * 180.0 / M_PI < 0.1) &&
                   (err_asin * 180.0 / M_PI < 0.1) && (err_sqrt < 1e-4),
                   "atan2/asin within 0.1 deg, sqrt within 1e-4");
    }

    /* H-IM02/03: humps of known height at 12, 15 and 20 cm/s. */
    {
        static double const heights[3] = {20.0, 40.0, 55.0};
        static double const speeds[3]  = {120.0, 150.0, 200.0};
        double worst = 0.0;

        ok = 1;
        for (int h = 0; h < 3; h++)
        {
            for (int s = 0; s < 3; s++)
            {
                g_hump_deg = 12.0;
                g_hump_len = heights[h] / sin(g_hump_deg * M_PI / 180.0);
                run_reset(&run);
                drive(&run, ground_hump, speeds[s],
                      900.0 + (2.0 * g_hump_len), 0.02, 0, 0.0, 0);

                if (0u == (run.events & GLUE_EVT_HUMP_REPORT))
                {
                    ok = 0;
                    printf("        no report: %.0f mm at %.0f mm/s\n",
                           heights[h], speeds[s]);
                }
                else
                {
                    double err = fabs(run.glue.last_mm - heights[h]) /
                                 heights[h];

                    worst = fmax(worst, err);
                    printf("        %2.0f mm hump at %3.0f mm/s -> %u mm\n",
                           heights[h], speeds[s], (unsigned)run.glue.last_mm);
                }

                ok = ok && (0u == (run.events & GLUE_EVT_LIFTED)) &&
                     run.b_active_seen &&
                     (0u != (run.events & GLUE_EVT_HUMP_START)) &&
                     (0u != (run.events & GLUE_EVT_HUMP_END));
            }
        }

        HOST_CHECK("H-IM02", ok && (worst <= 0.15),
                   "hump height within +-15% (NFR9), start/end edges, "
                   "no false lift");

        /* Two humps in one run: max kept, last is the second one. */
        run_reset(&run);
        g_hump_deg = 12.0;
        g_hump_len = 50.0 / sin(12.0 * M_PI / 180.0);
        drive(&run, ground_hump, 150.0, 900.0 + (2.0 * g_hump_len), 0.02,
              0, 0.0, 0);
        g_hump_len = 20.0 / sin(12.0 * M_PI / 180.0);
        drive(&run, ground_hump, 150.0, 900.0 + (2.0 * g_hump_len), 0.02,
              0, 0.0, 0);
        printf("        two humps -> last %u mm, max %u mm\n",
               (unsigned)run.glue.last_mm, (unsigned)run.glue.max_mm);
        HOST_CHECK("H-IM03", (run.glue.max_mm >= 45u) &&
                   (run.glue.last_mm <= 25u),
                   "run maximum kept, last hump reported");
    }

    /* H-IM04: flat ground with hard braking: no hump, no lift. */
    run_reset(&run);
    drive(&run, ground_flat, 200.0, 1500.0, 0.02, 600, -250.0, 0);
    HOST_CHECK("H-IM04", 0u == (run.events & (GLUE_EVT_HUMP_START |
                                              GLUE_EVT_LIFTED)),
               "braking on flat ground: no hump, no lift");

    /* H-IM22..24 (Nov 2026, lift; H-IM06..08 were the collision tests and
     * are retired).  Through the glue with the independent world-frame
     * model above, 50 mg rms vibration. */
    {
        int at = 0;

        run_reset(&run);
        at = lift(&run, 200.0, 30, 0.0, 120);
        printf("        lift 0.2 g for 0.3 s, wheels turning: event after "
               "%d ms\n", at * 10);
        HOST_CHECK("H-IM22", (at >= 0) && (at <= 30),
                   "lifted by hand -> LIFTED within 0.3 s, wheels still "
                   "turning");

        run_reset(&run);
        at = lift(&run, 100.0, 40, 30.0, 0);
        printf("        gentle lift 0.1 g for 0.4 s, nose tipped 30 deg: "
               "event after %d ms\n", at * 10);
        HOST_CHECK("H-IM23", (at >= 0) && (at <= 40),
                   "gentle lift with a 30 deg tip -> LIFTED");
    }

    /* H-IM24: knocks are not lifts: +1.5 g / -1.5 g for 20 ms each, every
     * 0.3 s for 3 s (a hump edge, a seam), wheels turning. */
    {
        imu_glue_in_t in = {0, 0, 1000, 150, 150, 1, true};

        run_reset(&run);
        for (int i = 0; i < 450; i++)
        {
            int k = (i >= 150) ? ((i - 150) % 30) : 99;

            in.up_mg = (int16_t)((k < 2) ? 2500 : (k < 4) ? -500 : 1000);
            step(&run, &in, false);
        }
        HOST_CHECK("H-IM24", 0u == (run.events & GLUE_EVT_LIFTED),
                   "20 ms knocks of 1.5 g every 0.3 s are not a lift");
    }

    /* H-IM09/10: packing and heading. */
    {
        subsystem3_status_t a = {-128, 359u, S3_HUMP_INVALID,
                                 S3_MOTION_CRUISING, true};
        subsystem3_status_t b;

        ok = 1;
        for (int q = -128; q <= 127; q++)
        {
            a.pitch_q = (int16_t)q;
            imu_status_unpack(imu_status_pack(&a), &b);
            ok = ok && (b.pitch_q == q) && (b.heading_deg == 359u) &&
                 (b.hump == S3_HUMP_INVALID) &&
                 (b.motion == S3_MOTION_CRUISING) && b.b_hump_active;
        }
        HOST_CHECK("H-IM09", ok && (imu_glue_pitch_q(4000) == 127) &&
                   (imu_glue_pitch_q(-4000) == -128) &&
                   (imu_glue_pitch_q(310) == 12) &&
                   (imu_glue_pitch_q(-310) == -12),
                   "status word round trip; pitch saturates at +-32 deg");
        HOST_CHECK("H-IM10", (imu_heading_deg(1.0f, 0.0f) == 0u) &&
                   (imu_heading_deg(0.0f, 1.0f) == 89u ||
                    imu_heading_deg(0.0f, 1.0f) == 90u) &&
                   (imu_heading_deg(-1.0f, -0.01f) >= 179u),
                   "heading 0..359");
    }

    /* H-IM12: sign convention.  A real sensor tilted nose-up 10 deg reads
     * fwd = +174 mg; through the glue the pitch must be +10 deg.  Fed
     * straight into imu.c (no glue), it comes out -10 deg. */
    {
        IMU_SAMPLE raw = {174, 0, 985, 0, 0u, 1u, 0u};
        int16_t    direct = 0;

        run_reset(&run);
        hold(&run, 174, 0, 0, 50);
        imu_init();
        for (int i = 0; i < 50; i++) { imu_update(&raw); }
        direct = imu_pitch_cdeg();
        printf("        10 deg nose up: glue %+.2f deg, raw %+.2f deg\n",
               run.max_pitch_q * 0.25, direct / 100.0);
        HOST_CHECK("H-IM12", (run.max_pitch_q >= 38) &&
                   (run.max_pitch_q <= 42) && (direct < -900),
                   "nose up = + pitch through the glue (raw sign inverted)");
    }

    /* H-IM13: launch at 1 m/s^2 on flat ground: the encoder acceleration
     * cancels the car's own 100 mg (uncorrected it would read ~6 deg). */
    run_reset(&run);
    launch(&run, 1000.0);
    printf("        1.0 m/s^2 launch: pitch %+.2f..%+.2f deg\n",
           run.min_pitch_q * 0.25, run.max_pitch_q * 0.25);
    HOST_CHECK("H-IM13", (run.min_pitch_q >= -12) && (run.max_pitch_q <= 12) &&
               (0u == (run.events & GLUE_EVT_HUMP_START)),
               "acceleration compensated: |pitch| < 3 deg, no hump");

    /* H-IM25: overturned (H-IM18, the 2 Nov 55 deg pitch pickup, is retired).  A roll to 90 deg held 0.3 s latches it; a
     * 70 deg wobble held only 0.15 s does not (200 ms rule). */
    run_reset(&run);
    roll_over(&run, 90.0, 30, 30, 1);
    {
        uint8_t ev = run.events;

        run_reset(&run);
        roll_over(&run, 70.0, 10, 15, 1);
        HOST_CHECK("H-IM25",
                   (0u != (ev & GLUE_EVT_LIFTED)) &&
                   (0u == (run.events & GLUE_EVT_LIFTED)),
                   "on its side 0.3 s -> OVERTURNED; 0.15 s wobble ignored");
    }

    /* H-IM19: a 25 deg climb for 2 s is neither a lift nor an overturn
     * (the car must not stop on its own humps), and a module mounted
     * upside down reads upright as upright (the up sign is learned). */
    {
        IMU_SAMPLE slope = {-423, 0, 906, 0, 150u, 1u, 1500u};  /* 25 deg */
        IMU_STATUS st2 = {0};
        IMU_SAMPLE level = {0, 0, 1000, 0, 150u, 1u, 1500u};
        uint8_t    ev = 0u;

        imu_init();
        for (int i = 0; i < 150; i++) { imu_update(&level); }
        for (int i = 0; i < 200; i++) { imu_update(&slope); }
        imu_snapshot(&st2);

        run_reset(&run);
        roll_over(&run, 0.0, 10, 100, -1);
        ev = run.events;
        run_reset(&run);
        roll_over(&run, 180.0, 50, 30, -1);
        HOST_CHECK("H-IM19",
                   (0u == (st2.flags & (uint8_t)IMU_FLAG_LIFTED)) &&
                   (0u == (ev & GLUE_EVT_LIFTED)) &&
                   (0u != (run.events & GLUE_EVT_LIFTED)),
                   "25 deg climb is not a lift; upside-down module: upright "
                   "ok, rolled over -> OVERTURNED");
    }

    /* H-IM20: the latch.  After a lift the flag stays set (the car must
     * not drive off in the hand) until imu_clear() re-arms it; a new lift
     * after re-arming (1 s to learn g) is a new event. */
    {
        int at = 0;

        run_reset(&run);
        (void)lift(&run, 200.0, 30, 0.0, 0);
        {
            imu_glue_in_t still = {0, 0, 1000, 0, 0, 0, true};
            IMU_STATUS    st2   = {0};
            int           held  = 1;

            for (int i = 0; i < 300; i++)
            {
                step(&run, &still, false);
                imu_snapshot(&st2);
                held = held && (0u != (st2.flags & (uint8_t)IMU_FLAG_LIFTED));
            }

            imu_clear(IMU_FLAG_LIFTED);
            run.glue.b_lift_prev = false;
            run.events = 0u;
            at = lift(&run, 200.0, 30, 0.0, 0);
            HOST_CHECK("H-IM20", held && (at >= 0) && (at <= 30),
                       "LIFTED latched 3 s, re-armed, second lift detected");
        }
    }

    /* H-IM17: a hard 2.5 m/s^2 launch: the ~25 ms lag of the encoder
     * acceleration gives a short +-6 deg transient; it may open a hump but
     * must not report one (IS25 minimum height 5 mm). */
    run_reset(&run);
    launch(&run, 2500.0);
    printf("        2.5 m/s^2 launch: pitch %+.2f..%+.2f deg, start %s\n",
           run.min_pitch_q * 0.25, run.max_pitch_q * 0.25,
           (0u != (run.events & GLUE_EVT_HUMP_START)) ? "yes" : "no");
    HOST_CHECK("H-IM17", (0u == (run.events & (GLUE_EVT_HUMP_REPORT |
                                               GLUE_EVT_LIFTED))) &&
               (0u != (run.motions & (1u << S3_MOTION_ACCELERATING))),
               "hard launch: ACCELERATING, no hump report, no lift");

    /* H-IM14: a constant 10 deg wedge (ramp, books) is not a hump, at any
     * speed (IS25: a 4 s guard expiry that never crested is dropped). */
    ok = 1;
    g_wedge_deg = 10.0;
    for (double v = 100.0; v <= 250.0; v += 50.0)
    {
        run_reset(&run);
        drive(&run, ground_wedge, v, 200.0 + (v * 7.0), 0.02, 0, 0.0, 0);
        ok = ok && (0u == (run.events & GLUE_EVT_HUMP_REPORT));
    }
    HOST_CHECK("H-IM14", ok,
               "10 deg wedge for 7 s at 10..25 cm/s: no hump report");

    /* H-IM15: motion classes through the glue: cruising, turning,
     * stationary, decelerating. */
    run_reset(&run);
    drive(&run, ground_flat, 150.0, 300.0, 0.0, 0, 0.0, 0);
    drive(&run, ground_flat, 150.0, 300.0, 0.0, 0, 0.0, 1);
    drive(&run, ground_flat, 200.0, 400.0, 0.0, 1, -2000.0, 0);
    hold(&run, 0, 0, 0, 20);
    HOST_CHECK("H-IM15",
               (0u != (run.motions & (1u << S3_MOTION_CRUISING))) &&
               (0u != (run.motions & (1u << S3_MOTION_TURNING))) &&
               (0u != (run.motions & (1u << S3_MOTION_DECELERATING))) &&
               (0u != (run.motions & (1u << S3_MOTION_STATIONARY))),
               "motion classes: cruising, turning, decelerating, stopped");

    /* H-IM16: 60 s of noisy cruising (50 mg rms, motors-on level):
     * no hump report, no lift. */
    run_reset(&run);
    drive(&run, ground_flat, 150.0, 9000.0, 0.05, 0, 0.0, 0);
    HOST_CHECK("H-IM16", 0u == (run.events & (GLUE_EVT_HUMP_REPORT |
                                              GLUE_EVT_LIFTED)),
               "60 s of 50 mg noise at 15 cm/s: no hump, no lift");

    /* H-IM21: the demo course at demo speeds, 50 mg noise: humps of 20 to
     * 50 mm at 12 and 20 deg, at 2, 7 and 12 cm/s - every hump measured
     * (the ramp guard is a distance, not 4 s), never a lift or an
     * overturn. */
    ok = 1;
    for (double deg = 12.0; deg <= 20.0; deg += 8.0)
    {
        for (double h = 20.0; h <= 50.0; h += 30.0)
        {
            for (double v = 20.0; v <= 120.0; v += 50.0)
            {
                g_hump_deg = deg;
                g_hump_len = h / sin(deg * M_PI / 180.0);
                run_reset(&run);
                drive(&run, ground_hump, v, 900.0 + (2.0 * g_hump_len), 0.05,
                      0, 0.0, 0);
                printf("        %2.0f mm at %2.0f deg, %3.0f mm/s -> %u mm%s%s\n",
                       h, deg, v, (unsigned)run.glue.last_mm,
                       (0u != (run.events & GLUE_EVT_HUMP_REPORT)) ?
                       "" : " NOT MEASURED",
                       (0u != (run.events & GLUE_EVT_LIFTED)) ?
                       ", LIFTED (wrong)" : "");
                ok = ok && (0u != (run.events & GLUE_EVT_HUMP_REPORT)) &&
                     (fabs(run.glue.last_mm - h) <= (0.15 * h)) &&
                     (0u == (run.events & GLUE_EVT_LIFTED));
            }
        }
    }
    HOST_CHECK("H-IM21", ok,
               "humps 20-50 mm, 12-20 deg, 2-12 cm/s, noisy: within "
               "+-15%, never LIFTED");

    return (HOST_SUMMARY());
}

/*** end of file ***/
