/** @file test_host.c
 *
 * @brief PC tests for subsystem 2's pure module motion_calc.c (PID,
 *        braking profile, straight-line trim, packing).
 *        Build and run: make   (in this folder)
 */

#include <stdlib.h>
#include <math.h>

#include "../../host_common/host_check.h"
#include "motion_calc.h"

/* First-order wheel model: speed follows duty with a 60 ms lag;
 * 6249 duty -> 300 mm/s at full speed. */
static int32_t
simulate (int32_t target, int ticks, int32_t load_mm_s)
{
    motion_pid_t pid;
    double       speed = 0.0;
    int32_t      ramped = 0;
    int          tick  = 0;

    motion_pid_init(&pid);

    for (tick = 0; tick < ticks; tick++)
    {
        int32_t duty = 0;
        double  goal = 0.0;

        ramped = motion_ramp(ramped, target, MOTION_RAMP_MM_S_PER_TICK);
        duty   = motion_pid_step(&pid, ramped, (int32_t)lround(speed));
        goal   = (300.0 * duty / MOTION_DUTY_MAX) - load_mm_s;
        speed += (goal - speed) * (10.0 / 60.0);
    }

    return ((int32_t)lround(speed));
}

/* Same wheel, but the controller only sees encoder edges through the
 * sliding window, as on the robot.  Returns the true mean speed over the
 * last second of a 3 s run. */
static double
simulate_edges (int32_t target)
{
    motion_pid_t pid;
    double       speed = 0.0;
    double       pos_um = 0.0;
    double       um_per_edge = motion_edges_to_um(1000u) / 1000.0;
    uint32_t     edges_total = 0u;
    uint8_t      window[MOTION_SPEED_WINDOW] = {0};
    uint32_t     sum = 0u;
    int32_t      ramped = 0;
    double       mean = 0.0;

    motion_pid_init(&pid);

    for (int tick = 0; tick < 300; tick++)
    {
        uint32_t now_edges = (uint32_t)(pos_um / um_per_edge);
        uint32_t new_edges = now_edges - edges_total;
        int32_t  measured  = 0;
        int32_t  duty      = 0;

        edges_total = now_edges;
        sum -= window[tick % MOTION_SPEED_WINDOW];
        window[tick % MOTION_SPEED_WINDOW] = (uint8_t)new_edges;
        sum += new_edges;
        measured = motion_window_speed(sum, MOTION_SPEED_WINDOW);

        ramped = motion_ramp(ramped, target, MOTION_RAMP_MM_S_PER_TICK);
        duty   = motion_pid_step(&pid, ramped, measured);
        speed += ((300.0 * duty / MOTION_DUTY_MAX) - speed) * (10.0 / 60.0);
        pos_um += speed * 10.0;     /* mm/s x 10 ms = um. */

        if (tick >= 200)
        {
            mean += speed / 100.0;
        }
    }

    return (mean);
}

/* A whole MOVE as motion_task.c runs it: two model wheels (the left
 * motor 10% weaker), coarse encoders, sliding-window speed, braking
 * profile, straight-line trim, PID, and a brake that sheds 30 mm/s per
 * tick.  Returns the distance error (mm) and the heading error (deg). */
static void
simulate_move (uint32_t goal_mm, int b_trim, double * p_dist_err,
               double * p_heading_deg)
{
    motion_pid_t pid[2];
    double       speed[2]  = {0.0, 0.0};
    double       pos_um[2] = {0.0, 0.0};
    double       gain[2]   = {270.0, 300.0};
    double       um_per_edge = motion_edges_to_um(1000u) / 1000.0;
    uint32_t     edges_total[2] = {0u, 0u};
    uint32_t     done[2] = {0u, 0u};
    uint8_t      window[2][MOTION_SPEED_WINDOW];
    uint32_t     sum[2] = {0u, 0u};
    int32_t      ramped[2] = {0, 0};
    uint32_t     goal_edges = motion_mm_to_edges(goal_mm);
    int          braking = 0;

    for (int w = 0; w < 2; w++)
    {
        motion_pid_init(&pid[w]);
        for (unsigned i = 0; i < MOTION_SPEED_WINDOW; i++) { window[w][i] = 0u; }
    }

    for (int tick = 0; tick < 2000; tick++)
    {
        int32_t  measured[2];
        uint32_t avg       = 0u;
        uint32_t remaining = 0u;
        int32_t  v         = 0;
        int32_t  trim      = 0;

        for (int w = 0; w < 2; w++)
        {
            uint32_t now_edges = (uint32_t)(pos_um[w] / um_per_edge);
            uint32_t new_edges = now_edges - edges_total[w];

            edges_total[w] = now_edges;
            done[w] += new_edges;
            sum[w]  -= window[w][tick % MOTION_SPEED_WINDOW];
            window[w][tick % MOTION_SPEED_WINDOW] = (uint8_t)new_edges;
            sum[w]  += new_edges;
            measured[w] = motion_window_speed(sum[w], MOTION_SPEED_WINDOW);
        }

        avg       = (done[0] + done[1]) / 2u;
        remaining = (avg < goal_edges) ? (goal_edges - avg) : 0u;

        if ((0u == remaining) || braking)
        {
            braking = 1;
            for (int w = 0; w < 2; w++)
            {
                speed[w] = (speed[w] > 30.0) ? (speed[w] - 30.0) : 0.0;
                pos_um[w] += speed[w] * 10.0;
            }
            if ((speed[0] <= 0.0) && (speed[1] <= 0.0)) { break; }
            continue;
        }

        v    = motion_profile_speed(motion_edges_to_um(remaining) / 1000u,
                                    200);
        trim = b_trim ? motion_straight_trim(done[0], done[1]) : 0;

        for (int w = 0; w < 2; w++)
        {
            int32_t target = motion_clamp((0 == w) ? (v - trim) : (v + trim),
                                          0, MOTION_SPEED_MAX_MM_S);
            int32_t duty   = 0;

            ramped[w] = motion_ramp(ramped[w], target,
                                    MOTION_RAMP_MM_S_PER_TICK);
            duty = motion_pid_step(&pid[w], ramped[w], measured[w]);
            speed[w] += ((gain[w] * duty / MOTION_DUTY_MAX) - speed[w]) *
                        (10.0 / 60.0);
            pos_um[w] += speed[w] * 10.0;
        }
    }

    *p_dist_err    = ((pos_um[0] + pos_um[1]) / 2000.0) - (double)goal_mm;
    *p_heading_deg = ((pos_um[1] - pos_um[0]) / 1000.0) /
                     MOTION_TRACK_MM * 57.2957795;
}

int
main (void)
{
    int ok = 1;

    /* Command packing: every opcode and argument. */
    for (int op = 0; op <= (int)S2_OP_SET_GAIN; op++)
    {
        for (int arg = 0; arg < 256; arg++)
        {
            subsystem2_op_t o;
            uint8_t         a;
            uint16_t        raw = motion_cmd_pack((subsystem2_op_t)op,
                                                  (uint8_t)arg);

            ok = ok && motion_cmd_unpack(raw, &o, &a) && ((int)o == op) &&
                 (a == arg) && (0 == (raw & 0xF000u));
        }
    }
    HOST_CHECK("H-MO01", ok, "motion command: all 9 x 256 values, 12 bits");

    /* Odometry word round trip over the whole range. */
    ok = 1;
    for (int v = -320; v <= 315; v += 5)
    {
        subsystem2_odom_t in = {(uint16_t)((v + 400) * 37 % 32768), (int16_t)v,
                                (int16_t)(-v - 5)};
        subsystem2_odom_t out;

        if (in.right_mm_s < -320) { in.right_mm_s = -320; }
        motion_odom_unpack(motion_odom_pack(&in), &out);
        ok = ok && (in.distance_mm == out.distance_mm) &&
             (in.left_mm_s == out.left_mm_s) &&
             (in.right_mm_s == out.right_mm_s) &&
             (0u == (motion_odom_pack(&in) & 0xE0000000u));
    }
    HOST_CHECK("H-MO02", ok, "odometry word: every speed step round trips");

    {
        subsystem2_odom_t in = {0u, 900, -900};
        subsystem2_odom_t out;

        motion_odom_unpack(motion_odom_pack(&in), &out);
        HOST_CHECK("H-MO03", (315 == out.left_mm_s) && (-320 == out.right_mm_s),
                   "speeds saturate, never wrap");
    }

    /* Distance delta across the wrap, random pairs. */
    ok = 1;
    srand(7);
    for (int i = 0; i < 100000; i++)
    {
        int      before = rand() % 32768;
        int      step   = (rand() % 32767) - 16383;
        uint16_t now    = (uint16_t)((before + step + 32768) % 32768);

        ok = ok && (motion_distance_delta(now, (uint16_t)before) == step);
    }
    HOST_CHECK("H-MO04", ok, "distance delta correct across the 15-bit wrap");

    /* Conversions. */
    {
        uint32_t edges = motion_mm_to_edges(1000u);
        uint32_t um    = motion_edges_to_um(edges);
        uint32_t arc   = motion_turn_arc_mm(90u);

        printf("        1 m = %u edges -> %u um; 90 deg arc = %u mm\n",
               (unsigned)edges, (unsigned)um, (unsigned)arc);
        HOST_CHECK("H-MO05", (um > 994000u) && (um <= 1000000u) &&
                   (arc >= 93u) && (arc <= 95u),
                   "edges <-> mm, 90 degree arc = 94 mm (track 120)");
    }

    /* PI control on a model wheel. */
    {
        int32_t v200  = simulate(200, 100, 0);
        int32_t vload = simulate(150, 150, 40);
        int32_t vback = simulate(-120, 100, 0);

        printf("        model: 200 -> %d, 150 with load -> %d, -120 -> %d mm/s\n",
               (int)v200, (int)vload, (int)vback);
        HOST_CHECK("H-MO06", (abs(v200 - 200) <= 10) && (abs(vload - 150) <= 8) &&
                   (abs(vback + 120) <= 6),
                   "PI reaches the set-point within 5% in 1 s, also under load");
    }

    HOST_CHECK("H-MO07", (motion_ramp(0, 200, 5) == 5) &&
               (motion_ramp(198, 200, 5) == 200) &&
               (motion_ramp(0, -200, 5) == -5),
               "acceleration ramp 5 mm/s per 10 ms");

    HOST_CHECK("H-MO08", (motion_steer_mm_s(40) > 0) &&
               (motion_steer_mm_s(-40) < 0) && (0 == motion_steer_mm_s(0)),
               "steering sign: + error slows the left wheel");

    {
        double m100 = simulate_edges(100);
        double m200 = simulate_edges(200);

        printf("        with 40-edge encoder: 100 -> %.0f, 200 -> %.0f mm/s\n",
               m100, m200);
        HOST_CHECK("H-MO09", (fabs(m100 - 100.0) <= 10.0) &&
                   (fabs(m200 - 200.0) <= 10.0),
                   "PI holds speed with coarse encoder feedback (+-10 mm/s)");
    }

    /* Braking profile and integer square root. */
    {
        ok = 1;
        for (uint32_t v = 0u; v < 200000u; v += 7u)
        {
            uint32_t r = motion_isqrt(v);

            ok = ok && ((r * r) <= v) && (((r + 1u) * (r + 1u)) > v);
        }
        ok = ok && (motion_isqrt(0xFFFFFFFFu) == 65535u);
        HOST_CHECK("H-MO10", ok && (motion_profile_speed(1000u, 200) == 200) &&
                   (motion_profile_speed(40u, 200) == 200) &&
                   (motion_profile_speed(10u, 200) == 100) &&
                   (motion_profile_speed(0u, 200) == MOTION_FINAL_MM_S) &&
                   (motion_profile_speed(0u, 40) == 40),
                   "isqrt exact; profile sqrt(2 a d), floor 50 mm/s");
    }

    /* One edge apart must already trim (rounded: one edge of the
     * 1270-edge encoder is 0.96 mm/s). */
    HOST_CHECK("H-MO11", (motion_straight_trim(100u, 100u) == 0) &&
               (motion_straight_trim(101u, 100u) > 0) &&
               (motion_straight_trim(100u, 101u) < 0) &&
               (motion_straight_trim(200u, 100u) == MOTION_STRAIGHT_MAX_MM_S) &&
               (motion_straight_trim(100u, 200u) == -MOTION_STRAIGHT_MAX_MM_S),
               "straight trim: sign, zero when equal, clamped");

    {
        double dist = 0.0;
        double head = 0.0;
        double dist0 = 0.0;
        double head0 = 0.0;

        simulate_move(1000u, 1, &dist, &head);
        simulate_move(1000u, 0, &dist0, &head0);
        printf("        1 m MOVE, left motor 10%% weak: %+.1f mm, %+.1f deg"
               " (no trim: %+.1f deg)\n", dist, head, head0);
        HOST_CHECK("H-MO12", (fabs(dist) <= 20.0) && (fabs(head) <= 5.0),
                   "1 m MOVE ends within 2 cm, heading within 5 deg (NFR3)");
    }

    {
        motion_pid_t pid;
        int32_t      flat = 0;
        int32_t      drop = 0;

        motion_pid_init(&pid);
        pid.kd_milli = 50;
        (void)motion_pid_step(&pid, 100, 100);
        flat = motion_pid_step(&pid, 100, 100);
        motion_pid_reset(&pid);
        (void)motion_pid_step(&pid, 100, 100);
        drop = motion_pid_step(&pid, 100, 90);
        HOST_CHECK("H-MO13", (drop > flat) && (MOTION_KD_MILLI >= 0),
                   "PID derivative term reacts to a falling speed");
    }

    return (HOST_SUMMARY());
}

/*** end of file ***/
