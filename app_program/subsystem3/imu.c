/*
 *	imu.c
 *	Subsystem 4 -- Physics Watchdog: terrain, hump and lift maths.
 *
 *	Nov 2026: the collision rule (impact AND speed collapse) and the
 *	2 Nov pitch-only pickup rule are REMOVED (lecturer's change).  In
 *	their place lift_step() detects the robot being lifted (an upward
 *	speed no wheel can give) or overturned (tilt over 60 degrees, any
 *	direction, or upside down).  See imu.h for the numbers.
 *
 *	Student ID: 2500989 (Zi Siang)
 *
 *	Pure C.  No kernel calls, no I/O, no floating point, no dynamic memory.
 *	Compiles unchanged on the Pico (arm-none-eabi-gcc) and on a PC (gcc),
 *	which is what lets the algorithm be tested before the sensor exists.
 *
 *	Why integers: the RP2040 core is a Cortex-M0+ with no FPU.  A 100 Hz
 *	task that called atan2()/asin() would spend tens of microseconds per
 *	sample in soft-float library code.  The two trig needs are instead:
 *	  - sin(pitch) straight from the accelerometer  (ax / g)
 *	  - pitch angle from a 64-entry asin table      (cheap, exact enough)
 *	and the lift / overturn rules need only |a|, from an integer sqrt.
 */

#include "imu.h"

#include <stddef.h>		/* NULL (freestanding build has no stdio/string) */

/* ---------------------------------------------------------------------
 *  asin table: index = |sin| >> 9  (64 buckets of 512/32768 = 0.0156),
 *  value = asin(bucket midpoint) in centi-degrees.  Generated once from
 *  the closed form and pasted here, so no code runs in the loop.  Below
 *  30 deg the worst-case error is under 0.5 deg; the last bucket is the
 *  90 deg rail.
 * --------------------------------------------------------------------- */
static const int16_t asin_cdeg_tbl[64] = {
	   45,   134,   224,   313,   403,   493,   583,   673,
	  763,   854,   944,  1035,  1126,  1218,  1309,  1402,
	 1494,  1587,  1680,  1774,  1868,  1963,  2058,  2154,
	 2251,  2348,  2446,  2545,  2644,  2745,  2846,  2948,
	 3052,  3156,  3262,  3369,  3477,  3587,  3698,  3811,
	 3926,  4042,  4161,  4282,  4405,  4531,  4660,  4792,
	 4927,  5066,  5210,  5358,  5512,  5671,  5838,  6013,
	 6198,  6395,  6607,  6839,  7096,  7393,  7757,  9000
};

/* ---------------------------------------------------------------------
 *  Module state.  Only task_imu.c writes this (single-writer rule), so
 *  no mutex is needed inside the module.
 * --------------------------------------------------------------------- */
typedef struct {
	int32_t		ax_fast;	/* 15 Hz filter: tilt / hump path */
	int32_t		ay_fast;	/* 15 Hz: overturn test           */
	int32_t		az_fast;
	int32_t		ay_slow;	/*  4 Hz: roll (report only)      */
	int32_t		sin_pitch_q15;	/* sin(pitch), Q15 */
	int16_t		pitch_cdeg;
	int16_t		roll_cdeg;

	uint8_t		hump_state;
	uint8_t		hump_enter_cnt;
	uint8_t		hump_exit_cnt;
	uint16_t	hump_samples;
	uint32_t	hump_dist_um;	/* travelled since the hump began */
	int32_t		h_run_um;	/* running integral of sin(pitch) dx */
	int32_t		h_max_um;	/* its maximum = the hump height */

	uint16_t	hump_peak_mm;
	uint16_t	hump_count;

	int32_t		gx_q8;		/* learned gravity vector,         */
	int32_t		gy_q8;		/* mg x 256 (slow filter)          */
	int32_t		gz_q8;
	int32_t		g_ref;		/* learned |g|, mg (frozen)        */
	int32_t		g_sum;		/* its average while arming        */
	int32_t		up_sign;	/* +1, or -1 if the module is upside down */
	int32_t		uz_sum;		/* z while arming, decides up_sign */
	int32_t		v_up_ums;	/* leaky upward speed, um/s       */
	int32_t		v_var_q4;	/* its noise: mean v^2, (mm/s)^2 x 16 */
	int32_t		v_thr_ums;	/* current lift threshold, um/s   */
	int16_t		up_pct;		/* up axis as % of |a|            */
	uint8_t		settle;		/* samples since (re)arming       */
	uint8_t		lift_cnt;	/* consecutive samples over v     */
	uint8_t		flip_cnt;	/* consecutive overturned samples */
	uint8_t		lift_cause;	/* IMU_LIFT_*                     */

	uint8_t		motion;
	uint8_t		flags;		/* latched: LIFTED / FAULT / SATURATED */
	uint8_t		seq;
} IMU_STATE;

static IMU_STATE st;

/* One-sided hump: the height comes from the ROLL angle, not the pitch.
 * A 10 mm strip under one wheel rolls a 113 mm track by asin(10/113)
 * = 5.1 deg - smaller than one accelerometer sample's noise - so average
 * 320 ms, subtract a slow floor baseline and convert the peak with
 * h = track * sin(roll).  Enabled by imu_hump_diag(1) (test/fix build);
 * off means the spec behaviour and the host tests are untouched. */
#define IMU_ROLL_TRACK_MM       113      /* wheel track 113.3 mm measured
                                          * centre to centre (19 Nov) */
#define IMU_ROLL_ENTER_CDEG     100      /* 1.0 deg over the baseline */
#define IMU_ROLL_EXIT_CDEG      60
#define IMU_ROLL_AVG_SHIFT      5        /* 32 samples  = 320 ms */
#define IMU_ROLL_BASE_SHIFT     9        /* 512 samples =  5 s  */
#define IMU_ROLL_ENTER_SAMPLES  8u       /* 80 ms  */
#define IMU_ROLL_EXIT_SAMPLES   60u      /* 600 ms */
#define IMU_ROLL_MIN_MM         3
#define IMU_ROLL_MAX_MM         150

static int32_t  rl_avg_q8;      /* roll, cdeg x 256, averaged    */
static int32_t  rl_base_q8;     /* floor baseline, same units   */
static int16_t  rl_peak_cdeg;   /* peak deviation of this bump  */
static int32_t  rl_peak_sin;    /* its sine, Q15                */
static uint8_t  rl_enter;
static uint16_t rl_exit;
static uint8_t  rl_active;

static void roll_hump_step(int16_t roll_cdeg, int32_t sin_q15);

/* Test build only: when true, a hump that the spec filter would drop is
 * still reported (with its real height) so the dashboard shows a number
 * instead of nothing.  Default off - imu_task.c turns it on for the car
 * test build only, so the host tests keep the spec behaviour. */
static uint8_t g_b_hump_diag = 0u;   /* roll estimator on/off. */
static uint8_t g_b_pitch_diag = 0u;  /* report pitch rejects.   */

void
imu_hump_diag(uint8_t b_on)
{
    g_b_hump_diag = b_on;
}

/* Packed words read by other tasks.  Each is written with ONE aligned
   store at the end of imu_update(), and each reader copies it once
   (week 8 "one writer, one word, no mutex" rule). */
static volatile uint32_t g_terrain_word;
static volatile uint16_t g_attitude_word;

/* ---------------------------------------------------------------------
 *  Small integer helpers
 * --------------------------------------------------------------------- */
static int32_t clamp32(int32_t v, int32_t lo, int32_t hi)
{
	if (v < lo) {
		return lo;
	}
	if (v > hi) {
		return hi;
	}
	return v;
}

static int32_t iabs32(int32_t v)
{
	return (v < 0) ? -v : v;
}

/* floor(sqrt(v)), bit by bit: 16 iterations, no division. */
static uint32_t isqrt32(uint32_t v)
{
	uint32_t	root = 0u;
	uint32_t	bit = 1uL << 30;

	while (bit > v) {
		bit >>= 2;
	}
	while (bit != 0u) {
		if (v >= root + bit) {
			v -= root + bit;
			root = (root >> 1) + bit;
		} else {
			root >>= 1;
		}
		bit >>= 2;
	}
	return root;
}

/* |(x, y, z)| in mg, inputs clamped so the squares cannot overflow. */
static int32_t norm3(int32_t x, int32_t y, int32_t z)
{
	uint32_t	sum;

	x = clamp32(x, -IMU_AXIS_LIMIT_MG, IMU_AXIS_LIMIT_MG);
	y = clamp32(y, -IMU_AXIS_LIMIT_MG, IMU_AXIS_LIMIT_MG);
	z = clamp32(z, -IMU_AXIS_LIMIT_MG, IMU_AXIS_LIMIT_MG);
	sum = (uint32_t)(x * x) + (uint32_t)(y * y) + (uint32_t)(z * z);
	return (int32_t)isqrt32(sum);
}

/* sin(pitch) in Q15 -> centi-degrees. */
static int16_t asin_cdeg(int32_t sin_q15)
{
	int32_t	mag;
	int32_t	idx;
	int16_t	val;

	mag = iabs32(sin_q15);
	if (mag > 32767) {
		mag = 32767;
	}
	if (mag == 0) {
		return 0;		/* IS25: level must read 0, not bucket 0's 0.45 deg */
	}
	idx = mag >> 9;			/* 0..63 */
	val = asin_cdeg_tbl[idx];
	return (int16_t)((sin_q15 < 0) ? -val : val);
}

/* ---------------------------------------------------------------------
 *  Public: init
 * --------------------------------------------------------------------- */
void imu_init(void)
{
	st.ax_fast = 0;
	st.ay_fast = 0;
	st.az_fast = 1000;
	st.ay_slow = 0;
	st.sin_pitch_q15 = 0;
	st.pitch_cdeg = 0;
	st.roll_cdeg = 0;

	st.hump_state = IMU_HUMP_IDLE;
	st.hump_enter_cnt = 0;
	st.hump_exit_cnt = 0;
	st.hump_samples = 0;
	st.hump_dist_um = 0;
	st.h_run_um = 0;
	st.h_max_um = 0;

	st.hump_peak_mm = 0;
	st.hump_count = 0;

	st.gx_q8 = 0;
	st.gy_q8 = 0;
	st.gz_q8 = 1000 * 256;
	st.g_ref = 1000;
	st.g_sum = 0;
	st.up_sign = 1;
	st.uz_sum = 0;
	st.v_up_ums = 0;
	st.v_var_q4 = IMU_LIFT_VAR0_Q4;
	st.v_thr_ums = IMU_LIFT_VEL_UMS;
	st.up_pct = 100;
	st.settle = 0;
	st.lift_cnt = 0;
	st.flip_cnt = 0;
	st.lift_cause = IMU_LIFT_NONE;

	st.motion = IMU_MOTION_UNKNOWN;
	st.flags = 0;
	st.seq = 0;

	g_terrain_word = 0;
	g_attitude_word = 0;
}

/* ---------------------------------------------------------------------
 *  Public: fault / clear (week 8 rule: the setter is the only clearer)
 * --------------------------------------------------------------------- */
void imu_set_fault(uint8_t on)
{
	if (on != 0u) {
		st.flags = (uint8_t)(st.flags | (uint8_t)IMU_FLAG_FAULT);
	} else {
		st.flags = (uint8_t)(st.flags & (uint8_t)~IMU_FLAG_FAULT);
	}
}

void imu_clear(uint8_t flag_mask)
{
	st.flags = (uint8_t)(st.flags & (uint8_t)~flag_mask);

	/* Clearing LIFTED re-arms the detector from a clean state: the
	   gravity reference is learned again for 1 s (the robot is normally
	   standing on the start line when this happens). */
	if ((flag_mask & (uint8_t)IMU_FLAG_LIFTED) != 0u) {
		st.v_up_ums = 0;
		st.v_var_q4 = IMU_LIFT_VAR0_Q4;
		st.v_thr_ums = IMU_LIFT_VEL_UMS;
		st.settle = 0;
		st.lift_cnt = 0;
		st.flip_cnt = 0;
		st.lift_cause = IMU_LIFT_NONE;
	}
}

/* ---------------------------------------------------------------------
 *  Hump state machine
 *
 *  Height is the PEAK of the running integral of sin(pitch) over
 *  distance, not its final value: over a complete hump the integral
 *  returns to zero (the descent cancels the climb), so only the maximum
 *  is the height of the hump.
 *
 *  Geometry note (goes in the report): the integral equals the moving
 *  average of the terrain height over the wheelbase, so it is exact for
 *  humps longer than the wheelbase and under-reads shorter ones by about
 *  (hump length / wheelbase).  Record the measured wheelbase in README.
 * --------------------------------------------------------------------- */
static void hump_step(int32_t sin_q15, uint32_t dd_um, uint8_t inhibit,
		      uint8_t may_start)
{
	int32_t	contrib_um;
	int32_t	hint;

	/* Lifted or overturned: the pitch no longer describes the track,
	   so any hump in progress is abandoned rather than reported wrong. */
	if (inhibit != 0u) {
		st.hump_state = IMU_HUMP_IDLE;
		st.hump_enter_cnt = 0;
		st.hump_exit_cnt = 0;
		st.h_run_um = 0;
		st.h_max_um = 0;
		return;
	}

	contrib_um = (sin_q15 * (int32_t)dd_um) / 32768;

	switch (st.hump_state) {
	case IMU_HUMP_IDLE:
		if ((may_start != 0u) &&
		    (st.pitch_cdeg >= (int16_t)IMU_HUMP_ENTER_CDEG)) {
			if (st.hump_enter_cnt < 255u) {
				st.hump_enter_cnt++;
			}
			if (st.hump_enter_cnt >= (uint8_t)IMU_HUMP_ENTER_SAMPLES) {
				st.hump_state = IMU_HUMP_RISING;
				st.hump_samples = 0;
				st.hump_dist_um = 0;
				st.hump_exit_cnt = 0;
				st.h_run_um = 0;
				st.h_max_um = 0;
			}
		} else {
			st.hump_enter_cnt = 0;
		}
		break;

	case IMU_HUMP_RISING:
	case IMU_HUMP_FALLING:
		st.hump_samples++;
		st.hump_dist_um += dd_um;
		st.h_run_um += contrib_um;
		if (st.h_run_um > st.h_max_um) {
			st.h_max_um = st.h_run_um;
		}

		/* Crest: pitch crosses zero going down. */
		if ((st.hump_state == IMU_HUMP_RISING) && (st.pitch_cdeg <= 0)) {
			st.hump_state = IMU_HUMP_FALLING;
		}
		/* A second rise inside one hump keeps accumulating. */
		if ((st.hump_state == IMU_HUMP_FALLING) &&
		    (st.pitch_cdeg >= (int16_t)IMU_HUMP_ENTER_CDEG)) {
			st.hump_state = IMU_HUMP_RISING;
		}

		/* Exit: back near level for 5 samples, or the 4 s guard. */
		hint = iabs32((int32_t)st.pitch_cdeg);
		if (hint < (int32_t)IMU_HUMP_EXIT_CDEG) {
			if (st.hump_exit_cnt < 255u) {
				st.hump_exit_cnt++;
			}
		} else {
			st.hump_exit_cnt = 0;
		}
		if ((st.hump_exit_cnt >= (uint8_t)IMU_HUMP_EXIT_SAMPLES) ||
		    (st.hump_dist_um >= (uint32_t)IMU_HUMP_MAX_UM) ||
		    (st.hump_samples >= (uint16_t)IMU_HUMP_MAX_SAMPLES)) {
			/* h_max_um is micrometres; the field is millimetres. */
			hint = st.h_max_um / 1000;
			if (hint < 0) {
				hint = 0;
			}
			if ((hint > IMU_HUMP_MAX_MM) || (hint < IMU_HUMP_MIN_MM) ||
			    (((st.hump_dist_um >= (uint32_t)IMU_HUMP_MAX_UM) ||
			      (st.hump_samples >= (uint16_t)IMU_HUMP_MAX_SAMPLES)) &&
			     (st.hump_state == IMU_HUMP_RISING))) {
				/* A slope, a ramp or a stack of books, not a
				   hump: drop it silently.  IS25: also a peak under
				   IMU_HUMP_MIN_MM, and a guard expiry that never
				   crested (still RISING after 600 mm: a ramp).  Nov
				   2026: the guard is a distance, not 4 s, so a slow
				   car (3 cm/s) still measures a long hump. */
				/* Test build: report the rejected peak so the
				 * dashboard shows a number instead of nothing.
				 * Off by default - imu_task.c turns it on. */
				if (0u != g_b_pitch_diag) {
					if (hint > IMU_HUMP_MAX_MM) {
						hint = IMU_HUMP_MAX_MM;
					}
					st.hump_peak_mm = (uint16_t)hint;
					st.hump_count++;
				}
				hint = 0;
			} else {
				st.hump_peak_mm = (uint16_t)hint;
				st.hump_count++;
			}
			st.hump_state = IMU_HUMP_IDLE;
			st.hump_enter_cnt = 0;
			st.hump_exit_cnt = 0;
		}
		break;

	default:
		st.hump_state = IMU_HUMP_IDLE;
		break;
	}
}

/* ---------------------------------------------------------------------
 *  Lift / overturn detector (replaces the collision rule, Nov 2026).
 *
 *  Runs on every sample, whatever the robot is doing, and LATCHES
 *  IMU_FLAG_LIFTED: only imu_clear() (TSK_IMU, when a new run starts)
 *  releases it.  A lift ends the demo run, so there is no auto-clear.
 * --------------------------------------------------------------------- */
static void lift_step(const IMU_SAMPLE *s)
{
	int32_t	gx;
	int32_t	gy;
	int32_t	gz;
	int32_t	g_dir;
	int32_t	mag_fast;
	int32_t	a_par;
	int32_t	a_up;

	/* Gravity DIRECTION: a 0.32 s filter of the reading, so it follows
	   the car rotating (a hump slope, a hand tilting it) but not a short
	   push.  A vertical push does not turn the vector at all - only
	   lengthens it - so this filter cannot "learn away" a lift. */
	if (st.settle == 0u) {
		st.gx_q8 = (int32_t)s->ax_mg * 256;
		st.gy_q8 = (int32_t)s->ay_mg * 256;
		st.gz_q8 = (int32_t)s->az_mg * 256;
		st.g_sum = 0;
		st.uz_sum = 0;
		st.v_up_ums = 0;
	} else {
		st.gx_q8 += (((int32_t)s->ax_mg * 256) - st.gx_q8) / IMU_GDIR_DIV;
		st.gy_q8 += (((int32_t)s->ay_mg * 256) - st.gy_q8) / IMU_GDIR_DIV;
		st.gz_q8 += (((int32_t)s->az_mg * 256) - st.gz_q8) / IMU_GDIR_DIV;
	}
	gx = clamp32(st.gx_q8 / 256, -IMU_AXIS_LIMIT_MG, IMU_AXIS_LIMIT_MG);
	gy = clamp32(st.gy_q8 / 256, -IMU_AXIS_LIMIT_MG, IMU_AXIS_LIMIT_MG);
	gz = clamp32(st.gz_q8 / 256, -IMU_AXIS_LIMIT_MG, IMU_AXIS_LIMIT_MG);
	g_dir = norm3(gx, gy, gz);

	/* Acceleration ALONG gravity.  A horizontal push (the car braking,
	   turning, touching an obstacle) is at right angles to g and adds
	   nothing; a rotation the filter has not caught up with only makes it
	   SMALLER (g cos d).  Only a vertical acceleration - a hand lifting
	   the chassis - makes it larger than g. */
	if (g_dir > 100) {
		a_par = ((clamp32(s->ax_mg, -IMU_AXIS_LIMIT_MG, IMU_AXIS_LIMIT_MG) * gx) +
			 (clamp32(s->ay_mg, -IMU_AXIS_LIMIT_MG, IMU_AXIS_LIMIT_MG) * gy) +
			 (clamp32(s->az_mg, -IMU_AXIS_LIMIT_MG, IMU_AXIS_LIMIT_MG) * gz)) /
			g_dir;
	} else {
		a_par = st.g_ref;	/* falling: no reference, no lift */
	}

	/* Tilt from upright: the share of |a| on the up (z) axis.  Its SIGN
	   is learned while arming, so a module mounted upside down does not
	   read as an overturned car.  Only the sign: a full "upright" vector
	   learned while the car happened to be on a slope would be wrong by
	   the slope angle. */
	mag_fast = norm3(st.ax_fast, st.ay_fast, st.az_fast);
	st.up_pct = (int16_t)((mag_fast > 100) ?
			      ((st.up_sign * clamp32(st.az_fast, -IMU_AXIS_LIMIT_MG,
						     IMU_AXIS_LIMIT_MG) * 100) /
			       mag_fast) : 100);

	/* Gravity SIZE: a constant, so it is learned once, as the average
	   |a| over the 1 s after (re)arming (normally on the start line), and
	   then frozen.  It absorbs the sensor's scale error.  |a|, because a
	   rotation cannot change it (the along-gravity reading dips while the
	   car turns over a hump, and a reference learned then would sit low
	   and make the return to level look like a lift).  Vibration makes
	   |a| average slightly high: that only makes the rule less eager.
	   It is not tracked afterwards: a running filter would sag on a hump
	   in the same way. */
	if (st.settle < (uint8_t)IMU_LIFT_SETTLE) {
		st.g_sum += norm3(s->ax_mg, s->ay_mg, s->az_mg);
		st.uz_sum += s->az_mg;
		st.settle++;
		if (st.settle == (uint8_t)IMU_LIFT_SETTLE) {
			st.g_ref = st.g_sum / (int32_t)IMU_LIFT_SETTLE;
			st.up_sign = (st.uz_sum < 0) ? -1 : 1;
		}
		st.v_up_ums = 0;
		return;			/* learning g: not armed yet */
	}

	/* Upward speed: clamp (a knock lasts 1-2 samples), integrate, leak. */
	a_up = clamp32(a_par - st.g_ref, -IMU_LIFT_CLAMP_MG, IMU_LIFT_CLAMP_MG);
	st.v_up_ums += a_up * IMU_LIFT_UMS_PER_MG;
	st.v_up_ums -= st.v_up_ums / IMU_LIFT_LEAK_DIV;

	/* Noise floor.  Vibration makes v_up wander even on flat ground, and
	   a false LIFTED would end a demo run, so the threshold follows the
	   chassis: IMU_LIFT_SIGMAS x the rms of v_up, never below
	   IMU_LIFT_VEL_UMS.  The rms is learned only from quiet samples
	   (|v_up| under half the threshold), so the start of a real lift
	   cannot raise its own bar. */
	if ((iabs32(st.v_up_ums) * 2) < st.v_thr_ums) {
		int32_t	vm = clamp32(st.v_up_ums / 1000, -4000, 4000);

		st.v_var_q4 += ((vm * vm * 16) - st.v_var_q4) / IMU_LIFT_NOISE_DIV;
	}
	st.v_thr_ums = IMU_LIFT_SIGMAS * 1000 *
		       (int32_t)isqrt32((uint32_t)st.v_var_q4 / 16u);
	if (st.v_thr_ums < (int32_t)IMU_LIFT_VEL_UMS) {
		st.v_thr_ums = (int32_t)IMU_LIFT_VEL_UMS;
	}

	if ((st.flags & (uint8_t)IMU_FLAG_LIFTED) != 0u) {
		return;			/* latched until imu_clear() */
	}

	if (st.v_up_ums >= st.v_thr_ums) {
		if (st.lift_cnt < 255u) {
			st.lift_cnt++;
		}
	} else {
		st.lift_cnt = 0u;
	}

	if (st.up_pct < IMU_OVERTURN_PCT) {
		if (st.flip_cnt < 255u) {
			st.flip_cnt++;
		}
	} else {
		st.flip_cnt = 0u;	/* a wobble is not an overturn */
	}

	if (st.lift_cnt >= (uint8_t)IMU_LIFT_SAMPLES) {
		st.lift_cause = IMU_LIFT_LIFTED;
	} else if (st.flip_cnt >= (uint8_t)IMU_OVERTURN_SAMPLES) {
		st.lift_cause = IMU_LIFT_OVERTURNED;
	} else {
		return;
	}
	st.flags = (uint8_t)(st.flags | (uint8_t)IMU_FLAG_LIFTED);
}

/* ---------------------------------------------------------------------
 *  Motion classification (accelerometer + odometry; the turning state
 *  needs the encoder left/right speed difference, so it arrives with
 *  motion's interface rather than here).
 * --------------------------------------------------------------------- */
static void motion_step(const IMU_SAMPLE *s, uint8_t have_odom)
{
	if ((st.flags & (uint8_t)IMU_FLAG_LIFTED) != 0u) {
		st.motion = IMU_MOTION_LIFTED;
	} else if (st.hump_state != IMU_HUMP_IDLE) {
		st.motion = (st.pitch_cdeg >= 0) ? IMU_MOTION_CLIMBING
						 : IMU_MOTION_DESCENDING;
	} else if (have_odom == 0u) {
		st.motion = IMU_MOTION_UNKNOWN;
	} else if ((s->speed_mmps < 30u) && (iabs32(s->a_enc_mg) < 60)) {
		st.motion = IMU_MOTION_STATIONARY;
	} else if (s->a_enc_mg > 150) {
		st.motion = IMU_MOTION_ACCELERATING;
	} else if (s->a_enc_mg < -150) {
		st.motion = IMU_MOTION_DECELERATING;
	} else {
		st.motion = IMU_MOTION_CRUISING;
	}
}

/* ---------------------------------------------------------------------
 *  Public: one sample in, packed state out
 * --------------------------------------------------------------------- */
void imu_update(const IMU_SAMPLE *s)
{
	int32_t		delta;
	int32_t		force_mg;
	int32_t		sin_q15;
	uint8_t		have_odom;
	uint8_t		inhibit;
	uint8_t		may_start;
	int32_t		amax;

	/* 1. One-pole filters.  Division, not >>, because a right shift
	      of a negative value is implementation-defined. */
	delta = (int32_t)s->ax_mg - st.ax_fast;
	st.ax_fast += (delta * IMU_LP_FAST_Q10) / 1024;
	delta = (int32_t)s->ay_mg - st.ay_fast;
	st.ay_fast += (delta * IMU_LP_FAST_Q10) / 1024;
	delta = (int32_t)s->az_mg - st.az_fast;
	st.az_fast += (delta * IMU_LP_FAST_Q10) / 1024;
	delta = (int32_t)s->ay_mg - st.ay_slow;
	st.ay_slow += (delta * IMU_LP_SLOW_Q10) / 1024;

	/* 2. Lift / overturn first: once lifted, nothing below describes
	      the track any more. */
	lift_step(s);

	/* 3. Pitch: the accelerometer reads gravity's forward component
	      MINUS the car's own acceleration, so subtract the encoder
	      acceleration before converting:
	          ax = -g*sin(pitch) - a_car   ->   sin(pitch) = -(ax + a_car)/g
	      (week 8 A6: "correct pitch with the encoder-measured
	      acceleration" instead of only trusting steady speed). */
	force_mg = st.ax_fast + (int32_t)s->a_enc_mg;
	force_mg = clamp32(force_mg, -1000, 1000);
	sin_q15 = (-force_mg * 32768) / 1000;
	if (iabs32(sin_q15) < (int32_t)IMU_PITCH_DEADBAND_Q15) {
		sin_q15 = 0;			/* level robot: no drift */
	}
	st.sin_pitch_q15 = sin_q15;
	st.pitch_cdeg = asin_cdeg(sin_q15);

	/* Roll is informational; lateral acceleration corrupts it the same
	   way, so no decision in the car depends on roll. */
	st.roll_cdeg = asin_cdeg((-clamp32(st.ay_slow, -1000, 1000) * 32768) / 1000);
	roll_hump_step(st.roll_cdeg,
	                (-clamp32(st.ay_slow, -1000, 1000) * 32768) / 1000);

	/* 4. Saturation (IS25: the team driver sets +-4 g). */
	amax = iabs32(s->ax_mg);
	if (iabs32(s->ay_mg) > amax) {
		amax = iabs32(s->ay_mg);
	}
	if (iabs32(s->az_mg) > amax) {
		amax = iabs32(s->az_mg);
	}
	if (amax >= (int32_t)IMU_SATURATE_MG) {
		st.flags = (uint8_t)(st.flags | (uint8_t)IMU_FLAG_SATURATED);
	}

	/* 5. Hump: inhibited once lifted or overturned. */
	have_odom = (s->odom_valid != 0u) ? 1u : 0u;
	inhibit = ((st.flags & (uint8_t)IMU_FLAG_LIFTED) != 0u) ? 1u : 0u;
	may_start = ((have_odom != 0u) &&
		     (s->speed_mmps >= (uint16_t)IMU_HUMP_MIN_SPEED)) ? 1u : 0u;
	hump_step(sin_q15, s->dd_um, inhibit, may_start);

	motion_step(s, have_odom);

	if (st.hump_state != IMU_HUMP_IDLE) {
		st.flags = (uint8_t)(st.flags | (uint8_t)IMU_FLAG_HUMP_ACTIVE);
	} else {
		st.flags = (uint8_t)(st.flags & (uint8_t)~IMU_FLAG_HUMP_ACTIVE);
	}

	/* 6. Publish: two single-word stores, no lock (week 8 rule). */
	st.seq++;
	g_terrain_word = imu_pack_terrain(st.hump_peak_mm, st.hump_state,
					  st.motion, st.flags, st.seq);
	g_attitude_word = imu_pack_attitude(st.pitch_cdeg, st.roll_cdeg);
}

/* ---------------------------------------------------------------------
 *  Snapshot: one copy of each word, then decode the copy.
 * --------------------------------------------------------------------- */
void imu_snapshot(IMU_STATUS *out)
{
	uint32_t	word;
	uint16_t	att;
	uint8_t		seq;

	word = g_terrain_word;
	att = g_attitude_word;
	imu_unpack_terrain(word, out, &seq);
	imu_unpack_attitude(att, out);
}

/* ---------------------------------------------------------------------
 *  Packing.  Layout is fixed and mirrored in the dashboard decoder:
 *    terrain : [13:0] peak mm | [16:14] hump state | [19:17] motion
 *              [23:20] flags    | [31:24] sequence
 *    attitude: [7:0]  pitch (0.25 deg, +128 offset, so -32.00..+31.75)
 *              [15:8] roll  (same)
 * --------------------------------------------------------------------- */
uint32_t imu_pack_terrain(uint16_t peak_mm, uint8_t hump_state,
			  uint8_t motion, uint8_t flags, uint8_t seq)
{
	uint32_t	word;

	word = (uint32_t)peak_mm & 0x3FFFu;
	word |= ((uint32_t)hump_state & 0x07u) << 14;
	word |= ((uint32_t)motion & 0x07u) << 17;
	word |= ((uint32_t)flags & 0x0Fu) << 20;
	word |= ((uint32_t)seq) << 24;
	return word;
}

uint16_t imu_pack_attitude(int16_t pitch_cdeg, int16_t roll_cdeg)
{
	int32_t	pitch_code;
	int32_t	roll_code;

	/* 0.25 deg steps: centi-degrees / 25 (i.e. *4/100), +32 deg offset. */
	pitch_code = ((int32_t)pitch_cdeg * 4) / 100 + 128;
	roll_code = ((int32_t)roll_cdeg * 4) / 100 + 128;
	pitch_code = clamp32(pitch_code, 0, 255);
	roll_code = clamp32(roll_code, 0, 255);

	return (uint16_t)(((uint32_t)roll_code << 8) | (uint32_t)pitch_code);
}

void imu_unpack_terrain(uint32_t word, IMU_STATUS *out, uint8_t *seq)
{
	out->hump_peak_mm = (uint16_t)(word & 0x3FFFu);
	out->hump_state = (uint8_t)((word >> 14) & 0x07u);
	out->motion = (uint8_t)((word >> 17) & 0x07u);
	out->flags = (uint8_t)((word >> 20) & 0x0Fu);
	out->hump_count = 0;		/* not carried in the record */
	if (seq != NULL) {
		*seq = (uint8_t)((word >> 24) & 0xFFu);
	}
}

uint32_t imu_terrain_word(void)
{
	return g_terrain_word;		/* one aligned 32-bit load */
}

/* Diagnostics (single writer's state; 16-bit aligned reads are atomic). */
uint16_t imu_hump_count(void)
{
	return st.hump_count;
}

uint8_t imu_hump_state(void)
{
	return st.hump_state;
}

int16_t imu_pitch_cdeg(void)
{
	return st.pitch_cdeg;
}

uint8_t imu_lift_cause(void)
{
	return st.lift_cause;
}

int16_t imu_lift_speed_mm_s(void)
{
	return (int16_t)clamp32(st.v_up_ums / 1000, -32767, 32767);
}

int16_t imu_lift_threshold_mm_s(void)
{
	return (int16_t)clamp32(st.v_thr_ums / 1000, 0, 32767);
}

int16_t imu_gravity_mg(void)
{
	return (int16_t)clamp32(st.g_ref, 0, 32767);
}

int16_t imu_up_pct(void)
{
	return st.up_pct;
}

void imu_unpack_attitude(uint16_t word, IMU_STATUS *out)
{
	int32_t	pitch_code;
	int32_t	roll_code;

	pitch_code = (int32_t)(word & 0xFFu) - 128;
	roll_code = (int32_t)((word >> 8) & 0xFFu) - 128;
	out->pitch_cdeg = (int16_t)(pitch_code * 25);	/* back to centi-deg */
	out->roll_cdeg = (int16_t)(roll_code * 25);
}

/* =====================================================================
 *  Test hooks: a deterministic synthetic sensor.
 *
 *  The Pico W arrives before the GY-511 does, so the whole event path -
 *  task period, hump state machine, lift rule, packing - is proven
 *  with generated samples first and re-run unchanged on real hardware.
 *
 *  The generator is deliberately PHYSICAL: it starts from a slope and a
 *  car acceleration and then computes what the accelerometer would read,
 *      ax = -g*sin(theta) - a_car
 *  so the encoder-acceleration correction is exercised, not bypassed,
 *  and az = g*cos(theta), so |a| stays 1 g on a slope exactly as on the
 *  real sensor (the lift rule depends on that).
 *
 *  Hump geometry: a hump of height h and length L has a constant climb
 *  slope sin(theta) = 2h/L for the first half and the same slope down for
 *  the second half.  The running integral therefore rises to exactly
 *  (2h/L)*(L/2) = h at the crest and returns to zero - which is what the
 *  hump step machine must report.
 * ===================================================================== */
static void
roll_hump_step (int16_t roll_cdeg, int32_t sin_q15)
{
	int32_t now_q8;
	int32_t dev_cdeg;
	int32_t mag;
	int32_t mm;

	if (0u == g_b_hump_diag) {
		return;		/* spec behaviour */
	}

	now_q8      = (int32_t)roll_cdeg * 256;
	rl_avg_q8  += (now_q8 - rl_avg_q8) >> IMU_ROLL_AVG_SHIFT;
	rl_base_q8 += (rl_avg_q8 - rl_base_q8) >> IMU_ROLL_BASE_SHIFT;
	dev_cdeg    = (rl_avg_q8 - rl_base_q8) >> 8;
	mag         = iabs32(dev_cdeg);

	if (0u == rl_active) {
		if (mag >= IMU_ROLL_ENTER_CDEG) {
			if (rl_enter < 255u) {
				rl_enter++;
			}
			if (rl_enter >= (uint8_t)IMU_ROLL_ENTER_SAMPLES) {
				rl_active    = 1u;
				rl_peak_cdeg = (int16_t)dev_cdeg;
				rl_peak_sin  = iabs32(sin_q15);
				rl_exit      = 0u;
			}
		} else {
			rl_enter = 0u;
		}
		return;
	}

	if (mag > iabs32((int32_t)rl_peak_cdeg)) {
		rl_peak_cdeg = (int16_t)dev_cdeg;
		rl_peak_sin  = iabs32(sin_q15);
	}

	if (mag < IMU_ROLL_EXIT_CDEG) {
		if (rl_exit < 65535u) {
			rl_exit++;
		}
	} else {
		rl_exit = 0u;
	}

	if (rl_exit >= (uint16_t)IMU_ROLL_EXIT_SAMPLES) {
		mm = (int32_t)(((int32_t)IMU_ROLL_TRACK_MM * rl_peak_sin) / 32768);

		if ((mm >= IMU_ROLL_MIN_MM) && (mm <= IMU_ROLL_MAX_MM)) {
			st.hump_peak_mm = (uint16_t)mm;
			st.hump_count++;    /* the glue turns this into a report */
		}

		rl_active    = 0u;
		rl_enter     = 0u;
		rl_exit      = 0u;
		rl_peak_cdeg = 0;
		rl_peak_sin  = 0;
	}
}

#ifdef IMU_TEST_HOOKS

typedef struct {
	uint8_t		shape;
	uint16_t	height_mm;
	uint16_t	length_mm;
	uint16_t	speed_mmps;
	int16_t		tilt_cdeg;
	uint32_t	step;
} IMU_SIM;

static IMU_SIM sim;

/* centi-degrees -> sin, by walking the asin table backwards. */
static int32_t sin_from_cdeg(int16_t cdeg)
{
	int32_t	mag;
	int32_t	idx;
	int32_t	val;

	mag = iabs32((int32_t)cdeg);
	if (mag > 9000) {
		mag = 9000;
	}
	for (idx = 63; idx >= 0; idx--) {
		if ((int32_t)asin_cdeg_tbl[idx] <= mag) {
			val = (idx * 512) + 256;
			return (cdeg < 0) ? -val : val;		/* Q15 */
		}
	}
	return 0;
}

/* cos from sin, both Q15 (cos >= 0: |angle| <= 90 deg). */
static int32_t cos_from_sin(int32_t sin_q15)
{
	int32_t	m = clamp32(iabs32(sin_q15), 0, 32768);

	return (int32_t)isqrt32((uint32_t)(32768 * 32768) -
				(uint32_t)(m * m));
}

void imu_gen_sample(uint8_t shape, uint16_t height_mm, uint16_t length_mm,
		    uint16_t speed_mmps, int16_t tilt_cdeg,
		    uint32_t step, IMU_SAMPLE *out)
{
	uint32_t	x_um;
	int32_t		sin_q15;
	int32_t		cos_q15;
	uint32_t	dd_um;
	uint16_t	speed_now;
	int16_t		a_car;
	int32_t		ay;
	int32_t		az_extra;
	int32_t		ax;
	int32_t		az;

	sin_q15 = 0;
	a_car = 0;
	ay = 0;
	az_extra = 0;
	speed_now = speed_mmps;
	dd_um = (uint32_t)speed_mmps * IMU_SAMPLE_PERIOD_MS;
	x_um = step * dd_um;

	switch (shape) {
	case IMU_SIM_HUMP:
		if (length_mm != 0u) {
			int32_t smax = ((int32_t)height_mm * 2 * 32768) /
				       (int32_t)length_mm;
			if (smax > 32767) {
				smax = 32767;
			}
			if (x_um < ((uint32_t)length_mm * 1000u) / 2u) {
				sin_q15 = smax;			/* climbing */
			} else if (x_um < ((uint32_t)length_mm * 1000u)) {
				sin_q15 = -smax;		/* descending */
			}
		}
		break;

	case IMU_SIM_LIFT:
		/* 1.5 s of level cruise (the detector arms after 1 s), then a
		   hand lifts the car: an upward push, the same deceleration to
		   hold it, then held still in the air with the wheels still
		   turning (speed unchanged).  tilt_cdeg != 0: the hand also
		   tips the nose up to that angle during the push. */
		{
			int32_t	acc = (height_mm != 0u) ? (int32_t)height_mm : 200;
			uint32_t push = ((length_mm != 0u) ? (uint32_t)length_mm
							   : 300u) /
					IMU_SAMPLE_PERIOD_MS;
			int32_t	vert = 1000;
			int32_t	tilt = 0;
			int32_t	sl;

			if ((step >= 150u) && (step < 150u + push)) {
				vert += acc;
			} else if ((step >= 150u + push) &&
				   (step < 150u + (2u * push))) {
				vert -= acc;
			}
			if (step >= 150u + push) {
				tilt = tilt_cdeg;
			} else if (step >= 150u) {
				tilt = ((int32_t)tilt_cdeg * (int32_t)(step - 150u)) /
				       (int32_t)push;
			}
			/* World-vertical specific force seen by a tilted sensor. */
			sl = sin_from_cdeg((int16_t)tilt);
			out->ax_mg = (int16_t)(-((vert * sl) / 32768));
			out->ay_mg = 0;
			out->az_mg = (int16_t)((vert * cos_from_sin(sl)) / 32768);
			out->a_enc_mg = 0;
			out->speed_mmps = speed_now;
			out->odom_valid = 1u;
			out->dd_um = dd_um;
			return;
		}

	case IMU_SIM_BRAKE:
		/* Smooth 60 % slowdown, no spike: must raise nothing. */
		if ((step >= 20u) && (step < 60u)) {
			uint32_t k = (step - 20u) / 20u;
			speed_now = (uint16_t)(speed_mmps -
					       ((uint32_t)speed_mmps * 60u * k) / 200u);
			dd_um = (uint32_t)speed_now * IMU_SAMPLE_PERIOD_MS;
			if (k < 2u) {
				a_car = -600;
			}
		}
		break;

	case IMU_SIM_TILT:
		sin_q15 = sin_from_cdeg(tilt_cdeg);
		break;

	case IMU_SIM_FLIP:
		/* Roll over sideways from step 150 to tilt_cdeg (0 = 180 deg,
		   upside down) in 100 samples, then stay there. */
		{
			int32_t	target = (tilt_cdeg != 0) ? (int32_t)tilt_cdeg
							  : 18000;
			int32_t	roll;
			int32_t	r;
			int32_t	sr;

			if (step < 150u) {
				roll = 0;
			} else if (step < 250u) {
				roll = (target * (int32_t)(step - 150u)) / 100;
			} else {
				roll = target;
			}
			r = (roll > 9000) ? (18000 - roll) : roll;
			sr = sin_from_cdeg((int16_t)r);
			ay = (sr * 1000) / 32768;
			az_extra = 0;
			/* Up component: cos(roll), negative past 90 deg.  The
			   pitch path stays level (sin_q15 = 0): only roll moves. */
			az = (cos_from_sin(sr) * 1000) / 32768;
			if (roll > 9000) {
				az = -az;
			}
			out->ax_mg = 0;
			out->ay_mg = (int16_t)ay;
			out->az_mg = (int16_t)az;
			out->a_enc_mg = 0;
			out->speed_mmps = speed_now;
			out->odom_valid = 1u;
			out->dd_um = dd_um;
			return;
		}

	case IMU_SIM_BUMP:
		/* Flat cruise with motor vibration (+-150 mg, 50 Hz, all axes)
		   and a hard vertical knock every 0.3 s (+1.5 g then -1.5 g, two
		   samples each): a hump edge or a seam.  Must raise nothing. */
		ay = ((step & 1u) != 0u) ? 150 : -150;
		az_extra = ((step & 1u) != 0u) ? -150 : 150;
		a_car = (int16_t)(((step & 1u) != 0u) ? 150 : -150);
		if ((step >= 50u) && (((step - 50u) % 30u) < 4u)) {
			az_extra = (((step - 50u) % 30u) < 2u) ? 1500 : -1500;
		}
		break;

	case IMU_SIM_FLAT:
	default:
		break;
	}

	/* What the accelerometer reads: ax = -g*sin(theta) - a_car, and the
	   up axis g*cos(theta) plus any vertical acceleration. */
	cos_q15 = cos_from_sin(sin_q15);
	ax = -((sin_q15 * 1000) / 32768) - (int32_t)a_car;
	az = ((cos_q15 * 1000) / 32768) + az_extra;

	out->ax_mg = (int16_t)clamp32(ax, -32767, 32767);
	out->ay_mg = (int16_t)clamp32(ay, -32767, 32767);
	out->az_mg = (int16_t)clamp32(az, -32767, 32767);
	out->a_enc_mg = (shape == IMU_SIM_BUMP) ? 0 : a_car;
	out->speed_mmps = speed_now;
	out->odom_valid = 1u;
	out->dd_um = dd_um;
}

void imu_sim_start(uint8_t shape, uint16_t height_mm, uint16_t length_mm,
		   uint16_t speed_mmps, int16_t tilt_cdeg)
{
	sim.shape = shape;
	sim.height_mm = height_mm;
	sim.length_mm = length_mm;
	sim.speed_mmps = speed_mmps;
	sim.tilt_cdeg = tilt_cdeg;
	sim.step = 0;
}

void imu_sim_stop(void)
{
	sim.shape = IMU_SIM_FLAT;
	sim.step = 0;
}

uint8_t imu_sim_running(void)
{
	return 1u;
}

void imu_sim_next(IMU_SAMPLE *out)
{
	imu_gen_sample(sim.shape, sim.height_mm, sim.length_mm,
		       sim.speed_mmps, sim.tilt_cdeg, sim.step, out);
	sim.step++;
}

#endif /* IMU_TEST_HOOKS */
