/*
 *	imu.h
 *	Subsystem 4 -- Physics Watchdog: terrain / hump / lift maths.
 *
 *	Student ID: 2500989 (Zi Siang)
 *
 *	This module is PURE C: no kernel calls, no I/O, no floating point.
 *	It can therefore be compiled and unit-tested on a PC (see host_test/)
 *	as well as on the Pico.  All internal maths is integer (milli-g,
 *	millimetres, centi-degrees) because the Cortex-M0+ has no FPU and a
 *	100 Hz task should not pay for soft-float trigonometry.
 *
 *	Sensor frame (LSM303D module mounted flat on the chassis):
 *	    X = forward, Y = left, Z = up
 *	Accelerometer values are milli-g (1000 = 1 g).
 *
 *	Sign conventions (agreed with the motion subsystem, TEAM.md 5):
 *	    pitch_cdeg > 0  nose up      (climbing a hump)
 *	    roll_cdeg  > 0  left side up
 *	    a_enc_mg   > 0  accelerating forward
 *	    dd_mm           distance travelled since the previous sample
 *
 *	IS25 (integration, 26 Oct): this file now lives in subsystem3/ (the
 *	team's IMU subsystem) and is driven by imu_task.c through imu_glue.c.
 *	The sign model below (ax = -g*sin(pitch) - a_car) is the NEGATIVE of
 *	what a real LSM303 reports with X forward (it reads +g*sin(pitch) +
 *	a_car, the specific force).  imu_glue.c therefore passes -forward and
 *	-left readings, so this module is used unchanged.  See
 *	BUDDY4_INTEGRATION.md.
 *
 *	Data flow:  task_imu.c reads the sensor, fills IMU_SAMPLE and calls
 *	imu_update() once every 10 ms.  Everything else in the car reads the
 *	packed results (imu_snapshot) and the event flag bits.
 */

#ifndef IMU_H
#define IMU_H

#include <stdint.h>

/* The synthetic sensor is needed in two builds: the PC host test (which
   passes -DIMU_TEST_HOOKS) and the on-target test image (make TEST=1, which
   passes -DAPP_TEST=1). */
#if !defined(IMU_TEST_HOOKS) && defined(APP_TEST) && (APP_TEST == 1)
#define IMU_TEST_HOOKS	1
#endif

/* ---------------------------------------------------------------------
 *  Tuning constants.  Every one of these is a number the report and the
 *  test log quote, so change them here only, never inline.
 * --------------------------------------------------------------------- */

#define IMU_SAMPLE_HZ		100u	/* 10 ms task period */
#define IMU_SAMPLE_PERIOD_MS	10u

/* Two one-pole filters: the tilt / overturn path wants a FAST filter so
   the hump onset is not missed; roll (report only) uses the SLOW one.
   alpha = 1 - exp(-2*pi*fc/100), stored as Q10.
     fast: fc = ~15 Hz -> 0.61  -> 625
     slow: fc = ~4 Hz  -> 0.22  -> 226 */
#define IMU_LP_FAST_Q10		625
#define IMU_LP_SLOW_Q10		226

/* Pitch dead-band: below 0.40 deg the correction term is zero, so a
   stationary robot cannot integrate sensor noise into a fake hump. */
#define IMU_PITCH_DEADBAND_Q15	230

/* Hump state machine (centi-degrees, 300 = 3.00 deg) */
#define IMU_HUMP_ENTER_CDEG	300
#define IMU_HUMP_ENTER_SAMPLES	2u	/* 20 ms above the threshold */
#define IMU_HUMP_EXIT_CDEG	200
#define IMU_HUMP_EXIT_SAMPLES	5u	/* 50 ms back near level */
/* Ramp guard (Nov 2026: by DISTANCE, was 4 s).  Still climbing after
   600 mm of travel = a ramp, not a hump: dropped.  At the 3 cm/s demo
   speed a 50 mm hump at 12 deg takes 8 s to climb, so a 4 s guard threw
   real humps away; at 25 cm/s 4 s was 1000 mm anyway.  The time guard
   stays only for a car that stops on a slope. */
#define IMU_HUMP_MAX_UM		600000u	/* 600 mm in the hump state   */
#define IMU_HUMP_MAX_SAMPLES	3000u	/* 30 s safety release        */

/* Lift / overturn (Nov 2026 change from the lecturer: IMU collision
   detection is REMOVED; the IMU must detect the robot being LIFTED or
   OVERTURNED and stop the motors at once.  A lift ends the demo run).

   LIFT.  A robot held still in the air reads exactly 1 g, the same as on
   the floor, so the lift is detected while it HAPPENS: the hand gives the
   chassis an upward speed.  a_up = (a . g^) - |g|, the acceleration along
   the gravity direction (0.32 s filter) minus the size of g (learned once,
   while arming): a horizontal push (braking, turning, touching an
   obstacle) is at right angles to g and adds nothing, and a rotation
   (hump slope, a hand tilting the car) only makes it smaller.  It is clamped per sample
   so a short knock cannot count, and integrated into a leaky vertical
   speed.  On this course the car cannot rise faster than its own forward
   speed (30-120 mm/s up a hump slope gives under 40 mm/s upward); a hand
   lift gives 200-600 mm/s.  The threshold is 150 mm/s held for 50 ms, or
   5 x the rms of v_up while driving if the chassis vibrates more (the
   simulator showed 70 mg rms of vibration could reach 150 mm/s by
   chance), so a false lift cannot end a run.
     v_up += a_up x 98 um/s per mg per sample;  v_up -= v_up / 50
   (1 mg for 10 ms = 9.807e-3 m/s^2 x 0.01 s = 98 um/s; tau = 0.5 s.)

   OVERTURN.  The up (z) axis carries less than half of gravity (tilt over
   60 degrees, any direction, or upside down) for 200 ms.  The sign of
   "up" is learned while arming, so a module mounted upside down still
   works.  The track humps are under 30 degrees: never fires on the course. */
#define IMU_LIFT_CLAMP_MG	250	/* per-sample |a| - g limit           */
#define IMU_LIFT_UMS_PER_MG	98	/* um/s added per mg per 10 ms sample */
#define IMU_LIFT_LEAK_DIV	50	/* tau = 50 samples = 0.5 s           */
#define IMU_LIFT_VEL_UMS	150000	/* 150 mm/s upward, at least          */
#define IMU_LIFT_SIGMAS		5	/* ... or 5 x the rms of v_up         */
#define IMU_LIFT_NOISE_DIV	512	/* rms learned over ~5 s              */
#define IMU_LIFT_VAR0_Q4	(900 * 16)	/* start: 30 mm/s rms         */
#define IMU_LIFT_SAMPLES	5u	/* held for 50 ms                     */
#define IMU_GDIR_DIV		32	/* gravity direction, tau = 0.32 s    */
#define IMU_LIFT_SETTLE		100u	/* 1 s to learn |g| after (re)arming  */
#define IMU_OVERTURN_PCT	50	/* up < 50 % of |g|: tilt > 60 deg    */
#define IMU_OVERTURN_SAMPLES	20u	/* 200 ms                             */
#define IMU_AXIS_LIMIT_MG	8000	/* inputs clamped before squaring      */

/* Why the lift ended the run (imu_lift_cause()). */
#define IMU_LIFT_NONE		0u
#define IMU_LIFT_LIFTED		1u
#define IMU_LIFT_OVERTURNED	2u

/* Full-scale check: "near saturation" gets reported in the flags.
   IS25: the team driver sets CTRL_REG4_A to +-4 g (Week 6), so the limit
   is 3950 mg (was 1950 for his +-2 g setting). */
#define IMU_SATURATE_MG		3950

/* Motion classes: 3 bits in the telemetry record (week 8 decision). */
#define IMU_MOTION_UNKNOWN	0u	/* no odometry yet */
#define IMU_MOTION_STATIONARY	1u
#define IMU_MOTION_ACCELERATING	2u
#define IMU_MOTION_DECELERATING	3u
#define IMU_MOTION_CRUISING	4u
#define IMU_MOTION_CLIMBING	5u
#define IMU_MOTION_DESCENDING	6u
#define IMU_MOTION_LIFTED	7u	/* lifted or overturned (was IMPACT) */

/* Hump sanity: no hump on this course is taller than 150 mm.  A larger
   peak means the chassis is on a wedge, a ramp or a stack of books - the
   angle is real but it is not a hump, so it is not counted or reported. */
#define IMU_HUMP_MAX_MM		150
/* IS25: a peak below 5 mm is noise or a hard launch (the encoder
   acceleration lags the accelerometer by ~25 ms), not a hump.  The track
   humps are 20-40 mm. */
#define IMU_HUMP_MIN_MM		5
/* A hump can only be entered while the robot is actually moving. */
#define IMU_HUMP_MIN_SPEED	10u	/* mm/s (Nov 2026: was 30, the demo
					   follows at 30 and recovers at 20) */

/* Hump states: 3 bits in the telemetry record. */
#define IMU_HUMP_IDLE		0u
#define IMU_HUMP_RISING		1u
#define IMU_HUMP_FALLING	2u

/* Status flags: 4 bits in the telemetry record. */
#define IMU_FLAG_HUMP_ACTIVE	0x01u
#define IMU_FLAG_LIFTED		0x02u	/* lifted or overturned (latched) */
#define IMU_FLAG_FAULT		0x04u
#define IMU_FLAG_SATURATED	0x08u

/* One sensor sample plus the odometry the module needs from motion. */
typedef struct {
	int16_t		ax_mg;		/* forward, milli-g */
	int16_t		ay_mg;		/* left, milli-g */
	int16_t		az_mg;		/* up, milli-g */
	int16_t		a_enc_mg;	/* d(speed)/dt from encoders, + = faster */
	uint16_t	speed_mmps;
	uint8_t		odom_valid;	/* 1 = motion is publishing odometry.
					   Without this flag a stopped robot and a
					   missing encoder interface look identical
					   (both: speed 0, dd 0). */
	uint32_t	dd_um;		/* distance since previous sample, micrometres
					   (1 mm steps of a 15-bit odometry word would
					   quantise a 2.5 mm/sample motion too coarsely) */
} IMU_SAMPLE;

/* Everything the rest of the car may read. */
typedef struct {
	int16_t		pitch_cdeg;
	int16_t		roll_cdeg;
	uint16_t	hump_peak_mm;	/* last completed hump */
	uint16_t	hump_count;	/* how many humps seen this run */
	uint8_t		hump_state;	/* IMU_HUMP_* */
	uint8_t		motion;		/* IMU_MOTION_* */
	uint8_t		flags;		/* IMU_FLAG_* */
} IMU_STATUS;

/* ---- API used by task_imu.c and by subsystem4.c ------------------- */

void	imu_init(void);
void	imu_update(const IMU_SAMPLE *sample);	/* call every 10 ms */
void	imu_snapshot(IMU_STATUS *out);		/* copy the shared words */
void	imu_set_fault(uint8_t on);
void	imu_clear(uint8_t flag_mask);		/* owner-side clear rule;
						   clearing LIFTED re-arms */

/* Diagnostics for the task's own log line and for the tests.  Not part of
   the shared record: one 16-bit read from the single writer's state. */
uint32_t imu_terrain_word(void);	/* one copy of the packed terrain word */
uint16_t imu_hump_count(void);
void     imu_hump_diag(uint8_t b_on);      /* test build: report dropped humps */
uint8_t	 imu_hump_state(void);
int16_t	 imu_pitch_cdeg(void);
uint8_t	 imu_lift_cause(void);		/* IMU_LIFT_* */
int16_t	 imu_lift_speed_mm_s(void);	/* leaky upward speed, mm/s */
int16_t	 imu_lift_threshold_mm_s(void);	/* current threshold, mm/s */
int16_t	 imu_gravity_mg(void);		/* learned |g|, mg */
int16_t	 imu_up_pct(void);		/* % of |a| on the up axis (100 = level) */

/* Packed telemetry words (little-endian bit layout, week 8 decision):
 *   terrain  (32 bit): peak mm 14 | hump state 3 | motion 3 | flags 4 | seq 8
 *   attitude (16 bit): pitch 8 (0.25 deg, +128 offset) | roll 8 (same)
 * Both are pure functions of the values, so the test and any future
 * sender can build the same word the task builds.
 */
uint32_t imu_pack_terrain(uint16_t peak_mm, uint8_t hump_state,
			  uint8_t motion, uint8_t flags, uint8_t seq);
uint16_t imu_pack_attitude(int16_t pitch_cdeg, int16_t roll_cdeg);

/* Decode helpers -- used by the tests and by the dashboard table. */
void	imu_unpack_terrain(uint32_t word, IMU_STATUS *out, uint8_t *seq);
void	imu_unpack_attitude(uint16_t word, IMU_STATUS *out);

/* ---------------------------------------------------------------------
 *  Test-only hooks (APP_TEST / IMU_HOST_TEST builds only)
 * --------------------------------------------------------------------- */
#ifdef IMU_TEST_HOOKS
#define IMU_SIM_FLAT		0u
#define IMU_SIM_HUMP		1u
#define IMU_SIM_LIFT		2u	/* was COLLISION */
#define IMU_SIM_BRAKE		3u
#define IMU_SIM_TILT		4u
#define IMU_SIM_FLIP		5u
#define IMU_SIM_BUMP		6u

/* Configure the synthetic source. height/length in mm, speed in mm/s,
 * tilt in centi-degrees.  IMU_SIM_LIFT reuses height_mm as the hand's
 * upward acceleration in mg (0 = 200), length_mm as the duration of
 * the upward push in ms (0 = 300) and tilt_cdeg as a nose-up tip during
 * the push (0 = none).  IMU_SIM_FLIP rolls the car over to
 * tilt_cdeg (0 = 18000, upside down) in 1 s.  IMU_SIM_BUMP drives flat
 * with motor vibration and a hard vertical knock every 0.3 s. */
void	imu_sim_start(uint8_t shape, uint16_t height_mm,
		      uint16_t length_mm, uint16_t speed_mmps,
		      int16_t tilt_cdeg);
void	imu_sim_stop(void);
/* Produce the next synthetic sample for the task to consume. */
void	imu_sim_next(IMU_SAMPLE *out);
uint8_t	imu_sim_running(void);

/* Pure generator, also usable from the host test without any state. */
void	imu_gen_sample(uint8_t shape, uint16_t height_mm, uint16_t length_mm,
		       uint16_t speed_mmps, int16_t tilt_cdeg,
		       uint32_t step, IMU_SAMPLE *out);
#endif /* IMU_TEST_HOOKS */

#endif /* IMU_H */
