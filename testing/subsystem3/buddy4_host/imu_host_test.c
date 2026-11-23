/*
 *	imu_host_test.c
 *	Off-target unit tests for imu.c -- run on the PC, not on the Pico.
 *
 *	Student ID: 2500989 (Zi Siang)
 *
 *	Why this exists: the IMU and the Robo Pico had not arrived yet, so
 *	the hump / pitch maths is proven here first against synthetic traces
 *	with analytically known answers, then re-run unchanged on the real
 *	sensor (subsystem3_test.c).
 *
 *	Nov 2026 (lecturer's change): collision detection is removed; T6-T8
 *	and T10 now test the lift / overturn rule that replaced it, and T14-
 *	T16 are new (lift, overturn, latch and re-arm).
 *
 *	Build and run:   ./build_and_run.sh          (gcc, no dependencies)
 *	                 cc -std=c99 -Wall -Wextra -Werror -DIMU_TEST_HOOKS \
 *	                    -I.. imu.c imu_host_test.c -o imu_host_test
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "imu.h"

static int g_pass;
static int g_fail;

static void check(int ok, const char *what, long actual, long expect)
{
	if (ok) {
		g_pass++;
		printf("  PASS  %-52s (%ld)\n", what, actual);
	} else {
		g_fail++;
		printf("  FAIL  %-52s got %ld, want %ld\n", what, actual, expect);
	}
}

static void check_range(long v, long lo, long hi, const char *what)
{
	int ok = (v >= lo) && (v <= hi);
	if (ok) {
		g_pass++;
		printf("  PASS  %-52s (%ld in %ld..%ld)\n", what, v, lo, hi);
	} else {
		g_fail++;
		printf("  FAIL  %-52s got %ld, want %ld..%ld\n", what, v, lo, hi);
	}
}

/* Feed n synthetic samples of one shape into the module. */
static void run_shape(uint8_t shape, uint16_t h_mm, uint16_t l_mm,
		      uint16_t speed, int16_t tilt_cdeg, uint32_t n)
{
	IMU_SAMPLE s;
	uint32_t i;

	for (i = 0; i < n; i++) {
		imu_gen_sample(shape, h_mm, l_mm, speed, tilt_cdeg, i, &s);
		imu_update(&s);
	}
}

/* A tiny deterministic LCG, so the noise test is reproducible. */
static int16_t noise_mg(uint32_t *seed)
{
	*seed = (*seed * 1103515245u) + 12345u;
	return (int16_t)(((int32_t)((*seed >> 16) & 0x3FFu)) - 512) / 34;
}

int main(void)
{
	IMU_STATUS	snap;
	IMU_SAMPLE	s;
	uint32_t	seed = 7u;
	int		i;
	uint8_t		seq = 0u;
	uint32_t	word;
	uint16_t	att;
	int		lift_at = -1;
	int		hump_count;

	printf("imu.c host tests (100 Hz, integer maths)\n");

	/* ---- T1: 10 s level cruise, must stay silent ------------------- */
	printf("\n[T1] 10 s flat cruise at 250 mm/s\n");
	imu_init();
	run_shape(IMU_SIM_FLAT, 0, 0, 250, 0, 1000);
	imu_snapshot(&snap);
	check(imu_hump_count() == 0, "no hump on flat ground", imu_hump_count(), 0);
	check((snap.flags & IMU_FLAG_LIFTED) == 0, "no lift on flat ground",
	      (snap.flags & IMU_FLAG_LIFTED), 0);
	check_range(snap.pitch_cdeg, -50, 50, "pitch stays level (<0.5 deg)");

	/* ---- T2-T4: humps of known height ----------------------------- */
	printf("\n[T2] 30 mm hump, 400 mm long, 250 mm/s\n");
	imu_init();
	run_shape(IMU_SIM_HUMP, 30, 400, 250, 0, 400);
	imu_snapshot(&snap);
	check(imu_hump_count() == 1, "exactly one hump detected", imu_hump_count(), 1);
	check_range((long)snap.hump_peak_mm, 25, 35, "peak height 30 mm +/-15% (NFR9)");
	check((snap.flags & IMU_FLAG_HUMP_ACTIVE) == 0, "humP flag cleared after the hump",
	      (snap.flags & IMU_FLAG_HUMP_ACTIVE), 0);

	printf("\n[T3] 20 mm hump, 200 mm long, 150 mm/s\n");
	imu_init();
	run_shape(IMU_SIM_HUMP, 20, 200, 150, 0, 400);
	imu_snapshot(&snap);
	check(imu_hump_count() == 1, "exactly one hump detected", imu_hump_count(), 1);
	check_range((long)snap.hump_peak_mm, 17, 23, "peak height 20 mm +/-15%");

	printf("\n[T4] 40 mm hump, 600 mm long, 250 mm/s\n");
	imu_init();
	run_shape(IMU_SIM_HUMP, 40, 600, 250, 0, 600);
	imu_snapshot(&snap);
	check(imu_hump_count() == 1, "exactly one hump detected", imu_hump_count(), 1);
	check_range((long)snap.hump_peak_mm, 34, 46, "peak height 40 mm +/-15%");

	printf("\n[T5] two humps in one run\n");
	imu_init();
	run_shape(IMU_SIM_FLAT, 0, 0, 250, 0, 100);
	run_shape(IMU_SIM_HUMP, 25, 300, 250, 0, 200);
	run_shape(IMU_SIM_FLAT, 0, 0, 250, 0, 100);
	run_shape(IMU_SIM_HUMP, 25, 300, 250, 0, 200);
	run_shape(IMU_SIM_FLAT, 0, 0, 250, 0, 100);
	imu_snapshot(&snap);
	check(imu_hump_count() == 2, "both humps counted separately", imu_hump_count(), 2);
	check_range((long)snap.hump_peak_mm, 21, 29, "second hump peak still 25 mm");

	/* ---- T6: braking must NOT look like a lift --------------------- */
	printf("\n[T6] 60 %% braking ramp (must not stop the robot)\n");
	imu_init();
	run_shape(IMU_SIM_BRAKE, 0, 0, 250, 0, 200);
	imu_snapshot(&snap);
	check((snap.flags & IMU_FLAG_LIFTED) == 0,
	      "braking alone raises no LIFTED", (snap.flags & IMU_FLAG_LIFTED), 0);

	/* ---- T7: a lift by hand must be caught -------------------------- */
	printf("\n[T7] lifted by hand at 1.5 s (0.2 g up for 0.3 s)\n");
	imu_init();
	for (i = 0; i < 400; i++) {
		imu_gen_sample(IMU_SIM_LIFT, 0, 0, 250, 0, (uint32_t)i, &s);
		imu_update(&s);
		imu_snapshot(&snap);
		if (((snap.flags & IMU_FLAG_LIFTED) != 0) && (lift_at < 0)) {
			lift_at = i;
		}
	}
	check(lift_at > 0, "LIFTED raised", lift_at, 160);
	check_range(lift_at, 150, 180, "raised within 300 ms of the push");
	imu_snapshot(&snap);
	check(snap.motion == IMU_MOTION_LIFTED, "motion class = LIFTED",
	      snap.motion, IMU_MOTION_LIFTED);
	check(imu_lift_cause() == IMU_LIFT_LIFTED, "cause = LIFTED",
	      imu_lift_cause(), IMU_LIFT_LIFTED);
	check(imu_hump_count() == 0, "a lift does not produce a fake hump",
	      imu_hump_count(), 0);

	/* ---- T8: a vertical kick over a hump is not a lift ------------- */
	printf("\n[T8] 1.2 g vertical kick (hump / pothole edge)\n");
	imu_init();
	for (i = 0; i < 300; i++) {
		memset(&s, 0, sizeof(s));
		s.speed_mmps = 250u;
		s.odom_valid = 1u;
		s.dd_um = 2500u;
		s.az_mg = 1000;
		if ((i >= 150) && (i < 152)) {
			s.az_mg = 2200;		/* pure Z spike */
		}
		imu_update(&s);
	}
	imu_snapshot(&snap);
	check((snap.flags & IMU_FLAG_LIFTED) == 0,
	      "vertical kick raises no LIFTED", (snap.flags & IMU_FLAG_LIFTED), 0);

	/* ---- T9: steady tilt reads as tilt, not as a hump -------------- */
	printf("\n[T9] held on a 10 deg wedge, stationary\n");
	imu_init();
	run_shape(IMU_SIM_TILT, 0, 0, 250, 1000, 500);
	imu_snapshot(&snap);
	check_range(snap.pitch_cdeg, 850, 1150, "pitch reads 1000 cdeg +/-1.5 deg");
	check(imu_hump_count() == 0, "a constant slope is not a hump", imu_hump_count(), 0);

	/* ---- T10: 60 s of sensor noise, no false events ---------------- */
	printf("\n[T10] 60 s of +/-15 mg noise on a flat, moving robot\n");
	imu_init();
	for (i = 0; i < 6000; i++) {
		s.ax_mg = noise_mg(&seed);
		s.ay_mg = noise_mg(&seed);
		s.az_mg = (int16_t)(1000 + noise_mg(&seed));
		s.a_enc_mg = 0;
		s.speed_mmps = 250u;
		s.odom_valid = 1u;
		s.dd_um = 2500u;
		imu_update(&s);
	}
	imu_snapshot(&snap);
	hump_count = (int)imu_hump_count();
	check(hump_count == 0, "no false hump in 60 s of noise", hump_count, 0);
	check((snap.flags & IMU_FLAG_LIFTED) == 0,
	      "no false lift in 60 s of noise",
	      (snap.flags & IMU_FLAG_LIFTED), 0);
	check_range(snap.pitch_cdeg, -200, 200, "noise never tilts the pitch estimate (<2 deg)");

	/* ---- T11: stationary ------------------------------------------- */
	printf("\n[T11] robot at rest\n");
	imu_init();
	memset(&s, 0, sizeof(s));
	s.az_mg = 1000;
	s.odom_valid = 1u;
	for (i = 0; i < 100; i++) {
		imu_update(&s);
	}
	imu_snapshot(&snap);
	check(snap.motion == IMU_MOTION_STATIONARY, "motion class = STATIONARY",
	      snap.motion, IMU_MOTION_STATIONARY);

	printf("\n[T11b] no odometry published yet (IF-02 not ready)\n");
	imu_init();
	memset(&s, 0, sizeof(s));
	s.az_mg = 1000;
	for (i = 0; i < 100; i++) {
		imu_update(&s);
	}
	imu_snapshot(&snap);
	check(snap.motion == IMU_MOTION_UNKNOWN, "motion = UNKNOWN, not a wrong guess",
	      snap.motion, IMU_MOTION_UNKNOWN);

	/* ---- T12: packing round-trip ---------------------------------- */
	printf("\n[T12] telemetry packing round-trip\n");
	word = imu_pack_terrain(4095u, IMU_HUMP_FALLING, IMU_MOTION_DESCENDING,
				(uint8_t)(IMU_FLAG_LIFTED | IMU_FLAG_SATURATED),
				0xC3u);
	imu_unpack_terrain(word, &snap, &seq);
	check(snap.hump_peak_mm == 4095u, "peak field survives the 14-bit field",
	      snap.hump_peak_mm, 4095);
	check(snap.hump_state == IMU_HUMP_FALLING, "hump state field", snap.hump_state,
	      IMU_HUMP_FALLING);
	check(snap.motion == IMU_MOTION_DESCENDING, "motion field", snap.motion,
	      IMU_MOTION_DESCENDING);
	check(snap.flags == (IMU_FLAG_LIFTED | IMU_FLAG_SATURATED),
	      "flags field", snap.flags,
	      (IMU_FLAG_LIFTED | IMU_FLAG_SATURATED));
	check(seq == 0xC3u, "sequence field", seq, 0xC3);

	att = imu_pack_attitude(1234, -567);
	imu_unpack_attitude(att, &snap);
	check_range(snap.pitch_cdeg, 1225, 1250, "pitch 12.34 deg -> 0.25 deg grid");
	check_range(snap.roll_cdeg, -600, -550, "roll -5.67 deg -> 0.25 deg grid");
	check((imu_pack_attitude(-4000, 0) & 0xFFu) == 0u,
	      "pitch below -32 deg clamps, does not wrap",
	      (long)(imu_pack_attitude(-4000, 0) & 0xFFu), 0);

	/* ---- T13: 32-bit record size ---------------------------------- */
	printf("\n[T13] record sizes\n");
	check(sizeof(uint32_t) == 4, "terrain record is 4 bytes", (long)sizeof(uint32_t), 4);
	check((word >> 24) == 0xC3u, "sequence occupies the top byte", (long)(word >> 24), 0xC3);

	/* ---- T14: overturned ------------------------------------------- */
	printf("\n[T14] rolled onto its roof at 1.5 s, and onto its side\n");
	imu_init();
	run_shape(IMU_SIM_FLIP, 0, 0, 250, 0, 400);
	imu_snapshot(&snap);
	check((snap.flags & IMU_FLAG_LIFTED) != 0, "upside down -> LIFTED flag",
	      (snap.flags & IMU_FLAG_LIFTED), IMU_FLAG_LIFTED);
	check(imu_lift_cause() == IMU_LIFT_OVERTURNED, "cause = OVERTURNED",
	      imu_lift_cause(), IMU_LIFT_OVERTURNED);
	imu_init();
	run_shape(IMU_SIM_FLIP, 0, 0, 250, 9000, 400);
	check(imu_lift_cause() == IMU_LIFT_OVERTURNED, "on its side -> OVERTURNED",
	      imu_lift_cause(), IMU_LIFT_OVERTURNED);
	imu_init();
	run_shape(IMU_SIM_FLIP, 0, 0, 250, 4500, 400);
	check(imu_lift_cause() == IMU_LIFT_NONE, "45 deg roll is not an overturn",
	      imu_lift_cause(), IMU_LIFT_NONE);

	/* ---- T15: knocks and vibration ---------------------------------- */
	printf("\n[T15] 10 s of vibration with a 1.5 g knock every 0.3 s\n");
	imu_init();
	run_shape(IMU_SIM_BUMP, 0, 0, 250, 0, 1000);
	imu_snapshot(&snap);
	check((snap.flags & IMU_FLAG_LIFTED) == 0, "knocks raise no LIFTED",
	      (snap.flags & IMU_FLAG_LIFTED), 0);

	/* ---- T16: latch and re-arm ------------------------------------- */
	printf("\n[T16] latched until imu_clear(), then armed again\n");
	imu_init();
	run_shape(IMU_SIM_LIFT, 0, 0, 250, 0, 400);
	run_shape(IMU_SIM_FLAT, 0, 0, 0, 0, 500);
	imu_snapshot(&snap);
	check((snap.flags & IMU_FLAG_LIFTED) != 0, "still latched 5 s later",
	      (snap.flags & IMU_FLAG_LIFTED), IMU_FLAG_LIFTED);
	imu_clear(IMU_FLAG_LIFTED);
	imu_snapshot(&snap);
	run_shape(IMU_SIM_FLAT, 0, 0, 0, 0, 1);
	imu_snapshot(&snap);
	check((snap.flags & IMU_FLAG_LIFTED) == 0, "imu_clear() releases it",
	      (snap.flags & IMU_FLAG_LIFTED), 0);
	run_shape(IMU_SIM_LIFT, 0, 0, 250, 0, 400);
	imu_snapshot(&snap);
	check((snap.flags & IMU_FLAG_LIFTED) != 0, "a second lift is caught",
	      (snap.flags & IMU_FLAG_LIFTED), IMU_FLAG_LIFTED);

	printf("\n----------------------------------------------\n");
	printf("%d passed, %d failed\n", g_pass, g_fail);
	return (g_fail == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}
