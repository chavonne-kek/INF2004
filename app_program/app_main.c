/*
 *	app_main.c
 *	START HERE.  usermain() below is your program's entry point.
 *
 *	How micro T-Kernel starts your code
 *	-----------------------------------
 *	  reset -> kernel initialises itself -> creates the "initial task"
 *	        -> initial task calls usermain()
 *
 *	usermain() runs inside that initial task at priority 1 (the highest).
 *	On a single core nothing you create runs until usermain() blocks, but
 *	on the dual-core build the OTHER core may start a new task at once.
 *	So usermain() works in two phases:
 *	  1. every subsystem creates its kernel objects  (subsystemN_init)
 *	  2. every subsystem starts its tasks             (subsystemN_start)
 *	and then puts the initial task to sleep forever.
 *	If usermain() RETURNS, the kernel shuts the whole system down.
 *
 *	The IS25 robot car (TEAM.md has the full tables)
 *	-----------------------------------------------
 *	  subsystem1/  IR line following + barcodes  line_task  12  core 1
 *	  subsystem2/  motors, encoders, PID         motion_task 14  core 1
 *	  subsystem3/  IMU, humps, collisions        imu_task   16  core 0
 *	  subsystem4/  vehicle controller            ctrl_task  18  core 1
 *	               ultrasonic + servo scan       scan_task  19  core 0
 *	  subsystem5/  WiFi / MQTT telemetry          comms_task 20  core 0
 *
 *	Interfaces (TEAM.md section 5):
 *	  IF-01  S1 -> S2      line word (16 bits, one writer, no lock)
 *	  IF-02  S2 -> S3/4/5  odometry word (32 bits, one writer, no lock)
 *	  IF-03  all -> S4     flg_events bits; flg_faults fault bitmask
 *	  IF-04  S4 -> S2      motion commands (only TSK_CTRL sends)
 *	  IF-06  all -> S5     event telemetry records
 *	  IF-07  S5 <-> kernel WiFi service (lib_patches/, needs approval)
 *	The subsystem that RECEIVES or OWNS the data creates the object and
 *	publishes it through its header; the other side just uses it.
 *
 *	micro T-Kernel APIs used, and where to find them:
 *	  tasks       tk_cre_tsk tk_sta_tsk tk_slp_tsk tk_wup_tsk tk_can_wup
 *	              tk_dly_tsk tk_ref_tsk                 app_main.c, all
 *	  cyclic      tk_cre_cyc tk_sta_cyc tk_ref_cyc      S1, S2, S3
 *	  event flag  tk_cre_flg tk_set_flg tk_clr_flg tk_wai_flg tk_ref_flg  S4
 *	  msg buffer  tk_cre_mbf tk_snd_mbf tk_rcv_mbf      S1, S2, S4, S5
 *	  semaphore   tk_cre_sem tk_wai_sem tk_sig_sem      S4 (echo)
 *	  interrupt   tk_def_int EnableInt                  S2 (GPIO bank 0)
 *	  devices     tk_opn_dev tk_srea_dev tk_swri_dev    S1 (ADC), S3 (I2C)
 *	  time        tk_get_otm                            S4, S5
 *	  system      tk_ref_ver                            app_main.c
 *
 *	Output appears on the USB serial port (115200 8N1 in any terminal).
 */

#include "app.h"

/*
 * Create and start one task (declared in app.h for every subsystem).
 *   prc = TP_PRC1 (core 0), TP_PRC2 (core 1), or 0 for "any core".
 */
EXPORT ID app_start_task(FP entry, PRI pri, UW prc, const char *name)
{
	T_CTSK	ctsk = {0};
	ID	tid;
	ER	er;

	ctsk.task    = entry;		/* function the task runs */
	ctsk.itskpri = pri;		/* initial priority (1 = highest) */
	ctsk.stksz   = TASK_STACK_SIZE;	/* kernel allocates the stack */
	ctsk.tskatr  = TA_HLNG		/* written in a high-level language (C) */
		     | TA_RNG3;		/* runs at user protection level */
#if TK_SUPPORT_SMP
	if (prc != 0) {
		ctsk.tskatr |= TA_ASSPRC;	/* honour the affinity below */
		ctsk.assprc  = prc;		/* which core(s) it may run on */
	}
#else
	(void)prc;			/* single-core build: no affinity */
#endif

	/* tk_cre_tsk: create the task.  It starts in the DORMANT state. */
	tid = tk_cre_tsk(&ctsk);
	if (tid < E_OK) {
		CHECK(tid, name);
		return tid;
	}

	/* tk_sta_tsk: DORMANT -> READY.  The int is passed as stacd. */
	er = tk_sta_tsk(tid, 0);
	CHECK(er, name);
	return tid;
}

EXPORT INT usermain(void)
{
	T_RVER	ver;

	/* tk_ref_ver: ask the kernel who it is */
	tk_ref_ver(&ver);
	tm_printf((UB*)"\n=== micro T-Kernel on Raspberry Pi Pico W"
		  " (spec 0x%04x, product 0x%04x) ===\n", ver.spver, ver.prver);
#if TK_SUPPORT_SMP
	tm_printf((UB*)"dual-core (SMP) build, usermain on core %d\n",
		  tk_get_prc() - 1);	/* tk_get_prc() counts from 1 */
#else
	tm_printf((UB*)"single-core build\n");
#endif

	/* Phase 1: every subsystem creates its kernel objects */
	CHECK(subsystem1_init(), "subsystem1_init");
	CHECK(subsystem2_init(), "subsystem2_init");
	CHECK(subsystem3_init(), "subsystem3_init");
	CHECK(subsystem4_init(), "subsystem4_init");
	CHECK(subsystem5_init(), "subsystem5_init");

	/* Phase 2: every subsystem starts its tasks */
	CHECK(subsystem1_start(), "subsystem1_start");
	CHECK(subsystem2_start(), "subsystem2_start");
	CHECK(subsystem3_start(), "subsystem3_start");
	CHECK(subsystem4_start(), "subsystem4_start");
	CHECK(subsystem5_start(), "subsystem5_start");

	tm_printf((UB*)"all subsystems started\n\n");

#if APP_TEST && (APP_CHECK != 0)
	/* Check image (make CHECK=n): one subsystem's tests, then a live
	   readout of its hardware.  The other subsystems run as usual (they
	   provide what this one needs), but the car does not drive. */
	tm_printf((UB*)"[CHECK] === subsystem %d check image ===\n", APP_CHECK);
	tm_printf((UB*)"[TEST] === test run start ===\n");
#if APP_CHECK == 1
	subsystem1_test();
	subsystem1_check();
#elif APP_CHECK == 2
	subsystem2_test();
	subsystem2_check();
#elif APP_CHECK == 3
	subsystem3_test();
	subsystem3_check();
#elif APP_CHECK == 4
	subsystem4_test();
	subsystem4_check();
#elif APP_CHECK == 6
	subsystem1_test();
	subsystem1_barcode_bench();
#else
	subsystem5_test();
	subsystem5_check();
#endif
#elif APP_TEST
	/* Test build (make TEST=1): run every subsystem's tests in turn,
	   here in the initial task, while the system is running.  Tests may
	   call tk_dly_tsk() to let the other tasks work for a while. */
	tm_printf((UB*)"[TEST] === test run start ===\n");
	subsystem1_test();
	subsystem2_test();
	subsystem3_test();
	subsystem4_test();
	subsystem5_test();
	tm_printf((UB*)"[TEST] === test run end ===\n");
#endif

	/* Sleep forever.  Returning from usermain() would shut down. */
	tk_slp_tsk(TMO_FEVR);
	return 0;
}
