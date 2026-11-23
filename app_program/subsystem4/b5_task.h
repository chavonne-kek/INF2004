/**
 * @file b5_task.h
 * @brief Buddy 5 task: cruise watch -> scan -> profile -> plan -> bypass -> recover.
 */
#ifndef B5_TASK_H
#define B5_TASK_H

#include <stdint.h>
#include <stdbool.h>
#include "b5_if.h"
#include "b5_types.h"

/** Initialise servo, sonar, status word. Call once before starting the task. */
void B5_Task_Init(const B5_Interfaces_t *io);

/** Task body: never returns. Pass this to tk_cre_tsk (see README). */
void B5_Task_Run(void);

/* 22 Nov: pull the obstacle engine back to watching (called on START/RESET). */
void B5_Task_RequestWatchReset(void);

/* 22 Nov: the cruise watch's last reading and hit count, for telemetry. */
uint8_t B5_WatchCm(void);
uint8_t B5_WatchHits(void);

/** Latest profile (consistent copy; safe from any task). */
void B5_Task_GetProfile(B5_Profile_t *out);

/** Shared status word for telemetry (see B5_ST_GET). */
uint16_t B5_Task_GetStatus(void);

#endif /* B5_TASK_H */
