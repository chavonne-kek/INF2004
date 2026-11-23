/**
 * @file b5_types.h
 * @brief Minimum-bit data types shared by all Buddy 5 modules (Change Req. 1).
 *
 * Every field is sized to its valid range. See README "Data representation".
 */
#ifndef B5_TYPES_H
#define B5_TYPES_H

#include <stdint.h>
#include <stdbool.h>

/* ---- Distances / angles ------------------------------------------------ */
#define B5_DIST_MAX_CM      (250u)   /* beyond this = no echo               */
#define B5_DIST_NO_ECHO     (255u)   /* 8-bit sentinel                      */
#define B5_ANGLE_MAX_DEG    (180u)

/** One scan reading: 8-bit angle (0..180) + 8-bit range (1..250, 255=none). */
typedef struct
{
    uint16_t angle : 8;
    uint16_t dist  : 8;
} B5_Sample_t;

/* ---- Enumerations (3 bits each) ---------------------------------------- */
typedef enum
{
    B5_STATE_IDLE = 0,
    B5_STATE_COARSE,
    B5_STATE_FINE,
    B5_STATE_PLAN,
    B5_STATE_BYPASS,
    B5_STATE_RECOVER,
    B5_STATE_REVERSE,
    B5_STATE_FAULT          /* 8 states -> 3 bits */
} B5_State_t;

typedef enum
{
    B5_ACT_CONTINUE = 0,
    B5_ACT_STOP,
    B5_ACT_TURN_LEFT,
    B5_ACT_TURN_RIGHT,
    B5_ACT_REVERSE          /* 5 actions -> 3 bits */
} B5_Action_t;

typedef enum
{
    B5_SIDE_LEFT = 0,
    B5_SIDE_RIGHT
} B5_Side_t;

/* ---- Fault bitmask (4 bits, one bit per condition) ---------------------- */
/* Map these onto the team-wide shared fault mask (motor stall, encoder, IMU,
 * Wi-Fi loss ...) by shifting them into the bit positions the team agrees on. */
#define B5_FAULT_NO_ECHO        (0x01u)
#define B5_FAULT_LINE_LOST      (0x02u)
#define B5_FAULT_BYPASS_ABORT   (0x04u)
#define B5_FAULT_BLOCKED        (0x08u)
#define B5_FAULT_MASK           (0x0Fu)

/* ---- Obstacle profile: 49 payload bits ---------------------------------- */
/* RAM form: two 32-bit words (8 B). Wire form: 7 B via explicit bit packing. */
typedef struct
{
    uint32_t detected     : 1;
    uint32_t centreAngle  : 8;
    uint32_t closestAngle : 8;
    uint32_t closestDist  : 8;
    uint32_t width        : 8;   /* starts second 32-bit unit */
    uint32_t leftClear    : 8;
    uint32_t rightClear   : 8;
} B5_Profile_t;

#define B5_PROFILE_WIRE_BYTES   (7u)

/** Planner output: 3-bit action + 8-bit lateral shift. */
typedef struct
{
    uint16_t action  : 3;
    uint16_t shiftCm : 8;
} B5_Plan_t;

/* ---- Shared status word (13 bits in a uint16_t) ------------------------- */
#define B5_ST_STATE_SHIFT       (0u)
#define B5_ST_STATE_WIDTH       (3u)
#define B5_ST_ACTION_SHIFT      (3u)
#define B5_ST_ACTION_WIDTH      (3u)
#define B5_ST_OBSTACLE_SHIFT    (6u)
#define B5_ST_OBSTACLE_WIDTH    (1u)
#define B5_ST_RETRIES_SHIFT     (7u)
#define B5_ST_RETRIES_WIDTH     (2u)
#define B5_ST_FAULTS_SHIFT      (9u)
#define B5_ST_FAULTS_WIDTH      (4u)

#define B5_ST_GET(word, name) \
    ((uint16_t)(((word) >> B5_ST_##name##_SHIFT) & ((1u << B5_ST_##name##_WIDTH) - 1u)))

/* ---- Wire serialisation (explicit LSB-first bit packing) ---------------- */
void B5_Profile_Pack(const B5_Profile_t *profile, uint8_t out[B5_PROFILE_WIRE_BYTES]);
void B5_Profile_Unpack(const uint8_t in[B5_PROFILE_WIRE_BYTES], B5_Profile_t *profile);

/* ---- Shared status: every read-modify-write is inside a critical section - */
void     B5_Status_Init(void);
uint16_t B5_Status_Get(void);                 /* single aligned load: atomic */
void     B5_Status_SetState(B5_State_t state);
void     B5_Status_SetAction(B5_Action_t action);
void     B5_Status_SetObstacle(bool present);
void     B5_Status_SetRetries(uint8_t retries);
void     B5_Status_RaiseFault(uint8_t faultBits);
void     B5_Status_ClearFault(uint8_t faultBits);

#endif /* B5_TYPES_H */
