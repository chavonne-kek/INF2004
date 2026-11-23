/**
 * @file b5_types.c
 * @brief Bit packing for the wire format and race-free shared status word.
 */
#include "b5_types.h"
#include "b5_port.h"

#define BITS_PER_BYTE   (8u)

/* ---- Bit writer / reader (LSB first) ------------------------------------ */

static void PutBits(uint8_t *buf, uint8_t *bitPos, uint32_t value, uint8_t width)
{
    uint8_t i;

    for (i = 0u; i < width; i++)
    {
        const uint8_t pos = (uint8_t)(*bitPos + i);
        const uint8_t byteIdx = (uint8_t)(pos / BITS_PER_BYTE);
        const uint8_t bitIdx = (uint8_t)(pos % BITS_PER_BYTE);

        if (((value >> i) & 1u) != 0u)
        {
            buf[byteIdx] = (uint8_t)(buf[byteIdx] | (1u << bitIdx));
        }
    }
    *bitPos = (uint8_t)(*bitPos + width);
}

static uint32_t GetBits(const uint8_t *buf, uint8_t *bitPos, uint8_t width)
{
    uint32_t value = 0u;
    uint8_t i;

    for (i = 0u; i < width; i++)
    {
        const uint8_t pos = (uint8_t)(*bitPos + i);
        const uint8_t byteIdx = (uint8_t)(pos / BITS_PER_BYTE);
        const uint8_t bitIdx = (uint8_t)(pos % BITS_PER_BYTE);

        if (((buf[byteIdx] >> bitIdx) & 1u) != 0u)
        {
            value |= (1uL << i);
        }
    }
    *bitPos = (uint8_t)(*bitPos + width);
    return value;
}

void B5_Profile_Pack(const B5_Profile_t *profile, uint8_t out[B5_PROFILE_WIRE_BYTES])
{
    uint8_t pos = 0u;

    for (pos = 0u; pos < B5_PROFILE_WIRE_BYTES; pos++)   /* IS25: no memset */
    {
        out[pos] = 0u;
    }
    pos = 0u;
    PutBits(out, &pos, profile->detected, 1u);
    PutBits(out, &pos, profile->centreAngle, 8u);
    PutBits(out, &pos, profile->closestAngle, 8u);
    PutBits(out, &pos, profile->closestDist, 8u);
    PutBits(out, &pos, profile->width, 8u);
    PutBits(out, &pos, profile->leftClear, 8u);
    PutBits(out, &pos, profile->rightClear, 8u);
}

void B5_Profile_Unpack(const uint8_t in[B5_PROFILE_WIRE_BYTES], B5_Profile_t *profile)
{
    uint8_t pos = 0u;

    *profile = (B5_Profile_t){0};   /* IS25: no memset */
    profile->detected = (uint8_t)(GetBits(in, &pos, 1u) & 1u);
    profile->centreAngle = (uint8_t)GetBits(in, &pos, 8u);
    profile->closestAngle = (uint8_t)GetBits(in, &pos, 8u);
    profile->closestDist = (uint8_t)GetBits(in, &pos, 8u);
    profile->width = (uint8_t)GetBits(in, &pos, 8u);
    profile->leftClear = (uint8_t)GetBits(in, &pos, 8u);
    profile->rightClear = (uint8_t)GetBits(in, &pos, 8u);
}

/* ---- Shared status word -------------------------------------------------- */

/* Written by several tasks, read by others (e.g. telemetry). The word is
 * 16 bits, so a plain read is atomic on RP2040; every modify is a masked
 * read-modify-write done with interrupts disabled so no update is lost. */
static volatile uint16_t s_status;

static void StatusModify(uint8_t shift, uint8_t width, uint16_t value)
{
    const uint16_t fieldMask = (uint16_t)(((1u << width) - 1u) << shift);
    const B5_Irq_t irq = B5_Port_Enter();

    s_status = (uint16_t)((s_status & (uint16_t)~fieldMask) |
                          (uint16_t)((value << shift) & fieldMask));
    B5_Port_Exit(irq);
}

void B5_Status_Init(void)
{
    const B5_Irq_t irq = B5_Port_Enter();

    s_status = 0u;
    B5_Port_Exit(irq);
}

uint16_t B5_Status_Get(void)
{
    return s_status;
}

void B5_Status_SetState(B5_State_t state)
{
    StatusModify(B5_ST_STATE_SHIFT, B5_ST_STATE_WIDTH, (uint16_t)state);
}

void B5_Status_SetAction(B5_Action_t action)
{
    StatusModify(B5_ST_ACTION_SHIFT, B5_ST_ACTION_WIDTH, (uint16_t)action);
}

void B5_Status_SetObstacle(bool present)
{
    StatusModify(B5_ST_OBSTACLE_SHIFT, B5_ST_OBSTACLE_WIDTH, present ? 1u : 0u);
}

void B5_Status_SetRetries(uint8_t retries)
{
    StatusModify(B5_ST_RETRIES_SHIFT, B5_ST_RETRIES_WIDTH, retries);
}

void B5_Status_RaiseFault(uint8_t faultBits)
{
    const B5_Irq_t irq = B5_Port_Enter();

    s_status = (uint16_t)(s_status |
               (uint16_t)((faultBits & B5_FAULT_MASK) << B5_ST_FAULTS_SHIFT));
    B5_Port_Exit(irq);
}

void B5_Status_ClearFault(uint8_t faultBits)
{
    const B5_Irq_t irq = B5_Port_Enter();

    s_status = (uint16_t)(s_status &
               (uint16_t)~(uint16_t)((faultBits & B5_FAULT_MASK) << B5_ST_FAULTS_SHIFT));
    B5_Port_Exit(irq);
}
