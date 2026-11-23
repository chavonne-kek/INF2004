/** @file test_host.c
 *
 * @brief PC tests for subsystem 4's telemetry helpers (scan_calc.c).
 *        Buddy 5's profiler and planner have their own tests in
 *        ../buddy5_host (his original host_test.c).
 */

#include <stdlib.h>

#include "../../host_common/host_check.h"
#include "scan_calc.h"

int
main (void)
{
    int ok = 1;

    HOST_CHECK("H-SC01", (scan_echo_to_cm(1160u) == 20u) &&
               (scan_echo_to_cm(0u) == 0u) &&
               (scan_echo_to_cm(7366u) == 127u) &&
               (scan_echo_to_cm(38000u) == 127u),
               "echo -> cm, >= 127 cm saturates to 'none'");

    HOST_CHECK("H-SC02", (scan_index_to_deg(0u) == 30u) &&
               (scan_index_to_deg(12u) == 90u) &&
               (scan_index_to_deg(24u) == 150u) &&
               (scan_index_to_deg(31u) == 150u),
               "5-bit scan index 0..24 = 30..150 deg");

    srand(5u);
    for (int i = 0; i < 10000; i++)
    {
        subsystem4_profile_t a;
        subsystem4_profile_t b;
        uint8_t              bytes[S4_PROFILE_BYTES];

        a.width_cm       = (uint8_t)(rand() % 128);
        a.closest_cm     = (uint8_t)(rand() % 128);
        a.closest_index  = (uint8_t)(rand() % 25);
        a.left_clear_cm  = (uint8_t)(rand() % 128);
        a.right_clear_cm = (uint8_t)(rand() % 128);
        a.b_left_edge    = (rand() & 1) != 0;
        a.b_right_edge   = (rand() & 1) != 0;
        scan_profile_pack(&a, bytes);
        scan_profile_unpack(bytes, &b);
        ok = ok && (0 == (bytes[4] & 0xF8u)) &&
             (a.width_cm == b.width_cm) && (a.closest_cm == b.closest_cm) &&
             (a.closest_index == b.closest_index) &&
             (a.left_clear_cm == b.left_clear_cm) &&
             (a.right_clear_cm == b.right_clear_cm) &&
             (a.b_left_edge == b.b_left_edge) &&
             (a.b_right_edge == b.b_right_edge);
    }
    HOST_CHECK("H-SC03", ok, "telemetry profile 35 bits / 5 bytes round trip");

    return (HOST_SUMMARY());
}

/*** end of file ***/
