/* Flights on the stub majorant world (feature `stubs`): the same transport code as handover.c on
 * STUB_GRID (a [2,2,2] partition), plus the contained panic of a tracker whose advance_segment
 * panics. The CASE lines are compared by the Rust test with the Rust path, bit for bit. */
#include "flight_common.h"
#include "smrt_test.h"

int main(void) {
    SmrtTracker *t = NULL;
    CHECK(smrt_test_stub(STUB_GRID, 5.0, &t) == 0);
    Model m;
    model_load(t, &m);
    CHECK(m.nnodes == 9);

    const int64_t nhist = 10000;
    run_case("stub_surface", t, &m, 0, nhist, 0x4444);
    run_case("stub_delta", t, &m, 1, nhist, 0x5555);
    run_case("stub_delta_block", t, &m, 2, nhist, 0x6666);

    SmrtCtx *ctx = NULL;
    SmrtState st;
    CHECK(smrt_ctx_new(t, &ctx) == 0);
    CHECK(smrt_init_state(ctx, 1.0, 2.0, -3.0, 0.6, 0.0, 0.8, -1) == 0);
    memset(&st, 0, sizeof st);
    CHECK(smrt_get_state(ctx, &st) == 0);
    misuse_checks(t, &m, &st);
    CHECK(smrt_ctx_free(ctx) == 0);

    /* a panicking tracker returns SMRT_ERR_PANIC and the process survives */
    SmrtTracker *p = NULL;
    SmrtFlight *f = NULL;
    SmrtSegment seg;
    CHECK(smrt_test_stub(STUB_PANIC_ON_SEGMENT, 5.0, &p) == 0);
    Model pm;
    model_load(p, &pm);
    SmrtFlightOpts o = {0.0, 0, 0, SMRT_POLICY_SURFACE, SMRT_SCALE_CELL};
    CHECK(smrt_flight_new(p, &st, &o, &f) == 0);
    CHECK(smrt_flight_next(f, pm.maj, (size_t)pm.nnodes, NULL, 0, &seg) == SMRT_ERR_PANIC);
    char msg[256];
    CHECK(smrt_last_error_message(msg, sizeof msg) > 0 && strstr(msg, "panic") != NULL);
    CHECK(smrt_flight_free(f) == 0);
    model_free(&pm);
    CHECK(smrt_tracker_free(p) == 0);

    model_free(&m);
    CHECK(smrt_tracker_free(t) == 0);
    printf("flight_stub: ok\n");
    return 0;
}
