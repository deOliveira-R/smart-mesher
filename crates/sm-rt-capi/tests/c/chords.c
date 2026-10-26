/* Acceptance 1 and 2 from C: sphere chords, nested shells, mirror deck, white boundary,
 * 2-level lattice with slot changes. Built by tests/c_abi.rs and linked to the cdylib. */
#include <math.h>
#include <string.h>
#include "check.h"
#include "smrt_test.h"

static SmrtCtx *new_ctx(SmrtTracker *t) {
    SmrtCtx *c = NULL;
    CHECK(smrt_ctx_new(t, &c) == 0 && c != NULL);
    return c;
}

static void test_sphere_chords(void) {
    const double R = 5.0, bs[3] = {0.0, 3.0, 4.9};
    SmrtTracker *t = NULL;
    CHECK(smrt_test_stub(STUB_SPHERE, R, &t) == 0);
    SmrtCtx *c = new_ctx(t);
    for (int i = 0; i < 3; i++) {
        SmrtStep s;
        CHECK(smrt_init_state(c, -9.0, bs[i], 0.0, 1.0, 0.0, 0.0, -1) == 0);
        CHECK(smrt_fire(c, INFINITY, 0, &s) == 0 && s.end == SMRT_END_REGION_BOUNDARY && s.to_region == 0);
        CHECK(smrt_fire(c, INFINITY, 0, &s) == 0 && s.end == SMRT_END_REGION_BOUNDARY);
        CHECK(s.to_region == SMRT_COMPLEMENT);
        CHECK_NEAR(s.dist, 2.0 * sqrt(R * R - bs[i] * bs[i]), 1e-12);
    }
    CHECK(smrt_ctx_free(c) == 0 && smrt_tracker_free(t) == 0);
}

static void test_shells(void) {
    const double R = 5.0, bs[4] = {0.0, 1.5, 3.0, 4.9};
    SmrtTracker *t = NULL;
    CHECK(smrt_test_stub(STUB_SHELLS, R, &t) == 0);
    SmrtCtx *c = new_ctx(t);
    for (int i = 0; i < 4; i++) {
        SmrtStep s;
        double sum = 0.0;
        CHECK(smrt_init_state(c, -9.0, bs[i], 0.0, 1.0, 0.0, 0.0, -1) == 0);
        CHECK(smrt_fire(c, INFINITY, 0, &s) == 0); /* enter the outer shell */
        for (;;) {
            CHECK(smrt_fire(c, INFINITY, 0, &s) == 0 && s.end == SMRT_END_REGION_BOUNDARY);
            sum += s.dist;
            if (s.to_region == SMRT_COMPLEMENT) break;
        }
        CHECK_NEAR(sum, 2.0 * sqrt(R * R - bs[i] * bs[i]), 1e-12);
    }
    CHECK(smrt_ctx_free(c) == 0 && smrt_tracker_free(t) == 0);
}

static void test_mirror(void) {
    SmrtTracker *t = NULL;
    CHECK(smrt_test_stub(STUB_MIRROR_BOX, 0.0, &t) == 0);
    SmrtCtx *c = new_ctx(t);
    SmrtStep s;
    SmrtState st;
    CHECK(smrt_init_state(c, 0.0, 0.0, 0.0, 0.6, 0.0, 0.8, -1) == 0);
    CHECK(smrt_step(c, INFINITY, &s) == 0 && s.end == SMRT_END_BOUNDARY && s.action == SMRT_ACT_MIRROR);
    CHECK(smrt_get_state(c, &st) == 0);
    CHECK(st.dir[0] == 0.6 && st.dir[1] == 0.0 && st.dir[2] == -0.8); /* bit exact */
    CHECK(smrt_ctx_free(c) == 0 && smrt_tracker_free(t) == 0);
}

static void test_white(void) {
    SmrtTracker *t = NULL;
    CHECK(smrt_test_stub(STUB_WHITE_BOX, 0.0, &t) == 0);
    SmrtCtx *c = new_ctx(t);
    SmrtStep s;
    double d[3];
    CHECK(smrt_init_state(c, 1.0, 2.0, 3.0, 1.0, 0.0, 0.0, -1) == 0);
    CHECK(smrt_step(c, INFINITY, &s) == 0 && s.end == SMRT_END_BOUNDARY && s.action == SMRT_ACT_WHITE);
    for (int i = 0; i < 100000; i++) {
        double u0 = unit01(), u1 = unit01();
        CHECK(smrt_resolve_boundary(c, s.face, SMRT_RESP_WHITE, 0.0, u0, u1, d) == 0);
        CHECK_NEAR(sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]), 1.0, 1e-12);
        CHECK(d[0] <= 1e-12); /* inward at the +x wall */
    }
    CHECK(smrt_ctx_free(c) == 0 && smrt_tracker_free(t) == 0);
}

static void test_lattice(void) {
    SmrtTracker *t = NULL;
    CHECK(smrt_test_stub(STUB_LATTICE, 1.0, &t) == 0);
    SmrtCtx *c = new_ctx(t);
    SmrtInfo info;
    CHECK(smrt_info(t, &info) == 0 && info.cells == 54 && info.copies >= 1);
    const double offs[2] = {0.0, 0.6};
    for (int k = 0; k < 2; k++) {
        SmrtStep s;
        double sum = 0.0;
        int slots = 0;
        CHECK(smrt_init_state(c, 0.25, 2.0 + offs[k], 2.0, 1.0, 0.0, 0.0, -1) == 0);
        for (;;) {
            CHECK(smrt_step(c, INFINITY, &s) >= 0);
            if (s.region == 0) sum += s.dist;
            if (s.end == SMRT_END_SLOT_CHANGE) slots++;
            CHECK(s.end != SMRT_END_LOST);
            if (s.end == SMRT_END_BOUNDARY) break;
        }
        CHECK(slots == 2);
        CHECK_NEAR(sum, 3.0 * 2.0 * sqrt(1.0 - offs[k] * offs[k]), 1e-12);
    }
    /* step reports the slot change; fire skips it. */
    SmrtStep s;
    SmrtState st;
    CHECK(smrt_init_state(c, 3.5, 2.0, 2.0, 1.0, 0.0, 0.0, -1) == 0);
    CHECK(smrt_step(c, INFINITY, &s) == 0 && s.end == SMRT_END_SLOT_CHANGE);
    CHECK_NEAR(s.dist, 0.5, 1e-12);
    CHECK(smrt_get_state(c, &st) == 0 && st.levels_total == 1 && st.levels_len == 1);
    CHECK(smrt_init_state(c, 3.5, 2.0, 2.0, 1.0, 0.0, 0.0, -1) == 0);
    CHECK(smrt_fire(c, INFINITY, 0, &s) == 0 && s.end == SMRT_END_REGION_BOUNDARY && s.to_region == 0);
    CHECK_NEAR(s.dist, 1.5, 1e-12);
    CHECK(smrt_ctx_free(c) == 0 && smrt_tracker_free(t) == 0);
}

int main(void) {
    CHECK(smrt_version() == SMRT_ABI_VERSION);
    test_sphere_chords();
    test_shells();
    test_mirror();
    test_white();
    test_lattice();
    puts("chords: ok");
    return 0;
}
