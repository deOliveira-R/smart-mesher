/* Acceptance 3 and 4 from C: DAGMC shims on the stub sphere, status codes, thread-local error. */
#include <math.h>
#include <string.h>
#include "check.h"
#include "smrt_test.h"

int main(void) {
    const double R = 5.0;
    SmrtTracker *t = NULL;
    SmrtCtx *c = NULL;
    CHECK(smrt_test_stub(STUB_SPHERE, R, &t) == 0);
    CHECK(smrt_ctx_new(t, &c) == 0);

    int64_t surf = 0, face = 0;
    double dist = 0.0;
    const double dir[3] = {1.0, 0.0, 0.0};
    const double in_pt[3] = {-2.0, 0.0, 0.0}, out_pt[3] = {-9.0, 0.0, 0.0};
    CHECK(smrt_dag_ray_fire(t, c, 0, in_pt, dir, SMRT_ORIENT_EXITING, 0.0, &surf, &dist, &face) == 0);
    CHECK(surf == 0);
    CHECK_NEAR(dist, 7.0, 1e-12);
    CHECK(smrt_dag_ray_fire(t, c, 0, out_pt, dir, SMRT_ORIENT_ENTERING, 0.0, &surf, &dist, NULL) == 0);
    CHECK_NEAR(dist, 4.0, 1e-12);
    CHECK(smrt_dag_ray_fire(t, c, 0, out_pt, dir, SMRT_ORIENT_ANY, 0.0, &surf, &dist, NULL) == 0);
    CHECK_NEAR(dist, 4.0, 1e-12);
    CHECK(smrt_dag_ray_fire(t, c, 0, out_pt, dir, SMRT_ORIENT_EXITING, 0.0, &surf, &dist, NULL) == SMRT_NO_HIT);

    /* point_in_volume for 1e5 seeded points */
    for (int i = 0; i < 100000; i++) {
        double p[3] = {(unit01() - 0.5) * 18.0, (unit01() - 0.5) * 18.0, (unit01() - 0.5) * 18.0};
        double r = sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
        if (fabs(r - R) < 1e-9) continue;
        int32_t in_s = -1, in_c = -1;
        CHECK(smrt_dag_point_in_volume(t, c, 0, p, NULL, &in_s) == 0);
        CHECK(smrt_dag_point_in_volume(t, c, SMRT_COMPLEMENT, p, NULL, &in_c) == 0);
        CHECK(in_s == (r < R) && in_c == (r > R));
    }

    /* sense and next_vol agree; the complement id comes back outside */
    int32_t sense = 0;
    int64_t next = 0;
    CHECK(smrt_dag_surface_sense(t, 0, 0, &sense) == 0 && sense == 1);
    CHECK(smrt_dag_next_vol(t, 0, 0, &next) == 0 && next == SMRT_COMPLEMENT);
    CHECK(smrt_dag_surface_sense(t, 0, SMRT_COMPLEMENT, &sense) == 0 && sense == -1);
    CHECK(smrt_dag_next_vol(t, 0, SMRT_COMPLEMENT, &next) == 0 && next == 0);
    SmrtLocated loc;
    const double far_pt[3] = {8.0, 8.0, 8.0};
    CHECK(smrt_locate(c, far_pt, NULL, -1, &loc) == 0 && loc.region == SMRT_COMPLEMENT);

    double n[3], vol = 0.0, area = 0.0;
    const double on_surf[3] = {0.0, 5.0, 0.0};
    CHECK(smrt_dag_get_angle(t, 0, on_surf, n) == 0);
    CHECK_NEAR(n[1], 1.0, 1e-12);
    CHECK(smrt_dag_measure_volume(t, 0, &vol) == 0);
    CHECK_NEAR(vol, 4.0 / 3.0 * M_PI * R * R * R, 1e-9);
    CHECK(smrt_dag_measure_area(t, 0, &area) == SMRT_ERR_UNSUPPORTED);

    /* status and error message */
    CHECK(smrt_ctx_new(NULL, &c) == SMRT_ERR_NULL);
    char msg[128];
    CHECK(smrt_last_error_message(msg, sizeof msg) > 0 && strstr(msg, "null") != NULL);
    CHECK(smrt_init_state(c, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1) == SMRT_ERR_BAD_DIRECTION);
    CHECK(smrt_last_error_message(msg, sizeof msg) > 0 && strstr(msg, "direction") != NULL);
    SmrtTracker *none = NULL;
    CHECK(smrt_open("x.smrt", &none) == SMRT_ERR_UNSUPPORTED && none == NULL);

    /* a stub whose step() panics: the process survives with -101 */
    SmrtTracker *pt = NULL;
    SmrtCtx *pc = NULL;
    SmrtStep s;
    CHECK(smrt_test_stub(STUB_PANIC_ON_STEP, R, &pt) == 0 && smrt_ctx_new(pt, &pc) == 0);
    CHECK(smrt_init_state(pc, -9.0, 0.0, 0.0, 1.0, 0.0, 0.0, -1) == 0);
    CHECK(smrt_step(pc, 1.0, &s) == SMRT_ERR_PANIC);
    CHECK(smrt_last_error_message(msg, sizeof msg) > 0 && strstr(msg, "deliberate panic") != NULL);
    CHECK(smrt_fire(pc, INFINITY, 0, &s) == SMRT_ERR_PANIC);

    CHECK(smrt_ctx_free(pc) == 0 && smrt_tracker_free(pt) == 0);
    CHECK(smrt_ctx_free(c) == 0 && smrt_tracker_free(t) == 0);
    puts("dag: ok");
    return 0;
}
