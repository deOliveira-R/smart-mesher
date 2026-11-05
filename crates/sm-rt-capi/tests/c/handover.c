/* Transport handover from C (W3b acceptance 1, 2, 4): open the .smrt written by sm-rt, read node
 * sets and region names, assign sigma per name, fly 1e4 surface and delta histories. The CASE
 * lines are compared by the Rust test with the Rust path, bit for bit.
 * usage: handover <file.smrt> <corrupt.smrt> */
#include "flight_common.h"

int main(int argc, char **argv) {
    CHECK(argc == 3);
    SmrtTracker *t = NULL;
    CHECK(smrt_open(argv[1], &t) == 0 && t != NULL);

    /* failures leave *out null and return typed codes */
    SmrtTracker *bad = (SmrtTracker *)&bad;
    CHECK(smrt_open("/nonexistent/dir/none.smrt", &bad) == SMRT_ERR_BAD_ARG && bad == NULL);
    char msg[512];
    CHECK(smrt_last_error_message(msg, sizeof msg) > 0 && strstr(msg, "none.smrt") != NULL);
    bad = (SmrtTracker *)&bad;
    CHECK(smrt_open(argv[2], &bad) == SMRT_ERR_CACHE_CORRUPT && bad == NULL);
    CHECK(smrt_open(NULL, &bad) == SMRT_ERR_NULL);
    CHECK(smrt_open(argv[1], NULL) == SMRT_ERR_NULL);

    Model m;
    model_load(t, &m);
    CHECK(m.nreg >= 2 && m.nnodes > 2);
    for (int64_t r = 0; r < m.nreg; r++) {
        char name[64];
        int32_t n = smrt_region_name(t, r, name, sizeof name);
        printf("REGION %lld %s sigma=%g\n", (long long)r, name, m.sig[r]);
        (void)n;
    }
    printf("NODES %lld\n", (long long)m.nnodes);

    const int64_t nhist = 10000;
    run_case("surface", t, &m, 0, nhist, 0x1111);
    run_case("delta", t, &m, 1, nhist, 0x2222);
    run_case("delta_block", t, &m, 2, nhist, 0x3333);

    /* misuse and the understated majorant, from a located start inside the model */
    SmrtCtx *ctx = NULL;
    SmrtState st;
    CHECK(smrt_ctx_new(t, &ctx) == 0);
    double c[3];
    for (int k = 0; k < 3; k++) c[k] = 0.5 * (m.info.bounds_lo[k] + m.info.bounds_hi[k]);
    CHECK(smrt_init_state(ctx, c[0] + 0.01, c[1] + 0.02, c[2] + 0.03, 0.6, 0.0, 0.8, -1) == 0);
    memset(&st, 0, sizeof st);
    CHECK(smrt_get_state(ctx, &st) == 0);
    misuse_checks(t, &m, &st);
    CHECK(smrt_ctx_free(ctx) == 0);

    model_free(&m);
    CHECK(smrt_tracker_free(t) == 0);
    printf("handover: ok\n");
    return 0;
}
