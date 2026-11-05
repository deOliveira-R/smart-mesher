/* Shared by the flight C tests: a tiny transport "code" that owns its cross sections.
 *
 * sigma_of(name) is the code's cross section per material name. The library never sees it: the
 * code reads node sets and region names, pairs names with sigma_of and passes one majorant per
 * node to every call. The Rust side (tests/common/flight_ref.rs) runs the identical algorithm on
 * the Rust API; the printed lines must be equal bit for bit (only + - * / and sqrt are used, and
 * the C files are compiled with -ffp-contract=off). */
#ifndef FLIGHT_COMMON_H
#define FLIGHT_COMMON_H
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "check.h"
#include "smrt.h"

static double sigma_of(const char *name) {
    unsigned s = 0;
    for (const char *p = name; *p; p++) s += (unsigned char)*p;
    return 0.5 + (double)(s % 5) * 0.25;
}

typedef struct {
    int64_t nreg, comp, nnodes;
    double *sig; /* per region */
    double *maj; /* per node: max sigma over the regions of the node's set */
    SmrtInfo info;
} Model;

static int64_t ridx(const Model *m, int64_t r) {
    if (r == SMRT_COMPLEMENT) return m->comp;
    CHECK(r >= 0 && r < m->nreg);
    return r;
}

static void model_load(const SmrtTracker *t, Model *m) {
    CHECK(smrt_info(t, &m->info) == 0);
    m->nreg = m->info.regions;
    m->comp = m->info.complement_region;
    m->sig = (double *)malloc((size_t)m->nreg * sizeof(double));
    CHECK(m->sig != NULL);
    for (int64_t r = 0; r < m->nreg; r++) {
        char buf[256];
        int32_t n = smrt_region_name(t, r, buf, sizeof buf);
        CHECK(n >= 0 && n < (int32_t)sizeof buf);
        m->sig[r] = sigma_of(buf);
    }
    int64_t nsets = 0;
    CHECK(smrt_n_nodes(t, &m->nnodes) == 0 && m->nnodes > 0);
    CHECK(smrt_n_sets(t, &nsets) == 0 && nsets > 0);
    m->maj = (double *)malloc((size_t)m->nnodes * sizeof(double));
    CHECK(m->maj != NULL);
    for (int64_t node = 0; node < m->nnodes; node++) {
        int64_t set = -1, ids[64], n = 0;
        CHECK(smrt_node_set(t, node, &set) == 0 && set >= 0 && set < nsets);
        CHECK(smrt_set_regions(t, set, ids, 64, &n) == 0 && n >= 1 && n <= 64);
        double mx = 0.0;
        for (int64_t i = 0; i < n; i++) {
            double v = m->sig[ridx(m, ids[i])];
            if (v > mx) mx = v;
        }
        m->maj[node] = mx;
    }
}

static void model_free(Model *m) {
    free(m->sig);
    free(m->maj);
}

/* One case: nhist histories from points uniform in the model's bounding box.
 * mode 0: surface segments, track-length tally per region.
 * mode 1: delta policy: delta segments by rejection against sigma_t(region), surface segments
 *         (cells of a blocked node) by direct sampling; collision tally per region.
 * mode 2: as 1 with every odd node blocked.
 * Prints one line; the Rust reference prints the same text. */
static void run_case(const char *label, const SmrtTracker *t, const Model *m, int mode, int64_t nhist, uint64_t seed) {
    rng_state = seed;
    SmrtCtx *ctx = NULL;
    CHECK(smrt_ctx_new(t, &ctx) == 0);
    double *track = (double *)calloc((size_t)m->nreg, sizeof(double));
    double *coll = (double *)calloc((size_t)m->nreg, sizeof(double));
    uint32_t *block = NULL;
    size_t nblock = 0;
    if (mode == 2) {
        block = (uint32_t *)malloc((size_t)m->nnodes * sizeof(uint32_t));
        for (int64_t i = 1; i < m->nnodes; i += 2) block[nblock++] = (uint32_t)i;
    }
    uint64_t nseg = 0, ncoll = 0, nrej = 0, nskip = 0, kinds[6] = {0, 0, 0, 0, 0, 0};
    const size_t nn = (size_t)m->nnodes;
    for (int64_t h = 0; h < nhist; h++) {
        double p[3], d[3];
        for (int k = 0; k < 3; k++) p[k] = m->info.bounds_lo[k] + (m->info.bounds_hi[k] - m->info.bounds_lo[k]) * unit01();
        double r2;
        for (;;) {
            for (int k = 0; k < 3; k++) d[k] = 2.0 * unit01() - 1.0;
            r2 = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
            if (r2 <= 1.0 && r2 > 1e-4) break;
        }
        double nrm = sqrt(r2);
        for (int k = 0; k < 3; k++) d[k] = d[k] / nrm;
        int32_t rc = smrt_init_state(ctx, p[0], p[1], p[2], d[0], d[1], d[2], -1);
        if (rc == SMRT_NO_HIT) {
            nskip++;
            continue;
        }
        CHECK(rc == 0);
        SmrtState st;
        CHECK(smrt_get_state(ctx, &st) == 0);
        SmrtFlightOpts o = {0.0, 0, 0, mode == 0 ? SMRT_POLICY_SURFACE : SMRT_POLICY_DELTA, SMRT_SCALE_CELL};
        SmrtFlight *f = NULL;
        CHECK(smrt_flight_new(t, &st, &o, &f) == 0);
        int stopped = 0;
        for (;;) {
            SmrtSegment seg;
            rc = smrt_flight_next(f, m->maj, nn, block, nblock, &seg);
            if (rc == 1) break;
            CHECK(rc == 0);
            nseg++;
            double len = seg.t1 - seg.t0;
            int64_t ri = ridx(m, seg.region);
            if (mode == 0) {
                CHECK(seg.mode == SMRT_MODE_SURFACE);
                track[ri] += len;
                continue;
            }
            if (seg.mode == SMRT_MODE_SURFACE) {
                double sg = m->sig[ri], u = unit01(), tau = u / (1.0 - u);
                if (sg > 0.0 && tau / sg < len) {
                    double tt = seg.t0 + tau / sg;
                    if (tt > seg.t1) tt = seg.t1;
                    SmrtLocated loc;
                    CHECK(smrt_flight_check(f, sg) == 0);
                    CHECK(smrt_flight_stop_at(f, m->maj, nn, block, nblock, tt, &loc) == 0);
                    coll[ri] += 1.0 / sg;
                    ncoll++;
                    stopped = 1;
                    break;
                }
                continue;
            }
            double sm = seg.sigma_m, s = 0.0;
            for (;;) {
                double u = unit01(), tau = u / (1.0 - u);
                if (!(sm > 0.0)) break;
                s += tau / sm;
                if (s >= len) break;
                double tt = seg.t0 + s;
                if (tt > seg.t1) tt = seg.t1;
                SmrtLocated loc;
                CHECK(smrt_flight_probe(f, m->maj, nn, block, nblock, tt, &loc) == 0);
                int64_t rj = ridx(m, loc.region);
                double stt = m->sig[rj], u2 = unit01();
                if (u2 * sm < stt) {
                    CHECK(smrt_flight_check(f, stt) == 0);
                    CHECK(smrt_flight_stop_at(f, m->maj, nn, block, nblock, tt, &loc) == 0);
                    coll[rj] += 1.0 / stt;
                    ncoll++;
                    stopped = 1;
                    break;
                }
                nrej++;
            }
            if (stopped) break;
        }
        int32_t kind = -1;
        CHECK(smrt_flight_end(f, &kind, NULL, NULL) == 0);
        CHECK(kind >= 1 && kind <= 5 && (stopped == (kind == SMRT_FLIGHT_STOPPED)));
        kinds[kind]++;
        CHECK(smrt_flight_free(f) == 0);
    }
    printf("CASE %s hist=%lld skip=%llu seg=%llu coll=%llu rej=%llu kinds=%llu,%llu,%llu,%llu,%llu,%llu track=", label, (long long)nhist,
           (unsigned long long)nskip, (unsigned long long)nseg, (unsigned long long)ncoll, (unsigned long long)nrej, (unsigned long long)kinds[0],
           (unsigned long long)kinds[1], (unsigned long long)kinds[2], (unsigned long long)kinds[3], (unsigned long long)kinds[4],
           (unsigned long long)kinds[5]);
    for (int64_t r = 0; r < m->nreg; r++) {
        uint64_t bits;
        memcpy(&bits, &track[r], 8);
        printf("%s%016llx", r ? "," : "", (unsigned long long)bits);
    }
    printf(" colls=");
    for (int64_t r = 0; r < m->nreg; r++) {
        uint64_t bits;
        memcpy(&bits, &coll[r], 8);
        printf("%s%016llx", r ? "," : "", (unsigned long long)bits);
    }
    printf("\n");
    CHECK(smrt_ctx_free(ctx) == 0);
    free(track);
    free(coll);
    free(block);
}

static int fs(const SmrtFlight *f, SmrtState *s) {
    memset(s, 0, sizeof *s);
    return smrt_flight_get_state(f, s);
}

static int same_state(const SmrtState *a, const SmrtState *b) { return memcmp(a, b, sizeof *a) == 0; }

/* Contract violations return typed codes and never crash (acceptance 2 and 4). `start` is a
 * located state inside the model. */
static void misuse_checks(const SmrtTracker *t, const Model *m, const SmrtState *start) {
    const size_t nn = (size_t)m->nnodes;
    SmrtFlightOpts o = {0.0, 0, 0, SMRT_POLICY_DELTA, SMRT_SCALE_CELL};
    SmrtFlight *f = NULL;
    SmrtSegment seg;
    SmrtLocated loc;
    SmrtState before, after;
    char msg[256];

    CHECK(smrt_flight_new(NULL, start, &o, &f) == SMRT_ERR_NULL && f == NULL);
    CHECK(smrt_flight_new(t, NULL, &o, &f) == SMRT_ERR_NULL);
    CHECK(smrt_flight_new(t, start, &o, NULL) == SMRT_ERR_NULL);
    SmrtFlightOpts bad = o;
    bad.policy = 7;
    CHECK(smrt_flight_new(t, start, &bad, &f) == SMRT_ERR_BAD_ARG && f == NULL);
    bad = o;
    bad.scale = -1;
    CHECK(smrt_flight_new(t, start, &bad, &f) == SMRT_ERR_BAD_ARG);
    bad = o;
    bad.max_dist = NAN;
    CHECK(smrt_flight_new(t, start, &bad, &f) == SMRT_ERR_BAD_ARG);
    SmrtState badstate = *start;
    badstate.dir[0] = 3.0;
    CHECK(smrt_flight_new(t, &badstate, &o, &f) == SMRT_ERR_BAD_DIRECTION);
    CHECK(smrt_flight_free(NULL) == SMRT_ERR_NULL);
    CHECK(smrt_flight_check(NULL, 1.0) == SMRT_ERR_NULL);
    CHECK(smrt_flight_get_state(NULL, &before) == SMRT_ERR_NULL);

    CHECK(smrt_flight_new(t, start, &o, &f) == 0 && f != NULL);
    /* before the first segment */
    CHECK(smrt_flight_check(f, 1.0) == SMRT_ERR_INVALID_INPUT);
    CHECK(smrt_flight_probe(f, m->maj, nn, NULL, 0, 0.0, &loc) == SMRT_ERR_INVALID_INPUT);
    /* lengths and null arrays */
    CHECK(smrt_flight_next(f, m->maj, nn + 1, NULL, 0, &seg) == SMRT_ERR_INVALID_INPUT);
    CHECK(smrt_flight_next(f, m->maj, nn - 1, NULL, 0, &seg) == SMRT_ERR_INVALID_INPUT);
    CHECK(smrt_flight_next(f, m->maj, 0, NULL, 0, &seg) == SMRT_ERR_INVALID_INPUT);
    CHECK(smrt_flight_next(f, m->maj, nn, NULL, nn + 1, &seg) == SMRT_ERR_INVALID_INPUT);
    CHECK(smrt_flight_next(f, NULL, nn, NULL, 0, &seg) == SMRT_ERR_NULL);
    CHECK(smrt_flight_next(f, m->maj, nn, NULL, 1, &seg) == SMRT_ERR_NULL);
    CHECK(smrt_flight_next(f, m->maj, nn, NULL, 0, NULL) == SMRT_ERR_NULL);
    CHECK(smrt_flight_next(NULL, m->maj, nn, NULL, 0, &seg) == SMRT_ERR_NULL);
    CHECK(smrt_last_error_message(msg, sizeof msg) > 0);
    /* a bad entry at the segment's node leaves the state unchanged */
    double *bad_maj = (double *)malloc(nn * sizeof(double));
    double bads[3] = {NAN, -1.0, INFINITY};
    for (int i = 0; i < 3; i++) {
        for (size_t k = 0; k < nn; k++) bad_maj[k] = bads[i];
        CHECK(fs(f, &before) == 0);
        CHECK(smrt_flight_next(f, bad_maj, nn, NULL, 0, &seg) == SMRT_ERR_INVALID_MAJORANT);
        CHECK(fs(f, &after) == 0 && same_state(&before, &after));
    }
    free(bad_maj);
    CHECK(smrt_flight_probe(f, m->maj, nn, NULL, 0, 0.0, &loc) == SMRT_ERR_INVALID_INPUT);

    /* a good segment, then the collision check */
    CHECK(smrt_flight_next(f, m->maj, nn, NULL, 0, &seg) == 0);
    CHECK(seg.t1 > seg.t0 && seg.sigma_m > 0.0);
    CHECK(smrt_flight_check(f, 0.0) == 0);
    CHECK(smrt_flight_check(f, seg.sigma_m) == 0);
    CHECK(smrt_flight_check(f, seg.sigma_m * (1.0 + 1e-13)) == 0); /* rounding slack */
    CHECK(fs(f, &before) == 0);
    CHECK(smrt_flight_check(f, seg.sigma_m * 2.0 + 1.0) == SMRT_ERR_MAJORANT);
    CHECK(smrt_last_error_message(msg, sizeof msg) > 0 && strstr(msg, "majorant violated") != NULL && strstr(msg, "node") != NULL);
    CHECK(fs(f, &after) == 0 && same_state(&before, &after));
    CHECK(smrt_flight_check(f, NAN) == SMRT_ERR_NON_FINITE);
    CHECK(smrt_flight_check(f, -1.0) == SMRT_ERR_NON_FINITE);
    CHECK(smrt_flight_check(f, INFINITY) == SMRT_ERR_NON_FINITE);
    /* probe outside the segment, inside it, and nothing moved */
    CHECK(smrt_flight_probe(f, m->maj, nn, NULL, 0, seg.t0 - 1.0, &loc) == SMRT_ERR_INVALID_INPUT);
    CHECK(smrt_flight_probe(f, m->maj, nn, NULL, 0, seg.t1 + 1.0, &loc) == SMRT_ERR_INVALID_INPUT);
    CHECK(smrt_flight_probe(f, m->maj, nn, NULL, 0, NAN, &loc) == SMRT_ERR_INVALID_INPUT);
    CHECK(smrt_flight_probe(f, m->maj, nn, NULL, 0, 0.5 * (seg.t0 + seg.t1), &loc) == 0);
    CHECK(fs(f, &after) == 0 && same_state(&before, &after));
    CHECK(smrt_flight_set_policy(f, 9) == SMRT_ERR_BAD_ARG);
    CHECK(smrt_flight_set_policy(f, SMRT_POLICY_SURFACE) == 0);
    /* the flight goes on after a failed check */
    int32_t rc = smrt_flight_next(f, m->maj, nn, NULL, 0, &seg);
    CHECK(rc == 0 || rc == 1);
    int32_t kind = -1;
    double dist = -1.0;
    SmrtStep ev;
    CHECK(smrt_flight_end(f, &kind, &ev, &dist) == 0 && kind >= 0 && dist > 0.0);
    CHECK(smrt_flight_end(f, NULL, NULL, NULL) == SMRT_ERR_NULL);
    CHECK(smrt_flight_free(f) == 0);

    /* an understated majorant is caught by the check (-12) with the state unchanged */
    SmrtFlight *a = NULL, *b = NULL;
    CHECK(smrt_flight_new(t, start, &o, &a) == 0 && smrt_flight_new(t, start, &o, &b) == 0);
    SmrtSegment sa, sb;
    CHECK(smrt_flight_next(a, m->maj, nn, NULL, 0, &sa) == 0);
    double *low = (double *)malloc(nn * sizeof(double));
    memcpy(low, m->maj, nn * sizeof(double));
    low[sa.node] = 0.5 * m->maj[sa.node];
    CHECK(smrt_flight_input_validate(t, low, nn, NULL, 0) == 0); /* a lower majorant is still a valid array */
    CHECK(smrt_flight_next(b, low, nn, NULL, 0, &sb) == 0 && sb.node == sa.node && sb.sigma_m == 0.5 * sa.sigma_m);
    CHECK(fs(b, &before) == 0);
    CHECK(smrt_flight_check(b, sa.sigma_m) == SMRT_ERR_MAJORANT);
    CHECK(fs(b, &after) == 0 && same_state(&before, &after));
    CHECK(smrt_flight_check(a, sa.sigma_m) == 0);
    free(low);
    CHECK(smrt_flight_free(a) == 0 && smrt_flight_free(b) == 0);

    /* one-time validation */
    uint32_t good_block[2] = {0, (uint32_t)(nn - 1)};
    uint32_t unsorted[2] = {2, 1}, dup[2] = {1, 1}, outside[1] = {(uint32_t)nn};
    CHECK(smrt_flight_input_validate(t, m->maj, nn, good_block, nn > 1 ? 2 : 1) == 0);
    CHECK(smrt_flight_input_validate(t, m->maj, nn, unsorted, 2) == SMRT_ERR_INVALID_INPUT);
    CHECK(smrt_flight_input_validate(t, m->maj, nn, dup, 2) == SMRT_ERR_INVALID_INPUT);
    CHECK(smrt_flight_input_validate(t, m->maj, nn, outside, 1) == SMRT_ERR_INVALID_INPUT);
    CHECK(smrt_flight_input_validate(t, m->maj, nn + 1, NULL, 0) == SMRT_ERR_INVALID_INPUT);
    CHECK(smrt_flight_input_validate(t, NULL, nn, NULL, 0) == SMRT_ERR_NULL);
    CHECK(smrt_flight_input_validate(NULL, m->maj, nn, NULL, 0) == SMRT_ERR_NULL);
    double *nanm = (double *)malloc(nn * sizeof(double));
    memcpy(nanm, m->maj, nn * sizeof(double));
    nanm[nn - 1] = NAN;
    CHECK(smrt_flight_input_validate(t, nanm, nn, NULL, 0) == SMRT_ERR_INVALID_MAJORANT);
    CHECK(strstr((smrt_last_error_message(msg, sizeof msg), msg), "node") != NULL);
    free(nanm);

    /* node and set queries: bad ids and null outputs */
    int64_t v = 0;
    SmrtNodeInfo ni;
    CHECK(smrt_node_info(t, -1, &ni) == SMRT_ERR_UNKNOWN_ID);
    CHECK(smrt_node_info(t, m->nnodes, &ni) == SMRT_ERR_UNKNOWN_ID);
    CHECK(smrt_node_set(t, m->nnodes, &v) == SMRT_ERR_UNKNOWN_ID);
    CHECK(smrt_node_info(t, 0, NULL) == SMRT_ERR_NULL);
    CHECK(smrt_n_nodes(NULL, &v) == SMRT_ERR_NULL);
    CHECK(smrt_n_sets(t, NULL) == SMRT_ERR_NULL);
    CHECK(smrt_set_regions(t, 1000000, NULL, 0, &v) == SMRT_ERR_UNKNOWN_ID);
    CHECK(smrt_set_regions(t, 0, NULL, 4, &v) == SMRT_ERR_NULL);
    CHECK(smrt_set_regions(t, 0, NULL, -1, &v) == SMRT_ERR_BAD_ARG);
    CHECK(smrt_set_regions(t, 0, NULL, 0, &v) == 0 && v >= 1);
    int roots = 0; /* exactly one node has no parent */
    for (int64_t k = 0; k < m->nnodes; k++) {
        CHECK(smrt_node_info(t, k, &ni) == 0 && ni.parent >= SMRT_NONE && ni.parent < m->nnodes);
        CHECK(ni.bounds_lo[0] <= ni.bounds_hi[0] && ni.kind >= 0 && ni.kind <= 4);
        roots += ni.parent == SMRT_NONE;
    }
    CHECK(roots == 1);
    char name[8];
    CHECK(smrt_region_name(t, m->nreg, name, sizeof name) == SMRT_ERR_UNKNOWN_ID);
    CHECK(smrt_region_name(NULL, 0, name, sizeof name) == SMRT_ERR_NULL);
    CHECK(smrt_region_name(t, 0, NULL, 4) == SMRT_ERR_NULL);
    CHECK(smrt_region_name(t, 0, name, -1) == SMRT_ERR_BAD_ARG);
    int32_t full = smrt_region_name(t, 0, NULL, 0);
    CHECK(full >= 0);
    int32_t again = smrt_region_name(t, 0, name, 2); /* truncated, still NUL-terminated */
    CHECK(again == full && strlen(name) == (size_t)(full >= 1 ? 1 : 0));
}
#endif
