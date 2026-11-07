/* W3b milestone C driver (plan section 5.1): a tiny transport "code" that opens the tracking
 * image `session.export milestone.smrt` wrote, through libsmrt only (no sm-* Rust crate).
 *
 * The code owns its cross sections: it reads the region names and the node sets from the file,
 * assigns sigma per NAME (smart-mesh and sm-rt never see a cross section) and passes one majorant
 * per node to every flight call. It then checks what fits a pin-cell lattice:
 *   i.   1e4 seeded lines: the segment lengths of the surface flight and of the delta flight both
 *        sum to the chord of the model's bounding box (tolerance 1e-12 x (1 + chord), widened for
 *        grazing lines by 0.05 / min|d_k| as in rulings R1 and R7);
 *   ii.  an absorption-only source: the surface (track-length) and the delta (collision) flux
 *        estimates per region agree within 4 sigma, with fixed seeds;
 *   iii. an understated majorant for one node is caught by smrt_flight_check with -12 and the
 *        flight state unchanged.
 *
 * usage: w3b_driver <milestone.smrt>
 * Output: `ROW <id> PASS|FAIL <detail>` lines and informational lines; exit 0 iff every row passes. */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "smrt.h"

static int failures = 0;
static void row(const char *id, int ok, const char *fmt, ...) __attribute__((format(printf, 3, 4)));
#include <stdarg.h>
static void row(const char *id, int ok, const char *fmt, ...) {
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    printf("ROW %s %s %s\n", id, ok ? "PASS" : "FAIL", buf);
    if (!ok) failures++;
}

#define MUST(cond)                                                              \
    do {                                                                        \
        if (!(cond)) {                                                          \
            char m_[256] = "";                                                  \
            smrt_last_error_message(m_, sizeof m_);                             \
            fprintf(stderr, "%s:%d: %s (%s)\n", __FILE__, __LINE__, #cond, m_); \
            printf("ROW fatal FAIL %s:%d %s\n", __FILE__, __LINE__, #cond);     \
            exit(1);                                                            \
        }                                                                       \
    } while (0)

/* ---- the code's own cross sections, by material name ---------------------------------------
 * session.export names each region after its block (material group, `mat:<name>` in .h5m); the
 * code decides what that material is: fuel and water here. smart-mesh and sm-rt never see a
 * cross section. A name the code does not know (the complement) gets 1.0 and is not counted. */
static int known_material(const char *name) { return strcmp(name, "fuel") == 0 || strcmp(name, "water") == 0; }
static double sigma_of(const char *name) {
    if (strcmp(name, "fuel") == 0) return 1.5;
    if (strcmp(name, "water") == 0) return 0.5;
    return 1.0;
}

/* ---- deterministic randomness ------------------------------------------------------------- */
static uint64_t rng_state;
static uint64_t next_u64(void) {
    uint64_t z = (rng_state += 0x9E3779B97F4A7C15ull);
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    return z ^ (z >> 31);
}
static double unit01(void) { return (double)(next_u64() >> 11) / 9007199254740992.0; }
/* exponential variate with unit mean */
static double expo(void) { return -log(1.0 - unit01()); }

typedef struct {
    int64_t nreg, comp, nnodes, nsets;
    double *sig;  /* per region */
    double *maj;  /* per node: max sigma over the regions of the node's set */
    char **names; /* per region */
    SmrtInfo info;
} Model;

static int64_t ridx(const Model *m, int64_t r) {
    if (r == SMRT_COMPLEMENT) return m->comp;
    MUST(r >= 0 && r < m->nreg);
    return r;
}

static void model_load(const SmrtTracker *t, Model *m) {
    MUST(smrt_info(t, &m->info) == 0);
    m->nreg = m->info.regions;
    m->comp = m->info.complement_region;
    m->sig = (double *)calloc((size_t)m->nreg, sizeof(double));
    m->names = (char **)calloc((size_t)m->nreg, sizeof(char *));
    MUST(m->sig && m->names);
    for (int64_t r = 0; r < m->nreg; r++) {
        char buf[256];
        int32_t n = smrt_region_name(t, r, buf, sizeof buf);
        MUST(n >= 0 && n < (int32_t)sizeof buf);
        m->names[r] = strdup(buf);
        m->sig[r] = sigma_of(buf);
    }
    MUST(smrt_n_nodes(t, &m->nnodes) == 0 && m->nnodes > 0);
    MUST(smrt_n_sets(t, &m->nsets) == 0 && m->nsets > 0);
    m->maj = (double *)calloc((size_t)m->nnodes, sizeof(double));
    MUST(m->maj != NULL);
    for (int64_t node = 0; node < m->nnodes; node++) {
        int64_t set = -1, n = 0;
        MUST(smrt_node_set(t, node, &set) == 0 && set >= 0 && set < m->nsets);
        MUST(smrt_set_regions(t, set, NULL, 0, &n) == 0 && n >= 1);
        int64_t *ids = (int64_t *)malloc((size_t)n * sizeof(int64_t));
        int64_t got = 0;
        MUST(ids && smrt_set_regions(t, set, ids, n, &got) == 0 && got == n);
        double mx = 0.0;
        for (int64_t i = 0; i < n; i++) {
            double v = m->sig[ridx(m, ids[i])];
            if (v > mx) mx = v;
        }
        free(ids);
        m->maj[node] = mx;
    }
}

/* A start inside the model: uniform in the bounding box, isotropic. Returns 0 when located. */
static int start(SmrtCtx *ctx, const Model *m, double p[3], double d[3]) {
    for (int k = 0; k < 3; k++) p[k] = m->info.bounds_lo[k] + (m->info.bounds_hi[k] - m->info.bounds_lo[k]) * unit01();
    double r2;
    for (;;) {
        for (int k = 0; k < 3; k++) d[k] = 2.0 * unit01() - 1.0;
        r2 = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
        if (r2 <= 1.0 && r2 > 1e-4) break;
    }
    double nrm = sqrt(r2);
    for (int k = 0; k < 3; k++) d[k] /= nrm;
    int32_t rc = smrt_init_state(ctx, p[0], p[1], p[2], d[0], d[1], d[2], -1);
    MUST(rc == 0 || rc == SMRT_NO_HIT);
    return rc;
}

/* Distance from p along d to the exit of the bounding box. */
static double box_chord(const Model *m, const double p[3], const double d[3], double *min_cos) {
    double chord = INFINITY, mc = INFINITY;
    for (int k = 0; k < 3; k++) {
        if (d[k] == 0.0) continue;
        double t = ((d[k] > 0.0 ? m->info.bounds_hi[k] : m->info.bounds_lo[k]) - p[k]) / d[k];
        if (t < chord) chord = t;
        if (fabs(d[k]) < mc) mc = fabs(d[k]);
    }
    *min_cos = mc;
    return chord;
}

/* Sum of the segment lengths of one flight run to its end. */
static double flight_sum(const SmrtTracker *t, const Model *m, const SmrtState *st, int64_t policy, int32_t *kind, uint64_t *nseg, uint64_t *ndelta) {
    SmrtFlightOpts o = {0.0, 0, 0, policy, SMRT_SCALE_CELL};
    SmrtFlight *f = NULL;
    MUST(smrt_flight_new(t, st, &o, &f) == 0);
    double sum = 0.0;
    for (;;) {
        SmrtSegment seg;
        int32_t rc = smrt_flight_next(f, m->maj, (size_t)m->nnodes, NULL, 0, &seg);
        if (rc == 1) break;
        MUST(rc == 0);
        sum += seg.t1 - seg.t0;
        (*nseg)++;
        if (seg.mode == SMRT_MODE_DELTA) (*ndelta)++;
    }
    MUST(smrt_flight_end(f, kind, NULL, NULL) == 0);
    MUST(smrt_flight_free(f) == 0);
    return sum;
}

/* ---- (i) chord sums ------------------------------------------------------------------------ */
static void check_chords(const SmrtTracker *t, const Model *m) {
    SmrtCtx *ctx = NULL;
    MUST(smrt_ctx_new(t, &ctx) == 0);
    rng_state = 0xC40DD5ull;
    const int nlines = 10000;
    int ok_s = 0, ok_d = 0, used = 0, skipped = 0, bad_end = 0;
    double worst_s = 0.0, worst_d = 0.0, worst_ratio = 0.0;
    uint64_t nseg_s = 0, nseg_d = 0, ndelta = 0;
    for (int i = 0; i < nlines; i++) {
        double p[3], d[3], mc;
        if (start(ctx, m, p, d) != 0) {
            skipped++;
            continue;
        }
        SmrtState st;
        MUST(smrt_get_state(ctx, &st) == 0);
        double chord = box_chord(m, p, d, &mc);
        double tol = 1e-12 * (1.0 + chord) * fmax(1.0, 0.05 / mc);
        int32_t ks = -1, kd = -1;
        double ss = flight_sum(t, m, &st, SMRT_POLICY_SURFACE, &ks, &nseg_s, &ndelta);
        double sd = flight_sum(t, m, &st, SMRT_POLICY_DELTA, &kd, &nseg_d, &ndelta);
        used++;
        double es = fabs(ss - chord), ed = fabs(sd - chord);
        if (es <= tol) ok_s++;
        if (ed <= tol) ok_d++;
        if (es > worst_s) worst_s = es;
        if (ed > worst_d) worst_d = ed;
        if (es / tol > worst_ratio) worst_ratio = es / tol;
        if (ed / tol > worst_ratio) worst_ratio = ed / tol;
        if (ks == SMRT_FLIGHT_LOST || kd == SMRT_FLIGHT_LOST) bad_end++;
    }
    printf("INFO chords lines=%d located=%d skipped=%d segments_surface=%llu segments_delta=%llu delta_segments=%llu worst_abs_surface=%.3e worst_abs_delta=%.3e worst_over_tol=%.3f\n",
           nlines, used, skipped, (unsigned long long)nseg_s, (unsigned long long)nseg_d, (unsigned long long)ndelta, worst_s, worst_d, worst_ratio);
    row("i.surface", used == nlines && ok_s == used, "%d/%d lines sum to the box chord (worst %.3e)", ok_s, used, worst_s);
    row("i.delta", used == nlines && ok_d == used && ndelta > 0, "%d/%d lines sum to the box chord (worst %.3e, %llu delta segments)", ok_d, used, worst_d, (unsigned long long)ndelta);
    row("i.no_lost", bad_end == 0, "%d flights ended Lost", bad_end);
    MUST(smrt_ctx_free(ctx) == 0);
}

/* ---- (ii) surface vs delta flux ------------------------------------------------------------- */
/* Per-history tallies: sum and sum of squares over histories, per region. */
typedef struct {
    double *s1, *s2;
    int64_t nhist;
} Tally;

static void tally_new(Tally *t, int64_t nreg) {
    t->s1 = (double *)calloc((size_t)nreg, sizeof(double));
    t->s2 = (double *)calloc((size_t)nreg, sizeof(double));
    t->nhist = 0;
    MUST(t->s1 && t->s2);
}

static void tally_add(Tally *t, const double *x, int64_t nreg) {
    for (int64_t r = 0; r < nreg; r++) {
        t->s1[r] += x[r];
        t->s2[r] += x[r] * x[r];
    }
    t->nhist++;
}

/* mean and standard error of the mean of the sum of the regions in `sel` (NULL = region r only) */
static void tally_stat(const Tally *t, int64_t r, double *mean, double *se) {
    double n = (double)t->nhist, m = t->s1[r] / n;
    double var = (t->s2[r] / n - m * m) * n / (n - 1.0);
    *mean = m;
    *se = sqrt(fmax(var, 0.0) / n);
}

/* Absorption-only particles: implicit capture with weights along surface segments (track length). */
static void run_surface(const SmrtTracker *t, const Model *m, int64_t nhist, uint64_t seed, Tally *out) {
    rng_state = seed;
    SmrtCtx *ctx = NULL;
    MUST(smrt_ctx_new(t, &ctx) == 0);
    double *x = (double *)calloc((size_t)m->nreg, sizeof(double));
    MUST(x != NULL);
    for (int64_t h = 0; h < nhist; h++) {
        double p[3], d[3];
        int rc = start(ctx, m, p, d);
        memset(x, 0, (size_t)m->nreg * sizeof(double));
        if (rc == 0) {
            SmrtState st;
            MUST(smrt_get_state(ctx, &st) == 0);
            SmrtFlightOpts o = {0.0, 0, 0, SMRT_POLICY_SURFACE, SMRT_SCALE_CELL};
            SmrtFlight *f = NULL;
            MUST(smrt_flight_new(t, &st, &o, &f) == 0);
            double w = 1.0;
            for (;;) {
                SmrtSegment seg;
                int32_t r2 = smrt_flight_next(f, m->maj, (size_t)m->nnodes, NULL, 0, &seg);
                if (r2 == 1) break;
                MUST(r2 == 0 && seg.mode == SMRT_MODE_SURFACE);
                double len = seg.t1 - seg.t0, sg = m->sig[ridx(m, seg.region)];
                double surv = exp(-sg * len);
                x[ridx(m, seg.region)] += w * (sg > 0.0 ? (1.0 - surv) / sg : len);
                w *= surv;
            }
            MUST(smrt_flight_free(f) == 0);
        }
        tally_add(out, x, m->nreg);
    }
    free(x);
    MUST(smrt_ctx_free(ctx) == 0);
}

/* The same particles by delta tracking: collisions sampled with the node majorant, collision
 * estimator 1 / sigma_t(region) at each real collision; the particle is absorbed there. */
static void run_delta(const SmrtTracker *t, const Model *m, int64_t nhist, uint64_t seed, Tally *out, uint64_t *ncoll, uint64_t *nrej) {
    rng_state = seed;
    SmrtCtx *ctx = NULL;
    MUST(smrt_ctx_new(t, &ctx) == 0);
    double *x = (double *)calloc((size_t)m->nreg, sizeof(double));
    MUST(x != NULL);
    const size_t nn = (size_t)m->nnodes;
    for (int64_t h = 0; h < nhist; h++) {
        double p[3], d[3];
        int rc = start(ctx, m, p, d);
        memset(x, 0, (size_t)m->nreg * sizeof(double));
        if (rc == 0) {
            SmrtState st;
            MUST(smrt_get_state(ctx, &st) == 0);
            SmrtFlightOpts o = {0.0, 0, 0, SMRT_POLICY_DELTA, SMRT_SCALE_CELL};
            SmrtFlight *f = NULL;
            MUST(smrt_flight_new(t, &st, &o, &f) == 0);
            int stopped = 0;
            for (;;) {
                SmrtSegment seg;
                int32_t r2 = smrt_flight_next(f, m->maj, nn, NULL, 0, &seg);
                if (r2 == 1) break;
                MUST(r2 == 0);
                double len = seg.t1 - seg.t0;
                SmrtLocated loc;
                if (seg.mode == SMRT_MODE_SURFACE) {
                    /* the whole segment lies in one cell: sample it directly */
                    int64_t ri = ridx(m, seg.region);
                    double sg = m->sig[ri], tau = expo();
                    if (sg > 0.0 && tau / sg < len) {
                        double tt = seg.t0 + tau / sg;
                        if (tt > seg.t1) tt = seg.t1;
                        MUST(smrt_flight_check(f, sg) == 0);
                        MUST(smrt_flight_stop_at(f, m->maj, nn, NULL, 0, tt, &loc) == 0);
                        x[ri] += 1.0 / sg;
                        (*ncoll)++;
                        stopped = 1;
                        break;
                    }
                    continue;
                }
                double sm = seg.sigma_m, s = 0.0;
                if (!(sm > 0.0)) continue;
                for (;;) {
                    s += expo() / sm;
                    if (s >= len) break;
                    double tt = seg.t0 + s;
                    if (tt > seg.t1) tt = seg.t1;
                    MUST(smrt_flight_probe(f, m->maj, nn, NULL, 0, tt, &loc) == 0);
                    int64_t rj = ridx(m, loc.region);
                    double stt = m->sig[rj];
                    if (unit01() * sm < stt) {
                        MUST(smrt_flight_check(f, stt) == 0);
                        MUST(smrt_flight_stop_at(f, m->maj, nn, NULL, 0, tt, &loc) == 0);
                        x[rj] += 1.0 / stt;
                        (*ncoll)++;
                        stopped = 1;
                        break;
                    }
                    (*nrej)++;
                }
                if (stopped) break;
            }
            MUST(smrt_flight_free(f) == 0);
        }
        tally_add(out, x, m->nreg);
    }
    free(x);
    MUST(smrt_ctx_free(ctx) == 0);
}

static void check_flux(const SmrtTracker *t, const Model *m) {
    const int64_t nhist = 40000;
    Tally ts, td;
    tally_new(&ts, m->nreg);
    tally_new(&td, m->nreg);
    uint64_t ncoll = 0, nrej = 0;
    run_surface(t, m, nhist, 0x5EED0001ull, &ts);
    run_delta(t, m, nhist, 0x5EED0002ull, &td, &ncoll, &nrej);
    printf("INFO flux histories=%lld collisions=%llu rejected=%llu\n", (long long)nhist, (unsigned long long)ncoll, (unsigned long long)nrej);
    int checked = 0, within = 0;
    double worst = 0.0;
    for (int64_t r = 0; r < m->nreg; r++) {
        if (r == m->comp || !(m->sig[r] > 0.0)) continue;
        double ms, ses, md, sed;
        tally_stat(&ts, r, &ms, &ses);
        tally_stat(&td, r, &md, &sed);
        double se = sqrt(ses * ses + sed * sed);
        double z = se > 0.0 ? fabs(ms - md) / se : (ms == md ? 0.0 : INFINITY);
        printf("FLUX region=%lld name=%s surface=%.6e delta=%.6e se=%.3e z=%.2f\n", (long long)r, m->names[r], ms, md, se, z);
        checked++;
        if (z <= 4.0) within++;
        if (z > worst) worst = z;
    }
    row("ii.flux_per_region", checked > 0 && within == checked, "%d/%d regions within 4 sigma (worst z %.2f)", within, checked, worst);
}

/* ---- (iii) understated majorant ------------------------------------------------------------- */
static int same_state(const SmrtState *a, const SmrtState *b) { return memcmp(a, b, sizeof *a) == 0; }

static void check_understated(const SmrtTracker *t, const Model *m) {
    SmrtCtx *ctx = NULL;
    MUST(smrt_ctx_new(t, &ctx) == 0);
    rng_state = 0x0DD5EEDull;
    double p[3], d[3];
    while (start(ctx, m, p, d) != 0) {}
    SmrtState st;
    MUST(smrt_get_state(ctx, &st) == 0);
    SmrtFlightOpts o = {0.0, 0, 0, SMRT_POLICY_DELTA, SMRT_SCALE_CELL};
    const size_t nn = (size_t)m->nnodes;
    SmrtFlight *a = NULL, *b = NULL;
    MUST(smrt_flight_new(t, &st, &o, &a) == 0 && smrt_flight_new(t, &st, &o, &b) == 0);
    /* find a first segment with a positive majorant (the start may be in the complement) */
    SmrtSegment sa, sb;
    MUST(smrt_flight_next(a, m->maj, nn, NULL, 0, &sa) == 0);
    double sigma_true = sa.sigma_m;
    double *low = (double *)malloc(nn * sizeof(double));
    MUST(low != NULL);
    memcpy(low, m->maj, nn * sizeof(double));
    low[sa.node] = 0.5 * m->maj[sa.node];
    MUST(smrt_flight_next(b, low, nn, NULL, 0, &sb) == 0 && sb.node == sa.node);
    SmrtState before, after;
    memset(&before, 0, sizeof before);
    memset(&after, 0, sizeof after);
    MUST(smrt_flight_get_state(b, &before) == 0);
    int32_t rc_low = smrt_flight_check(b, sigma_true);
    MUST(smrt_flight_get_state(b, &after) == 0);
    char msg[256] = "";
    smrt_last_error_message(msg, sizeof msg);
    int32_t rc_ok = smrt_flight_check(a, sigma_true);
    row("iii.understated", sigma_true > 0.0 && rc_low == SMRT_ERR_MAJORANT && same_state(&before, &after) && rc_ok == 0,
        "node %lld majorant %.3f understated to %.3f: check %d, correct majorant %d, state %s (%s)", (long long)sa.node, sigma_true, low[sa.node],
        rc_low, rc_ok, same_state(&before, &after) ? "unchanged" : "CHANGED", msg);
    free(low);
    MUST(smrt_flight_free(a) == 0 && smrt_flight_free(b) == 0);
    MUST(smrt_ctx_free(ctx) == 0);
}

int main(int argc, char **argv) {
    if (argc != 2) {
        fprintf(stderr, "usage: %s <milestone.smrt>\n", argv[0]);
        return 2;
    }
    SmrtTracker *t = NULL;
    MUST(smrt_open(argv[1], &t) == 0 && t != NULL);
    printf("INFO abi %d\n", smrt_version());
    Model m;
    model_load(t, &m);

    /* node sets and region names reach the code; its own sigma per name */
    int64_t nvol = 0, hasfuel = 0, haswater = 0;
    for (int64_t r = 0; r < m.nreg; r++) {
        printf("REGION %lld %s sigma=%g\n", (long long)r, m.names[r], m.sig[r]);
        if (known_material(m.names[r])) nvol++;
        if (strcmp(m.names[r], "fuel") == 0) hasfuel++;
        if (strcmp(m.names[r], "water") == 0) haswater++;
    }
    printf("INFO nodes=%lld sets=%lld regions=%lld complement=%lld cells=%lld\n", (long long)m.nnodes, (long long)m.nsets, (long long)m.nreg,
           (long long)m.comp, (long long)m.info.cells);
    int64_t mixed = 0; /* sets that hold regions of more than one sigma (the delta-tracking case) */
    for (int64_t s = 0; s < m.nsets; s++) {
        int64_t n = 0;
        MUST(smrt_set_regions(t, s, NULL, 0, &n) == 0);
        int64_t *ids = (int64_t *)malloc((size_t)n * sizeof(int64_t));
        int64_t got = 0;
        MUST(ids && smrt_set_regions(t, s, ids, n, &got) == 0);
        for (int64_t i = 1; i < n; i++)
            if (m.sig[ridx(&m, ids[i])] != m.sig[ridx(&m, ids[0])]) {
                mixed++;
                break;
            }
        free(ids);
    }
    row("names", hasfuel == 1 && haswater == 1 && nvol == 2 && nvol <= m.nreg && m.nreg >= 2 && m.nnodes > 1 && m.nsets >= 1 && mixed >= 1,
        "%lld material regions (fuel, water) by name, %lld nodes, %lld sets (%lld with mixed sigma)", (long long)nvol, (long long)m.nnodes, (long long)m.nsets,
        (long long)mixed);

    check_chords(t, &m);
    check_flux(t, &m);
    check_understated(t, &m);

    MUST(smrt_tracker_free(t) == 0);
    printf("w3b_driver: %s (%d failing rows)\n", failures ? "FAIL" : "ok", failures);
    return failures ? 1 : 0;
}
