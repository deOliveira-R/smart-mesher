/* smrt.h: C ABI of libsmrt (crate sm-rt-capi): ray tracing / particle tracking over any
 * sm-rt-api tracker, plus DAGMC-shaped shims.
 *
 * Conventions
 *  - Every function returns int32_t: 0 ok, 1 "no hit" (not an error), negative = error
 *    (SMRT_ERR_*). The message of the last error on the calling thread is available from
 *    smrt_last_error_message. No panic crosses this boundary.
 *  - Ids are int64_t: >= 0 dense id, SMRT_NONE (-1) none / outside the model, SMRT_COMPLEMENT (-2)
 *    the implicit complement. Outputs always spell the implicit-complement region SMRT_COMPLEMENT;
 *    inputs accept it or the region's raw id (SmrtInfo.complement_region).
 *  - A SmrtTracker is shared and read-only: any number of threads may use it at once. A SmrtCtx
 *    (particle state + scratch) is used by ONE thread at a time (not locked). A ctx keeps its
 *    tracker alive, so handles may be freed in any order.
 *  - This file uses only int32_t, int64_t, uint64_t, double, char and opaque pointers.
 */
#ifndef SMRT_H
#define SMRT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- status codes ------------------------------------------------------------------------- */
#define SMRT_OK 0
#define SMRT_NO_HIT 1
#define SMRT_ERR_BUILD (-1)
#define SMRT_ERR_BAD_DIRECTION (-2)
#define SMRT_ERR_NON_FINITE (-3)
#define SMRT_ERR_UNKNOWN_ID (-4)
#define SMRT_ERR_UNSUPPORTED (-5)
#define SMRT_ERR_CACHE_STALE (-6)
#define SMRT_ERR_CACHE_CORRUPT (-7)
#define SMRT_ERR_GEOM (-8)
#define SMRT_ERR_BAD_ALBEDO (-9)
#define SMRT_ERR_BIN_COUNT (-10)
#define SMRT_ERR_INVALID_INPUT (-11)
#define SMRT_ERR_NULL (-100)
#define SMRT_ERR_PANIC (-101)
#define SMRT_ERR_BAD_ARG (-102)
#define SMRT_ERR_LOST (-103)

/* ---- ids and enums ------------------------------------------------------------------------ */
#define SMRT_ABI_VERSION 10000 /* major*10000 + minor*100 + patch */
#define SMRT_NONE (-1)
#define SMRT_COMPLEMENT (-2)
#define SMRT_MAX_LEVELS 8

#define SMRT_END_LIMIT 0
#define SMRT_END_CELL_FACE 1
#define SMRT_END_SLOT_CHANGE 2
#define SMRT_END_REGION_BOUNDARY 3
#define SMRT_END_BOUNDARY 4
#define SMRT_END_LOST 5

#define SMRT_ACT_NONE 0
#define SMRT_ACT_VACUUM 1
#define SMRT_ACT_WHITE 2
#define SMRT_ACT_ALBEDO 3
#define SMRT_ACT_REFLECTIVE 4
#define SMRT_ACT_MIRROR 10
#define SMRT_ACT_WRAP 11

#define SMRT_RESP_VACUUM 1
#define SMRT_RESP_WHITE 2
#define SMRT_RESP_ALBEDO 3
#define SMRT_RESP_REFLECTIVE 4

#define SMRT_ORIENT_EXITING 1
#define SMRT_ORIENT_ENTERING (-1)
#define SMRT_ORIENT_ANY 0

#define SMRT_LOST_NONE 0

/* ---- handles ------------------------------------------------------------------------------ */
typedef struct SmrtTracker SmrtTracker; /* opaque, shared, read-only */
typedef struct SmrtCtx SmrtCtx;         /* opaque, one thread at a time */

/* ---- data --------------------------------------------------------------------------------- */
typedef struct SmrtLevel {
    int64_t component;
    int64_t instance;
    int32_t lattice[3];
    int32_t pad;
} SmrtLevel;

/* levels_total is the true depth; only the first min(levels_total, 8) entries are present. */
typedef struct SmrtState {
    double pos[3];
    double dir[3];
    int64_t cell;
    int64_t region;
    int64_t entered_via;
    int32_t levels_total;
    int32_t levels_len;
    SmrtLevel levels[SMRT_MAX_LEVELS];
} SmrtState;

typedef struct SmrtStep {
    double dist;
    int64_t cell;
    int64_t region;
    int32_t end;    /* SMRT_END_* */
    int32_t action; /* SMRT_ACT_* */
    int64_t face;
    int64_t surface;
    int64_t to_region;
    int64_t to_cell;
    double albedo;
    int32_t lost_cause; /* 1 no cell, 2 no exit face, 3 bad entry face, 4 non-finite, 5 step cap, 6 recovery */
    int32_t pad;
    int64_t lost_cell;
    double lost_pos[3];
    double lost_dir[3];
} SmrtStep;

typedef struct SmrtLocated {
    int64_t cell;
    int64_t region;
    int64_t on_face;
    int32_t levels_total;
    int32_t levels_len;
    SmrtLevel levels[SMRT_MAX_LEVELS];
} SmrtLocated;

typedef struct SmrtClosest {
    double dist;
    int64_t surface;
    double point[3];
} SmrtClosest;

typedef struct SmrtInfo {
    int64_t cells;
    int64_t faces;
    int64_t regions;
    int64_t surfaces;
    int64_t prototypes;
    int64_t copies;
    int64_t memory_bytes;
    int64_t exact_faces;
    int64_t faceted_faces;
    int64_t caps;
    int64_t complement_region;
    int64_t max_events;
    uint64_t structure_hash_lo;
    uint64_t structure_hash_hi;
    double bounds_lo[3];
    double bounds_hi[3];
    double tol_linear;
    double tol_angular;
} SmrtInfo;

typedef struct SmrtRegionInfo {
    int64_t original_id;
    int64_t color_id; /* reserved: always -1 */
    double measure;   /* volume; NaN when unknown */
    int64_t implicit_complement;
} SmrtRegionInfo;

typedef struct SmrtTraceOpts {
    double max_dist;     /* <= 0 or infinite: unlimited */
    int64_t max_steps;   /* <= 0: tracker default */
    int64_t stop_at_deck;
} SmrtTraceOpts;

typedef struct SmrtRays {
    const double *x, *y, *z; /* origins */
    const double *u, *v, *w; /* unit directions */
    const int64_t *start_cell; /* may be NULL; ignored by the contract's trace_bundle */
} SmrtRays;

/* ---- handles and construction ------------------------------------------------------------- */
int32_t smrt_open(const char *path, SmrtTracker **out);
int32_t smrt_tracker_free(SmrtTracker *tracker);
int32_t smrt_ctx_new(const SmrtTracker *tracker, SmrtCtx **out);
int32_t smrt_ctx_free(SmrtCtx *ctx);
int32_t smrt_info(const SmrtTracker *tracker, SmrtInfo *out);
int32_t smrt_region_info(const SmrtTracker *tracker, int64_t region, SmrtRegionInfo *out);
int32_t smrt_version(void);
int32_t smrt_last_error_message(char *buf, int64_t cap);

/* ---- tracking ----------------------------------------------------------------------------- */
int32_t smrt_init_state(SmrtCtx *ctx, double x, double y, double z, double u, double v, double w, int64_t hint_cell);
int32_t smrt_get_state(const SmrtCtx *ctx, SmrtState *out);
int32_t smrt_set_state(SmrtCtx *ctx, const SmrtState *state);
int32_t smrt_set_direction(SmrtCtx *ctx, double u, double v, double w);
int32_t smrt_step(SmrtCtx *ctx, double max_dist, SmrtStep *out);
int32_t smrt_peek(SmrtCtx *ctx, double limit, SmrtStep *out);
int32_t smrt_fire(SmrtCtx *ctx, double dist_limit, int64_t max_steps, SmrtStep *out);
int32_t smrt_resolve_boundary(SmrtCtx *ctx, int64_t face, int32_t kind, double albedo, double u0, double u1, double out_dir[3]);
int32_t smrt_locate(SmrtCtx *ctx, const double p[3], const double *dir_or_null, int64_t hint, SmrtLocated *out);
int32_t smrt_closest(SmrtCtx *ctx, const double p[3], int64_t region, SmrtClosest *out);
int32_t smrt_trace_bundle(const SmrtTracker *tracker, int64_t n, const SmrtRays *rays, int64_t block_size,
                          const SmrtTraceOpts *opts, uint64_t *out_buf, uint64_t cap, uint64_t *out_len);

/* ---- DAGMC-shaped shims ------------------------------------------------------------------- */
int32_t smrt_dag_ray_fire(const SmrtTracker *tracker, SmrtCtx *ctx, int64_t region, const double origin[3],
                          const double dir[3], int32_t orientation, double dist_limit, int64_t *out_surface,
                          double *out_dist, int64_t *out_face);
int32_t smrt_dag_point_in_volume(const SmrtTracker *tracker, SmrtCtx *ctx, int64_t region, const double p[3],
                                 const double *dir_or_null, int32_t *out_inside);
int32_t smrt_dag_next_vol(const SmrtTracker *tracker, int64_t surface, int64_t old_region, int64_t *out_new_region);
int32_t smrt_dag_surface_sense(const SmrtTracker *tracker, int64_t surface, int64_t region, int32_t *out_sense);
int32_t smrt_dag_get_angle(const SmrtTracker *tracker, int64_t face, const double p[3], double out_normal[3]);
int32_t smrt_dag_measure_volume(const SmrtTracker *tracker, int64_t region, double *out_volume);
int32_t smrt_dag_measure_area(const SmrtTracker *tracker, int64_t surface, double *out_area);

#ifdef __cplusplus
}
#endif

#endif /* SMRT_H */
