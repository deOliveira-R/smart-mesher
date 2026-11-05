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
 *  - This file uses only int32_t, int64_t, uint32_t, uint64_t, size_t, double, char and opaque pointers.
 */
#ifndef SMRT_H
#define SMRT_H

#include <stddef.h>
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
#define SMRT_ERR_MAJORANT (-12)
#define SMRT_ERR_INVALID_MAJORANT (-13)
#define SMRT_ERR_NULL (-100)
#define SMRT_ERR_PANIC (-101)
#define SMRT_ERR_BAD_ARG (-102)
#define SMRT_ERR_LOST (-103)

/* ---- ids and enums ------------------------------------------------------------------------ */
#define SMRT_ABI_VERSION 10100 /* major*10000 + minor*100 + patch */
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
#define SMRT_LOST_NO_CELL 1
#define SMRT_LOST_NO_EXIT_FACE 2
#define SMRT_LOST_INCONSISTENT_ENTRY 3
#define SMRT_LOST_NON_FINITE 4
#define SMRT_LOST_MAX_STEPS 5
#define SMRT_LOST_RECOVERY_FAILED 6
#define SMRT_LOST_SENSE_MISMATCH 7
#define SMRT_LOST_OVERLAP 8

#define SMRT_MODE_SURFACE 0 /* SmrtSegment.mode */
#define SMRT_MODE_DELTA 1
#define SMRT_POLICY_SURFACE 0 /* SmrtFlightOpts.policy */
#define SMRT_POLICY_DELTA 1
#define SMRT_SCALE_CELL 0 /* SmrtFlightOpts.scale */
#define SMRT_SCALE_REGION 1
#define SMRT_FLIGHT_RUNNING 0 /* smrt_flight_end kinds */
#define SMRT_FLIGHT_LIMIT 1
#define SMRT_FLIGHT_LEFT_MODEL 2
#define SMRT_FLIGHT_BOUNDARY 3
#define SMRT_FLIGHT_LOST 4
#define SMRT_FLIGHT_STOPPED 5
#define SMRT_NODE_INTERNAL 0 /* SmrtNodeInfo.kind */
#define SMRT_NODE_LATTICE 1
#define SMRT_NODE_ANALYTIC 2
#define SMRT_NODE_MESH 3
#define SMRT_NODE_HOMOGENEOUS 4

/* ---- handles ------------------------------------------------------------------------------ */
typedef struct SmrtTracker SmrtTracker; /* opaque, shared, read-only */
typedef struct SmrtCtx SmrtCtx;         /* opaque, one thread at a time */
typedef struct SmrtBundle SmrtBundle;   /* opaque, an encoded track bundle */
typedef struct SmrtFlight SmrtFlight;   /* opaque, one thread at a time; keeps its tracker alive */

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

/* One node of the majorant partition. */
typedef struct SmrtNodeInfo {
    int64_t parent; /* -1 for the root */
    int64_t kind;   /* SMRT_NODE_* */
    int64_t set;    /* material set: the regions that occur inside */
    int64_t cells;
    double bounds_lo[3];
    double bounds_hi[3];
} SmrtNodeInfo;

/* Options of smrt_flight_new (NULL = defaults). */
typedef struct SmrtFlightOpts {
    double max_dist;    /* <= 0 or infinite: unlimited */
    int64_t max_steps;  /* <= 0: 1e6 segments */
    int64_t stop_at_deck;
    int64_t policy;     /* SMRT_POLICY_* */
    int64_t scale;      /* SMRT_SCALE_* */
} SmrtFlightOpts;

/* One flight segment. t0/t1 are flight parameters (distance from the flight origin);
 * step.dist is t1 - t0 and step.end/face/to_cell/... describe the event that ended it. */
typedef struct SmrtSegment {
    int64_t mode; /* SMRT_MODE_* */
    int64_t node;
    int64_t set;
    int64_t cell;   /* at t0 */
    int64_t region; /* at t0 */
    int64_t resolved; /* 1: the whole segment lies in `cell` */
    int64_t face_in;
    int64_t face_out;
    double t0;
    double t1;
    double sigma_m; /* majorants[node] of the call */
    SmrtStep step;
} SmrtSegment;

/* ---- handles and construction ------------------------------------------------------------- */
/* Opens the .smrt tracking image (geometry, connectivity, region names; no cross sections)
 * written by `session.export` format smrt. */
int32_t smrt_open(const char *path, SmrtTracker **out);
int32_t smrt_tracker_free(SmrtTracker *tracker);
int32_t smrt_ctx_new(const SmrtTracker *tracker, SmrtCtx **out);
int32_t smrt_ctx_free(SmrtCtx *ctx);
int32_t smrt_info(const SmrtTracker *tracker, SmrtInfo *out);
int32_t smrt_region_info(const SmrtTracker *tracker, int64_t region, SmrtRegionInfo *out);
int32_t smrt_version(void);
int32_t smrt_last_error_message(char *buf, int64_t cap);
/* snprintf semantics: returns the full length; buf may be NULL when cap is 0. */
int32_t smrt_region_name(const SmrtTracker *tracker, int64_t region, char *buf, int64_t cap);
int32_t smrt_region_n_tags(const SmrtTracker *tracker, int64_t region, int64_t *out_n);
int32_t smrt_region_tag(const SmrtTracker *tracker, int64_t region, int64_t index, char *buf, int64_t cap,
                        int64_t *out_value);

/* ---- majorant partition and material sets --------------------------------------------------
 * SMRT_ERR_UNSUPPORTED when the tracker has no partition. The caller pairs smrt_region_name of
 * every region of a set with its own cross sections and passes one majorant per node. */
int32_t smrt_n_nodes(const SmrtTracker *tracker, int64_t *out_n);
int32_t smrt_node_info(const SmrtTracker *tracker, int64_t node, SmrtNodeInfo *out);
int32_t smrt_node_set(const SmrtTracker *tracker, int64_t node, int64_t *out_set);
int32_t smrt_n_sets(const SmrtTracker *tracker, int64_t *out_n);
/* Writes up to cap region ids; *out_n is the full count (cap = 0 sizes the buffer). */
int32_t smrt_set_regions(const SmrtTracker *tracker, int64_t set, int64_t *out, int64_t cap, int64_t *out_n);

/* ---- flights (two-phase, callback free) ----------------------------------------------------
 * The caller's arrays are passed to every call and never retained: majorants has one entry per
 * node (n_nodes must equal smrt_n_nodes), block lists nodes forced to surface segments (sorted,
 * unique, n_block <= n_nodes, may be NULL when n_block is 0). Only the majorant of the segment's
 * leaf is read, and it must be finite and >= 0. */
int32_t smrt_flight_input_validate(const SmrtTracker *tracker, const double *majorants, size_t n_nodes,
                                   const uint32_t *block, size_t n_block);
int32_t smrt_flight_new(const SmrtTracker *tracker, const SmrtState *state, const SmrtFlightOpts *opts,
                        SmrtFlight **out);
int32_t smrt_flight_free(SmrtFlight *flight);
/* 0 = segment in *out, 1 = the flight is over (see smrt_flight_end), < 0 = error. */
int32_t smrt_flight_next(SmrtFlight *flight, const double *majorants, size_t n_nodes, const uint32_t *block,
                         size_t n_block, SmrtSegment *out);
/* Locate the point at flight parameter t inside the last segment; changes nothing. */
int32_t smrt_flight_probe(SmrtFlight *flight, const double *majorants, size_t n_nodes, const uint32_t *block,
                          size_t n_block, double t, SmrtLocated *out);
/* End the flight with a collision at t inside the last segment; the state is placed there. */
int32_t smrt_flight_stop_at(SmrtFlight *flight, const double *majorants, size_t n_nodes, const uint32_t *block,
                            size_t n_block, double t, SmrtLocated *out);
/* sigma_t against the majorant cached at the last smrt_flight_next; SMRT_ERR_MAJORANT on violation. */
int32_t smrt_flight_check(const SmrtFlight *flight, double sigma_t);
int32_t smrt_flight_set_policy(SmrtFlight *flight, int64_t policy);
int32_t smrt_flight_get_state(const SmrtFlight *flight, SmrtState *out);
/* *out_kind is a SMRT_FLIGHT_* value; out_event and out_dist may be NULL. */
int32_t smrt_flight_end(const SmrtFlight *flight, int32_t *out_kind, SmrtStep *out_event, double *out_dist);

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
/* Trace once, keep the words in a handle (no size-query retrace); *out_len = words. */
int32_t smrt_trace_bundle_new(const SmrtTracker *tracker, int64_t n, const SmrtRays *rays, int64_t block_size,
                              const SmrtTraceOpts *opts, SmrtBundle **out, uint64_t *out_len);
int32_t smrt_bundle_words(const SmrtBundle *bundle, const uint64_t **out_words, uint64_t *out_len);
int32_t smrt_bundle_free(SmrtBundle *bundle);

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
/* Area of the surface; SMRT_ERR_UNSUPPORTED when the tracker has no surface areas. */
int32_t smrt_dag_measure_area(const SmrtTracker *tracker, int64_t surface, double *out_area);
/* Faces of the surface, ascending; *out_n is the full count (cap = 0 sizes the buffer). */
int32_t smrt_dag_surface_faces(const SmrtTracker *tracker, int64_t surface, int64_t *out, int64_t cap,
                               int64_t *out_n);
/* Normal at p of the lowest-numbered face of the surface (the surface normal for a planar one). */
int32_t smrt_dag_get_angle_surface(const SmrtTracker *tracker, int64_t surface, const double p[3],
                                   double out_normal[3]);

#ifdef __cplusplus
}
#endif

#endif /* SMRT_H */
