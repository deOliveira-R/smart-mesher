! smrt.f90: Fortran (iso_c_binding) interfaces to libsmrt, the C ABI of crate sm-rt-capi.
! Mirrors include/smrt.h; see that file for the conventions (status codes, ids, threading).
! Names differ from smrt.h only where Fortran needs it: the SmrtStep component `end` is `end_code` and the
! `kind` argument of smrt_resolve_boundary is `resp_kind`; SMRT_VERSION is SMRT_ABI_VERSION (Fortran is
! case-insensitive and would clash with the function smrt_version); uint64 fields are int64 bit patterns.
! Link with -lsm_rt_capi. Handles are type(c_ptr); a context is used by one thread at a time.
module smrt
  use, intrinsic :: iso_c_binding
  implicit none

  ! ---- status codes
  integer(c_int32_t), parameter :: SMRT_OK = 0
  integer(c_int32_t), parameter :: SMRT_NO_HIT = 1
  integer(c_int32_t), parameter :: SMRT_ERR_BUILD = -1
  integer(c_int32_t), parameter :: SMRT_ERR_BAD_DIRECTION = -2
  integer(c_int32_t), parameter :: SMRT_ERR_NON_FINITE = -3
  integer(c_int32_t), parameter :: SMRT_ERR_UNKNOWN_ID = -4
  integer(c_int32_t), parameter :: SMRT_ERR_UNSUPPORTED = -5
  integer(c_int32_t), parameter :: SMRT_ERR_CACHE_STALE = -6
  integer(c_int32_t), parameter :: SMRT_ERR_CACHE_CORRUPT = -7
  integer(c_int32_t), parameter :: SMRT_ERR_GEOM = -8
  integer(c_int32_t), parameter :: SMRT_ERR_BAD_ALBEDO = -9
  integer(c_int32_t), parameter :: SMRT_ERR_BIN_COUNT = -10
  integer(c_int32_t), parameter :: SMRT_ERR_INVALID_INPUT = -11
  integer(c_int32_t), parameter :: SMRT_ERR_MAJORANT = -12
  integer(c_int32_t), parameter :: SMRT_ERR_INVALID_MAJORANT = -13
  integer(c_int32_t), parameter :: SMRT_ERR_NULL = -100
  integer(c_int32_t), parameter :: SMRT_ERR_PANIC = -101
  integer(c_int32_t), parameter :: SMRT_ERR_BAD_ARG = -102
  integer(c_int32_t), parameter :: SMRT_ERR_LOST = -103

  ! ---- ids and enums
  integer(c_int32_t), parameter :: SMRT_ABI_VERSION = 10200
  integer(c_int64_t), parameter :: SMRT_NONE = -1_c_int64_t
  integer(c_int64_t), parameter :: SMRT_COMPLEMENT = -2_c_int64_t
  integer(c_int32_t), parameter :: SMRT_MAX_LEVELS = 8

  integer(c_int32_t), parameter :: SMRT_END_LIMIT = 0
  integer(c_int32_t), parameter :: SMRT_END_CELL_FACE = 1
  integer(c_int32_t), parameter :: SMRT_END_SLOT_CHANGE = 2
  integer(c_int32_t), parameter :: SMRT_END_REGION_BOUNDARY = 3
  integer(c_int32_t), parameter :: SMRT_END_BOUNDARY = 4
  integer(c_int32_t), parameter :: SMRT_END_LOST = 5

  integer(c_int32_t), parameter :: SMRT_ACT_NONE = 0
  integer(c_int32_t), parameter :: SMRT_ACT_VACUUM = 1
  integer(c_int32_t), parameter :: SMRT_ACT_WHITE = 2
  integer(c_int32_t), parameter :: SMRT_ACT_ALBEDO = 3
  integer(c_int32_t), parameter :: SMRT_ACT_REFLECTIVE = 4
  integer(c_int32_t), parameter :: SMRT_ACT_MIRROR = 10
  integer(c_int32_t), parameter :: SMRT_ACT_WRAP = 11

  integer(c_int32_t), parameter :: SMRT_RESP_VACUUM = 1
  integer(c_int32_t), parameter :: SMRT_RESP_WHITE = 2
  integer(c_int32_t), parameter :: SMRT_RESP_ALBEDO = 3
  integer(c_int32_t), parameter :: SMRT_RESP_REFLECTIVE = 4

  integer(c_int32_t), parameter :: SMRT_ORIENT_EXITING = 1
  integer(c_int32_t), parameter :: SMRT_ORIENT_ENTERING = -1
  integer(c_int32_t), parameter :: SMRT_ORIENT_ANY = 0

  integer(c_int32_t), parameter :: SMRT_LOST_NONE = 0
  integer(c_int32_t), parameter :: SMRT_LOST_NO_CELL = 1
  integer(c_int32_t), parameter :: SMRT_LOST_NO_EXIT_FACE = 2
  integer(c_int32_t), parameter :: SMRT_LOST_INCONSISTENT_ENTRY = 3
  integer(c_int32_t), parameter :: SMRT_LOST_NON_FINITE = 4
  integer(c_int32_t), parameter :: SMRT_LOST_MAX_STEPS = 5
  integer(c_int32_t), parameter :: SMRT_LOST_RECOVERY_FAILED = 6
  integer(c_int32_t), parameter :: SMRT_LOST_SENSE_MISMATCH = 7
  integer(c_int32_t), parameter :: SMRT_LOST_OVERLAP = 8

  integer(c_int64_t), parameter :: SMRT_MODE_SURFACE = 0_c_int64_t
  integer(c_int64_t), parameter :: SMRT_MODE_DELTA = 1_c_int64_t
  integer(c_int64_t), parameter :: SMRT_POLICY_SURFACE = 0_c_int64_t
  integer(c_int64_t), parameter :: SMRT_POLICY_DELTA = 1_c_int64_t
  integer(c_int64_t), parameter :: SMRT_SCALE_CELL = 0_c_int64_t
  integer(c_int64_t), parameter :: SMRT_SCALE_REGION = 1_c_int64_t
  integer(c_int32_t), parameter :: SMRT_FLIGHT_RUNNING = 0
  integer(c_int32_t), parameter :: SMRT_FLIGHT_LIMIT = 1
  integer(c_int32_t), parameter :: SMRT_FLIGHT_LEFT_MODEL = 2
  integer(c_int32_t), parameter :: SMRT_FLIGHT_BOUNDARY = 3
  integer(c_int32_t), parameter :: SMRT_FLIGHT_LOST = 4
  integer(c_int32_t), parameter :: SMRT_FLIGHT_STOPPED = 5
  integer(c_int64_t), parameter :: SMRT_NODE_INTERNAL = 0_c_int64_t
  integer(c_int64_t), parameter :: SMRT_NODE_LATTICE = 1_c_int64_t
  integer(c_int64_t), parameter :: SMRT_NODE_ANALYTIC = 2_c_int64_t
  integer(c_int64_t), parameter :: SMRT_NODE_MESH = 3_c_int64_t
  integer(c_int64_t), parameter :: SMRT_NODE_HOMOGENEOUS = 4_c_int64_t
  integer(c_int64_t), parameter :: SMRT_NODE_UNKNOWN = 5_c_int64_t

  ! ---- data (layouts identical to smrt.h)
  type, bind(C) :: SmrtLevel
    integer(c_int64_t) :: component
    integer(c_int64_t) :: instance
    integer(c_int32_t) :: lattice(3)
    integer(c_int32_t) :: pad
  end type SmrtLevel

  type, bind(C) :: SmrtState
    real(c_double) :: pos(3)
    real(c_double) :: dir(3)
    integer(c_int64_t) :: cell
    integer(c_int64_t) :: region
    integer(c_int64_t) :: entered_via
    integer(c_int32_t) :: levels_total
    integer(c_int32_t) :: levels_len
    type(SmrtLevel) :: levels(8)
  end type SmrtState

  type, bind(C) :: SmrtStep
    real(c_double) :: dist
    integer(c_int64_t) :: cell
    integer(c_int64_t) :: region
    integer(c_int32_t) :: end_code
    integer(c_int32_t) :: action
    integer(c_int64_t) :: face
    integer(c_int64_t) :: surface
    integer(c_int64_t) :: to_region
    integer(c_int64_t) :: to_cell
    real(c_double) :: albedo
    integer(c_int32_t) :: lost_cause
    integer(c_int32_t) :: pad
    integer(c_int64_t) :: lost_cell
    real(c_double) :: lost_pos(3)
    real(c_double) :: lost_dir(3)
  end type SmrtStep

  type, bind(C) :: SmrtLocated
    integer(c_int64_t) :: cell
    integer(c_int64_t) :: region
    integer(c_int64_t) :: on_face
    integer(c_int32_t) :: levels_total
    integer(c_int32_t) :: levels_len
    type(SmrtLevel) :: levels(8)
  end type SmrtLocated

  type, bind(C) :: SmrtClosest
    real(c_double) :: dist
    integer(c_int64_t) :: surface
    real(c_double) :: point(3)
  end type SmrtClosest

  type, bind(C) :: SmrtInfo
    integer(c_int64_t) :: cells
    integer(c_int64_t) :: faces
    integer(c_int64_t) :: regions
    integer(c_int64_t) :: surfaces
    integer(c_int64_t) :: prototypes
    integer(c_int64_t) :: copies
    integer(c_int64_t) :: memory_bytes
    integer(c_int64_t) :: exact_faces
    integer(c_int64_t) :: faceted_faces
    integer(c_int64_t) :: caps
    integer(c_int64_t) :: complement_region
    integer(c_int64_t) :: max_events
    integer(c_int64_t) :: structure_hash_lo   ! uint64 bit pattern
    integer(c_int64_t) :: structure_hash_hi   ! uint64 bit pattern
    real(c_double) :: bounds_lo(3)
    real(c_double) :: bounds_hi(3)
    real(c_double) :: tol_linear
    real(c_double) :: tol_angular
  end type SmrtInfo

  type, bind(C) :: SmrtRegionInfo
    integer(c_int64_t) :: original_id
    integer(c_int64_t) :: color_id
    real(c_double) :: measure
    integer(c_int64_t) :: implicit_complement
  end type SmrtRegionInfo

  type, bind(C) :: SmrtTraceOpts
    real(c_double) :: max_dist
    integer(c_int64_t) :: max_steps
    integer(c_int64_t) :: stop_at_deck
  end type SmrtTraceOpts

  type, bind(C) :: SmrtNodeInfo
    integer(c_int64_t) :: parent
    integer(c_int64_t) :: kind
    integer(c_int64_t) :: set
    integer(c_int64_t) :: cells
    real(c_double) :: bounds_lo(3)
    real(c_double) :: bounds_hi(3)
  end type SmrtNodeInfo

  type, bind(C) :: SmrtFlightOpts
    real(c_double) :: max_dist
    integer(c_int64_t) :: max_steps
    integer(c_int64_t) :: stop_at_deck
    integer(c_int64_t) :: policy
    integer(c_int64_t) :: scale
  end type SmrtFlightOpts

  type, bind(C) :: SmrtSegment
    integer(c_int64_t) :: mode
    integer(c_int64_t) :: node
    integer(c_int64_t) :: set
    integer(c_int64_t) :: cell
    integer(c_int64_t) :: region
    integer(c_int64_t) :: resolved
    integer(c_int64_t) :: face_in
    integer(c_int64_t) :: face_out
    real(c_double) :: t0
    real(c_double) :: t1
    real(c_double) :: sigma_m
    type(SmrtStep) :: step
  end type SmrtSegment

  type, bind(C) :: SmrtRays
    type(c_ptr) :: x, y, z
    type(c_ptr) :: u, v, w
    type(c_ptr) :: start_cell
  end type SmrtRays

  interface
    ! ---- handles and construction
    function smrt_open(path, out) bind(C, name="smrt_open") result(rc)
      import :: c_char, c_ptr, c_int32_t
      character(kind=c_char), intent(in) :: path(*)
      type(c_ptr), intent(out) :: out
      integer(c_int32_t) :: rc
    end function smrt_open

    function smrt_tracker_free(tracker) bind(C, name="smrt_tracker_free") result(rc)
      import :: c_ptr, c_int32_t
      type(c_ptr), value :: tracker
      integer(c_int32_t) :: rc
    end function smrt_tracker_free

    function smrt_ctx_new(tracker, out) bind(C, name="smrt_ctx_new") result(rc)
      import :: c_ptr, c_int32_t
      type(c_ptr), value :: tracker
      type(c_ptr), intent(out) :: out
      integer(c_int32_t) :: rc
    end function smrt_ctx_new

    function smrt_ctx_free(ctx) bind(C, name="smrt_ctx_free") result(rc)
      import :: c_ptr, c_int32_t
      type(c_ptr), value :: ctx
      integer(c_int32_t) :: rc
    end function smrt_ctx_free

    function smrt_info(tracker, out) bind(C, name="smrt_info") result(rc)
      import :: c_ptr, c_int32_t, SmrtInfo
      type(c_ptr), value :: tracker
      type(SmrtInfo), intent(out) :: out
      integer(c_int32_t) :: rc
    end function smrt_info

    function smrt_region_info(tracker, region, out) bind(C, name="smrt_region_info") result(rc)
      import :: c_ptr, c_int32_t, c_int64_t, SmrtRegionInfo
      type(c_ptr), value :: tracker
      integer(c_int64_t), value :: region
      type(SmrtRegionInfo), intent(out) :: out
      integer(c_int32_t) :: rc
    end function smrt_region_info

    function smrt_version() bind(C, name="smrt_version") result(v)
      import :: c_int32_t
      integer(c_int32_t) :: v
    end function smrt_version

    function smrt_last_error_message(buf, cap) bind(C, name="smrt_last_error_message") result(n)
      import :: c_char, c_int32_t, c_int64_t
      character(kind=c_char), intent(out) :: buf(*)
      integer(c_int64_t), value :: cap
      integer(c_int32_t) :: n
    end function smrt_last_error_message

    ! ---- tracking
    function smrt_init_state(ctx, x, y, z, u, v, w, hint_cell) bind(C, name="smrt_init_state") result(rc)
      import :: c_ptr, c_int32_t, c_int64_t, c_double
      type(c_ptr), value :: ctx
      real(c_double), value :: x, y, z, u, v, w
      integer(c_int64_t), value :: hint_cell
      integer(c_int32_t) :: rc
    end function smrt_init_state

    function smrt_get_state(ctx, out) bind(C, name="smrt_get_state") result(rc)
      import :: c_ptr, c_int32_t, SmrtState
      type(c_ptr), value :: ctx
      type(SmrtState), intent(out) :: out
      integer(c_int32_t) :: rc
    end function smrt_get_state

    function smrt_set_state(ctx, state) bind(C, name="smrt_set_state") result(rc)
      import :: c_ptr, c_int32_t, SmrtState
      type(c_ptr), value :: ctx
      type(SmrtState), intent(in) :: state
      integer(c_int32_t) :: rc
    end function smrt_set_state

    function smrt_set_direction(ctx, u, v, w) bind(C, name="smrt_set_direction") result(rc)
      import :: c_ptr, c_int32_t, c_double
      type(c_ptr), value :: ctx
      real(c_double), value :: u, v, w
      integer(c_int32_t) :: rc
    end function smrt_set_direction

    function smrt_step(ctx, max_dist, out) bind(C, name="smrt_step") result(rc)
      import :: c_ptr, c_int32_t, c_double, SmrtStep
      type(c_ptr), value :: ctx
      real(c_double), value :: max_dist
      type(SmrtStep), intent(out) :: out
      integer(c_int32_t) :: rc
    end function smrt_step

    function smrt_peek(ctx, limit, out) bind(C, name="smrt_peek") result(rc)
      import :: c_ptr, c_int32_t, c_double, SmrtStep
      type(c_ptr), value :: ctx
      real(c_double), value :: limit
      type(SmrtStep), intent(out) :: out
      integer(c_int32_t) :: rc
    end function smrt_peek

    function smrt_fire(ctx, dist_limit, max_steps, out) bind(C, name="smrt_fire") result(rc)
      import :: c_ptr, c_int32_t, c_int64_t, c_double, SmrtStep
      type(c_ptr), value :: ctx
      real(c_double), value :: dist_limit
      integer(c_int64_t), value :: max_steps
      type(SmrtStep), intent(out) :: out
      integer(c_int32_t) :: rc
    end function smrt_fire

    function smrt_resolve_boundary(ctx, face, resp_kind, albedo, u0, u1, out_dir) &
        bind(C, name="smrt_resolve_boundary") result(rc)
      import :: c_ptr, c_int32_t, c_int64_t, c_double
      type(c_ptr), value :: ctx
      integer(c_int64_t), value :: face
      integer(c_int32_t), value :: resp_kind
      real(c_double), value :: albedo, u0, u1
      real(c_double), intent(out) :: out_dir(3)
      integer(c_int32_t) :: rc
    end function smrt_resolve_boundary

    function smrt_locate(ctx, p, dir_or_null, hint, out) bind(C, name="smrt_locate") result(rc)
      import :: c_ptr, c_int32_t, c_int64_t, c_double, SmrtLocated
      type(c_ptr), value :: ctx
      real(c_double), intent(in) :: p(3)
      type(c_ptr), value :: dir_or_null
      integer(c_int64_t), value :: hint
      type(SmrtLocated), intent(out) :: out
      integer(c_int32_t) :: rc
    end function smrt_locate

    function smrt_closest(ctx, p, region, out) bind(C, name="smrt_closest") result(rc)
      import :: c_ptr, c_int32_t, c_int64_t, c_double, SmrtClosest
      type(c_ptr), value :: ctx
      real(c_double), intent(in) :: p(3)
      integer(c_int64_t), value :: region
      type(SmrtClosest), intent(out) :: out
      integer(c_int32_t) :: rc
    end function smrt_closest

    function smrt_trace_bundle(tracker, n, rays, block_size, opts, out_buf, cap, out_len) &
        bind(C, name="smrt_trace_bundle") result(rc)
      import :: c_ptr, c_int32_t, c_int64_t, SmrtRays
      type(c_ptr), value :: tracker
      integer(c_int64_t), value :: n
      type(SmrtRays), intent(in) :: rays
      integer(c_int64_t), value :: block_size
      type(c_ptr), value :: opts
      type(c_ptr), value :: out_buf
      integer(c_int64_t), value :: cap
      integer(c_int64_t), intent(out) :: out_len
      integer(c_int32_t) :: rc
    end function smrt_trace_bundle

    function smrt_trace_bundle_new(tracker, n, rays, block_size, opts, out, out_len) &
        bind(C, name="smrt_trace_bundle_new") result(rc)
      import :: c_ptr, c_int32_t, c_int64_t, SmrtRays
      type(c_ptr), value :: tracker
      integer(c_int64_t), value :: n
      type(SmrtRays), intent(in) :: rays
      integer(c_int64_t), value :: block_size
      type(c_ptr), value :: opts
      type(c_ptr), intent(out) :: out
      integer(c_int64_t), intent(out) :: out_len
      integer(c_int32_t) :: rc
    end function smrt_trace_bundle_new

    function smrt_bundle_words(bundle, out_words, out_len) bind(C, name="smrt_bundle_words") result(rc)
      import :: c_ptr, c_int32_t, c_int64_t
      type(c_ptr), value :: bundle
      type(c_ptr), intent(out) :: out_words
      integer(c_int64_t), intent(out) :: out_len
      integer(c_int32_t) :: rc
    end function smrt_bundle_words

    function smrt_bundle_free(bundle) bind(C, name="smrt_bundle_free") result(rc)
      import :: c_ptr, c_int32_t
      type(c_ptr), value :: bundle
      integer(c_int32_t) :: rc
    end function smrt_bundle_free

    ! ---- DAGMC-shaped shims
    function smrt_dag_ray_fire(tracker, ctx, region, origin, dir, orientation, dist_limit, &
        out_surface, out_dist, out_face) bind(C, name="smrt_dag_ray_fire") result(rc)
      import :: c_ptr, c_int32_t, c_int64_t, c_double
      type(c_ptr), value :: tracker
      type(c_ptr), value :: ctx
      integer(c_int64_t), value :: region
      real(c_double), intent(in) :: origin(3)
      real(c_double), intent(in) :: dir(3)
      integer(c_int32_t), value :: orientation
      real(c_double), value :: dist_limit
      integer(c_int64_t), intent(out) :: out_surface
      real(c_double), intent(out) :: out_dist
      type(c_ptr), value :: out_face
      integer(c_int32_t) :: rc
    end function smrt_dag_ray_fire

    function smrt_dag_point_in_volume(tracker, ctx, region, p, dir_or_null, out_inside) &
        bind(C, name="smrt_dag_point_in_volume") result(rc)
      import :: c_ptr, c_int32_t, c_int64_t, c_double
      type(c_ptr), value :: tracker
      type(c_ptr), value :: ctx
      integer(c_int64_t), value :: region
      real(c_double), intent(in) :: p(3)
      type(c_ptr), value :: dir_or_null
      integer(c_int32_t), intent(out) :: out_inside
      integer(c_int32_t) :: rc
    end function smrt_dag_point_in_volume

    function smrt_dag_next_vol(tracker, surface, old_region, out_new_region) &
        bind(C, name="smrt_dag_next_vol") result(rc)
      import :: c_ptr, c_int32_t, c_int64_t
      type(c_ptr), value :: tracker
      integer(c_int64_t), value :: surface
      integer(c_int64_t), value :: old_region
      integer(c_int64_t), intent(out) :: out_new_region
      integer(c_int32_t) :: rc
    end function smrt_dag_next_vol

    function smrt_dag_surface_sense(tracker, surface, region, out_sense) &
        bind(C, name="smrt_dag_surface_sense") result(rc)
      import :: c_ptr, c_int32_t, c_int64_t
      type(c_ptr), value :: tracker
      integer(c_int64_t), value :: surface
      integer(c_int64_t), value :: region
      integer(c_int32_t), intent(out) :: out_sense
      integer(c_int32_t) :: rc
    end function smrt_dag_surface_sense

    function smrt_dag_get_angle(tracker, face, p, out_normal) bind(C, name="smrt_dag_get_angle") result(rc)
      import :: c_ptr, c_int32_t, c_int64_t, c_double
      type(c_ptr), value :: tracker
      integer(c_int64_t), value :: face
      real(c_double), intent(in) :: p(3)
      real(c_double), intent(out) :: out_normal(3)
      integer(c_int32_t) :: rc
    end function smrt_dag_get_angle

    function smrt_dag_measure_volume(tracker, region, out_volume) &
        bind(C, name="smrt_dag_measure_volume") result(rc)
      import :: c_ptr, c_int32_t, c_int64_t, c_double
      type(c_ptr), value :: tracker
      integer(c_int64_t), value :: region
      real(c_double), intent(out) :: out_volume
      integer(c_int32_t) :: rc
    end function smrt_dag_measure_volume

    function smrt_dag_measure_area(tracker, surface, out_area) bind(C, name="smrt_dag_measure_area") result(rc)
      import :: c_ptr, c_int32_t, c_int64_t, c_double
      type(c_ptr), value :: tracker
      integer(c_int64_t), value :: surface
      real(c_double), intent(out) :: out_area
      integer(c_int32_t) :: rc
    end function smrt_dag_measure_area

    function smrt_dag_surface_faces(tracker, surface, out, cap, out_n) bind(C, name="smrt_dag_surface_faces") result(rc)
      import :: c_ptr, c_int32_t, c_int64_t
      type(c_ptr), value :: tracker
      integer(c_int64_t), value :: surface
      integer(c_int64_t), intent(out) :: out(*)
      integer(c_int64_t), value :: cap
      integer(c_int64_t), intent(out) :: out_n
      integer(c_int32_t) :: rc
    end function smrt_dag_surface_faces

    function smrt_dag_get_angle_surface(tracker, surface, p, out_normal) &
        bind(C, name="smrt_dag_get_angle_surface") result(rc)
      import :: c_ptr, c_int32_t, c_int64_t, c_double
      type(c_ptr), value :: tracker
      integer(c_int64_t), value :: surface
      real(c_double), intent(in) :: p(3)
      real(c_double), intent(out) :: out_normal(3)
      integer(c_int32_t) :: rc
    end function smrt_dag_get_angle_surface

    ! ---- region names and tags
    function smrt_region_name(tracker, region, buf, cap) bind(C, name="smrt_region_name") result(n)
      import :: c_ptr, c_char, c_int32_t, c_int64_t
      type(c_ptr), value :: tracker
      integer(c_int64_t), value :: region
      character(kind=c_char), intent(out) :: buf(*)
      integer(c_int64_t), value :: cap
      integer(c_int32_t) :: n
    end function smrt_region_name

    function smrt_region_n_tags(tracker, region, out_n) bind(C, name="smrt_region_n_tags") result(rc)
      import :: c_ptr, c_int32_t, c_int64_t
      type(c_ptr), value :: tracker
      integer(c_int64_t), value :: region
      integer(c_int64_t), intent(out) :: out_n
      integer(c_int32_t) :: rc
    end function smrt_region_n_tags

    function smrt_region_tag(tracker, region, index, buf, cap, out_value) bind(C, name="smrt_region_tag") result(n)
      import :: c_ptr, c_char, c_int32_t, c_int64_t
      type(c_ptr), value :: tracker
      integer(c_int64_t), value :: region
      integer(c_int64_t), value :: index
      character(kind=c_char), intent(out) :: buf(*)
      integer(c_int64_t), value :: cap
      integer(c_int64_t), intent(out) :: out_value
      integer(c_int32_t) :: n
    end function smrt_region_tag

    ! ---- majorant partition and material sets
    function smrt_n_nodes(tracker, out_n) bind(C, name="smrt_n_nodes") result(rc)
      import :: c_ptr, c_int32_t, c_int64_t
      type(c_ptr), value :: tracker
      integer(c_int64_t), intent(out) :: out_n
      integer(c_int32_t) :: rc
    end function smrt_n_nodes

    function smrt_node_info(tracker, node, out) bind(C, name="smrt_node_info") result(rc)
      import :: c_ptr, c_int32_t, c_int64_t, SmrtNodeInfo
      type(c_ptr), value :: tracker
      integer(c_int64_t), value :: node
      type(SmrtNodeInfo), intent(out) :: out
      integer(c_int32_t) :: rc
    end function smrt_node_info

    function smrt_node_set(tracker, node, out_set) bind(C, name="smrt_node_set") result(rc)
      import :: c_ptr, c_int32_t, c_int64_t
      type(c_ptr), value :: tracker
      integer(c_int64_t), value :: node
      integer(c_int64_t), intent(out) :: out_set
      integer(c_int32_t) :: rc
    end function smrt_node_set

    function smrt_n_sets(tracker, out_n) bind(C, name="smrt_n_sets") result(rc)
      import :: c_ptr, c_int32_t, c_int64_t
      type(c_ptr), value :: tracker
      integer(c_int64_t), intent(out) :: out_n
      integer(c_int32_t) :: rc
    end function smrt_n_sets

    function smrt_set_regions(tracker, set, out, cap, out_n) bind(C, name="smrt_set_regions") result(rc)
      import :: c_ptr, c_int32_t, c_int64_t
      type(c_ptr), value :: tracker
      integer(c_int64_t), value :: set
      integer(c_int64_t), intent(out) :: out(*)
      integer(c_int64_t), value :: cap
      integer(c_int64_t), intent(out) :: out_n
      integer(c_int32_t) :: rc
    end function smrt_set_regions

    ! ---- flights (arrays are passed to every call and never retained)
    function smrt_flight_input_validate(tracker, majorants, n_nodes, block, n_block) &
        bind(C, name="smrt_flight_input_validate") result(rc)
      import :: c_ptr, c_int32_t, c_size_t, c_double
      type(c_ptr), value :: tracker
      real(c_double), intent(in) :: majorants(*)
      integer(c_size_t), value :: n_nodes
      type(c_ptr), value :: block   ! uint32_t array or C_NULL_PTR
      integer(c_size_t), value :: n_block
      integer(c_int32_t) :: rc
    end function smrt_flight_input_validate

    function smrt_flight_new(tracker, state, opts, out) bind(C, name="smrt_flight_new") result(rc)
      import :: c_ptr, c_int32_t, SmrtState
      type(c_ptr), value :: tracker
      type(SmrtState), intent(in) :: state
      type(c_ptr), value :: opts   ! SmrtFlightOpts or C_NULL_PTR
      type(c_ptr), intent(out) :: out
      integer(c_int32_t) :: rc
    end function smrt_flight_new

    function smrt_flight_free(flight) bind(C, name="smrt_flight_free") result(rc)
      import :: c_ptr, c_int32_t
      type(c_ptr), value :: flight
      integer(c_int32_t) :: rc
    end function smrt_flight_free

    function smrt_flight_next(flight, majorants, n_nodes, block, n_block, out) bind(C, name="smrt_flight_next") result(rc)
      import :: c_ptr, c_int32_t, c_size_t, c_double, SmrtSegment
      type(c_ptr), value :: flight
      real(c_double), intent(in) :: majorants(*)
      integer(c_size_t), value :: n_nodes
      type(c_ptr), value :: block
      integer(c_size_t), value :: n_block
      type(SmrtSegment), intent(out) :: out
      integer(c_int32_t) :: rc
    end function smrt_flight_next

    function smrt_flight_probe(flight, majorants, n_nodes, block, n_block, t, out) &
        bind(C, name="smrt_flight_probe") result(rc)
      import :: c_ptr, c_int32_t, c_size_t, c_double, SmrtLocated
      type(c_ptr), value :: flight
      real(c_double), intent(in) :: majorants(*)
      integer(c_size_t), value :: n_nodes
      type(c_ptr), value :: block
      integer(c_size_t), value :: n_block
      real(c_double), value :: t
      type(SmrtLocated), intent(out) :: out
      integer(c_int32_t) :: rc
    end function smrt_flight_probe

    function smrt_flight_stop_at(flight, majorants, n_nodes, block, n_block, t, out) &
        bind(C, name="smrt_flight_stop_at") result(rc)
      import :: c_ptr, c_int32_t, c_size_t, c_double, SmrtLocated
      type(c_ptr), value :: flight
      real(c_double), intent(in) :: majorants(*)
      integer(c_size_t), value :: n_nodes
      type(c_ptr), value :: block
      integer(c_size_t), value :: n_block
      real(c_double), value :: t
      type(SmrtLocated), intent(out) :: out
      integer(c_int32_t) :: rc
    end function smrt_flight_stop_at

    function smrt_flight_check(flight, sigma_t) bind(C, name="smrt_flight_check") result(rc)
      import :: c_ptr, c_int32_t, c_double
      type(c_ptr), value :: flight
      real(c_double), value :: sigma_t
      integer(c_int32_t) :: rc
    end function smrt_flight_check

    function smrt_flight_set_policy(flight, policy) bind(C, name="smrt_flight_set_policy") result(rc)
      import :: c_ptr, c_int32_t, c_int64_t
      type(c_ptr), value :: flight
      integer(c_int64_t), value :: policy
      integer(c_int32_t) :: rc
    end function smrt_flight_set_policy

    function smrt_flight_get_state(flight, out) bind(C, name="smrt_flight_get_state") result(rc)
      import :: c_ptr, c_int32_t, SmrtState
      type(c_ptr), value :: flight
      type(SmrtState), intent(out) :: out
      integer(c_int32_t) :: rc
    end function smrt_flight_get_state

    function smrt_flight_end(flight, out_kind, out_event, out_dist) bind(C, name="smrt_flight_end") result(rc)
      import :: c_ptr, c_int32_t, c_double
      type(c_ptr), value :: flight
      integer(c_int32_t), intent(out) :: out_kind
      type(c_ptr), value :: out_event   ! SmrtStep or C_NULL_PTR
      type(c_ptr), value :: out_dist    ! double or C_NULL_PTR
      integer(c_int32_t) :: rc
    end function smrt_flight_end
  end interface
end module smrt
