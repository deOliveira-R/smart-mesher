! Surface flight and the collision check through the Fortran interfaces (stub STUB_GRID, R = 5).
program smrt_flight
  use smrt
  use, intrinsic :: iso_c_binding
  implicit none
  interface
    function smrt_test_stub(kind, radius, out) bind(C, name="smrt_test_stub") result(rc)
      import :: c_ptr, c_int32_t, c_double
      integer(c_int32_t), value :: kind
      real(c_double), value :: radius
      type(c_ptr), intent(out) :: out
      integer(c_int32_t) :: rc
    end function smrt_test_stub
  end interface
  type(c_ptr) :: trk, ctx, fl
  type(SmrtState) :: st
  type(SmrtSegment) :: seg
  type(SmrtFlightOpts), target :: opts
  integer(c_int64_t) :: nn
  integer(c_int32_t) :: rc, kind
  real(c_double), allocatable :: maj(:)
  real(c_double) :: total
  character(kind=c_char) :: name(32)

  rc = smrt_test_stub(8_c_int32_t, 5.0_c_double, trk);   if (rc /= 0) stop 1
  rc = smrt_n_nodes(trk, nn);                             if (rc /= 0 .or. nn /= 9) stop 2
  rc = smrt_region_name(trk, 0_c_int64_t, name, 32_c_int64_t);  if (rc < 0) stop 3
  allocate(maj(nn))
  maj = 2.0_c_double
  rc = smrt_ctx_new(trk, ctx);                            if (rc /= 0) stop 4
  rc = smrt_init_state(ctx, -9.0_c_double, 1.0_c_double, 0.0_c_double, 1.0_c_double, 0.0_c_double, &
                       0.0_c_double, SMRT_NONE);          if (rc /= 0) stop 5
  rc = smrt_get_state(ctx, st);                           if (rc /= 0) stop 6
  opts%max_dist = 0.0_c_double
  opts%max_steps = 0_c_int64_t
  opts%stop_at_deck = 0_c_int64_t
  opts%policy = SMRT_POLICY_SURFACE
  opts%scale = SMRT_SCALE_CELL
  rc = smrt_flight_new(trk, st, c_loc(opts), fl);         if (rc /= 0) stop 7
  total = 0.0_c_double
  do
    rc = smrt_flight_next(fl, maj, int(nn, c_size_t), c_null_ptr, 0_c_size_t, seg)
    if (rc == 1) exit
    if (rc /= 0) stop 8
    total = total + (seg%t1 - seg%t0)
  end do
  ! The line y = 1, z = 0 crosses x = -9 .. 10: a chord of 19 in the cubic domain.
  if (abs(total - 19.0_c_double) > 1.0e-9_c_double) stop 9
  rc = smrt_flight_end(fl, kind, c_null_ptr, c_null_ptr)
  if (rc /= 0 .or. kind /= SMRT_FLIGHT_BOUNDARY) stop 10
  rc = smrt_flight_check(fl, 1.0_c_double);               if (rc /= SMRT_ERR_INVALID_INPUT) stop 11
  rc = smrt_flight_free(fl)
  rc = smrt_ctx_free(ctx)
  rc = smrt_tracker_free(trk)
  print *, 'fortran flight ok'
end program smrt_flight
