! Sphere chord through the Fortran interfaces (stub sphere R = 5), plus struct size checks.
program smrt_chord
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
  type(c_ptr) :: trk, ctx
  type(SmrtStep) :: s
  type(SmrtState) :: st
  type(SmrtInfo) :: info
  integer(c_int32_t) :: rc
  real(c_double), parameter :: r = 5.0_c_double, b = 3.0_c_double

  rc = smrt_test_stub(1_c_int32_t, r, trk);          if (rc /= 0) stop 1
  rc = smrt_ctx_new(trk, ctx);                        if (rc /= 0) stop 2
  rc = smrt_init_state(ctx, -9.0_c_double, b, 0.0_c_double, 1.0_c_double, 0.0_c_double, &
                       0.0_c_double, SMRT_NONE);      if (rc /= 0) stop 3
  rc = smrt_fire(ctx, huge(1.0_c_double), 0_c_int64_t, s)
  if (rc /= 0 .or. s%end_code /= SMRT_END_REGION_BOUNDARY .or. s%to_region /= 0) stop 4
  rc = smrt_fire(ctx, huge(1.0_c_double), 0_c_int64_t, s)
  if (rc /= 0 .or. s%to_region /= SMRT_COMPLEMENT) stop 5
  if (abs(s%dist - 2.0_c_double * sqrt(r * r - b * b)) > 1.0e-12_c_double) stop 6
  rc = smrt_get_state(ctx, st);                       if (rc /= 0) stop 7
  if (st%cell /= 1 .or. st%region /= SMRT_COMPLEMENT) stop 8
  rc = smrt_info(trk, info);                          if (rc /= 0) stop 9
  if (info%cells /= 2 .or. info%complement_region /= 1) stop 10
  if (storage_size(s) / 8 /= SIZE_STEP) stop 11
  if (storage_size(st) / 8 /= SIZE_STATE) stop 12
  if (storage_size(info) / 8 /= SIZE_INFO) stop 13
  rc = smrt_ctx_free(ctx)
  rc = smrt_tracker_free(trk)
  print *, 'fortran chord ok'
end program smrt_chord
