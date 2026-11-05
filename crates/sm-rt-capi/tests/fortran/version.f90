program smrt_version_check
  use smrt
  use, intrinsic :: iso_c_binding
  implicit none
  integer(c_int32_t) :: v
  type(c_ptr) :: trk
  integer(c_int32_t) :: rc
  v = smrt_version()
  if (v /= SMRT_ABI_VERSION) then
    print *, 'version mismatch', v
    stop 1
  end if
  ! smrt_open of a missing file is a bad argument (unsupported only in a build without the loader).
  rc = smrt_open('/nonexistent/none.smrt' // c_null_char, trk)
  if (rc /= SMRT_ERR_BAD_ARG .and. rc /= SMRT_ERR_UNSUPPORTED) then
    print *, 'smrt_open status', rc
    stop 2
  end if
  print *, 'smrt ok', v
end program smrt_version_check
