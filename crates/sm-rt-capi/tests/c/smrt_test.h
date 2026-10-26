/* Test-only hooks (crate feature `stubs`): stub trackers over the contract's AnalyticWorld. */
#ifndef SMRT_TEST_H
#define SMRT_TEST_H
#include "smrt.h"

#define STUB_SPHERE 1          /* sphere of `radius` at the origin in [-10,10]^3, vacuum */
#define STUB_SHELLS 2          /* spheres of radius radius*k/5, k=1..5 */
#define STUB_MIRROR_BOX 3      /* empty box, mirror deck on all faces */
#define STUB_WHITE_BOX 4       /* empty box, white response */
#define STUB_REFLECTIVE_BOX 5  /* empty box, reflective response */
#define STUB_ALBEDO_BOX 6      /* empty box, albedo 0.5 response */
#define STUB_LATTICE 7         /* 3x3x3 lattice, pitch 4, domain [0,12]^3, sphere of `radius` per element */
#define STUB_PANIC_ON_STEP 99  /* STUB_SPHERE whose step() panics */

int32_t smrt_test_stub(int32_t kind, double radius, SmrtTracker **out);
#endif
