#ifndef CHECK_H
#define CHECK_H
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#define CHECK(cond)                                                                    \
    do {                                                                               \
        if (!(cond)) {                                                                 \
            fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #cond);  \
            exit(1);                                                                   \
        }                                                                              \
    } while (0)
#define CHECK_NEAR(a, b, tol)                                                          \
    do {                                                                               \
        double a_ = (a), b_ = (b);                                                     \
        if (!(fabs(a_ - b_) <= (tol))) {                                               \
            fprintf(stderr, "%s:%d: %s = %.17g, expected %.17g (tol %g)\n", __FILE__,  \
                    __LINE__, #a, a_, b_, (double)(tol));                              \
            exit(1);                                                                   \
        }                                                                              \
    } while (0)

static uint64_t rng_state = 0x9E3779B97F4A7C15ull;
static uint64_t next_u64(void) {
    uint64_t z = (rng_state += 0x9E3779B97F4A7C15ull);
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    return z ^ (z >> 31);
}
static double unit01(void) { return (double)(next_u64() >> 11) / 9007199254740992.0; }
#endif
