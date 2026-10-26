// Stretch S2: the C++ RAII wrapper over the stub sphere (R = 5).
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>

#include "smrt.hpp"
extern "C" {
#include "smrt_test.h"
}

#define REQUIRE(c) do { if (!(c)) { std::fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #c); std::exit(1); } } while (0)

int main() {
  SmrtTracker* raw = nullptr;
  REQUIRE(smrt_test_stub(STUB_SPHERE, 5.0, &raw) == 0);
  smrt::Tracker t = smrt::Tracker::adopt(raw);
  smrt::Tracker moved = std::move(t);  // move-only: the source no longer owns anything
  REQUIRE(t.get() == nullptr && moved.get() != nullptr);
  REQUIRE(moved.info().cells == 2);

  smrt::Ctx c(moved);
  const double p[3] = {-9.0, 3.0, 0.0}, d[3] = {1.0, 0.0, 0.0};
  REQUIRE(c.init(p, d));
  SmrtStep s;
  REQUIRE(c.fire(std::numeric_limits<double>::infinity(), s) == 0 && s.to_region == 0);
  REQUIRE(c.fire(std::numeric_limits<double>::infinity(), s) == 0);
  REQUIRE(std::fabs(s.dist - 8.0) < 1e-12);  // chord at b = 3 in R = 5: 2 * sqrt(25 - 9)
  REQUIRE(s.to_region == SMRT_COMPLEMENT);

  // Errors become exceptions with the library's message.
  bool threw = false;
  try {
    const double bad[3] = {0.0, 0.0, 0.0};
    c.init(p, bad);
  } catch (const smrt::Error& e) {
    threw = e.code() == SMRT_ERR_BAD_DIRECTION && std::string(e.what()).find("direction") != std::string::npos;
  }
  REQUIRE(threw);
  try {
    smrt::Tracker::open("model.smrt");
    REQUIRE(false);
  } catch (const smrt::Error& e) {
    REQUIRE(e.code() == SMRT_ERR_UNSUPPORTED);
  }
  std::puts("raii: ok");
  return 0;
}
