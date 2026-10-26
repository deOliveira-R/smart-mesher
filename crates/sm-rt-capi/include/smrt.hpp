// smrt.hpp: header-only C++17 RAII wrapper over smrt.h (stretch S2).
//
// Tracker and Ctx own their C handles (move-only) and turn negative status codes into
// smrt::Error carrying the code and the thread's last-error message. "No hit" (status 1) is not
// an error: the calls that can report it return the status as an int.
#ifndef SMRT_HPP
#define SMRT_HPP

#include <stdexcept>
#include <string>
#include <utility>

#include "smrt.h"

namespace smrt {

class Error : public std::runtime_error {
 public:
  Error(int32_t code, const std::string& what) : std::runtime_error(what), code_(code) {}
  int32_t code() const noexcept { return code_; }

 private:
  int32_t code_;
};

// Throws on a negative status; returns the status (0 or 1) otherwise.
inline int32_t check(int32_t status) {
  if (status < 0) {
    char buf[512] = {0};
    smrt_last_error_message(buf, sizeof buf);
    throw Error(status, buf[0] ? std::string(buf) : std::string("smrt error ") + std::to_string(status));
  }
  return status;
}

class Tracker {
 public:
  // Takes ownership of a raw handle (for example from smrt_open).
  static Tracker adopt(SmrtTracker* raw) { return Tracker(raw); }
  static Tracker open(const char* path) {
    SmrtTracker* t = nullptr;
    check(smrt_open(path, &t));
    return Tracker(t);
  }
  Tracker(const Tracker&) = delete;
  Tracker& operator=(const Tracker&) = delete;
  Tracker(Tracker&& o) noexcept : t_(std::exchange(o.t_, nullptr)) {}
  Tracker& operator=(Tracker&& o) noexcept {
    if (this != &o) {
      reset();
      t_ = std::exchange(o.t_, nullptr);
    }
    return *this;
  }
  ~Tracker() { reset(); }

  SmrtTracker* get() const noexcept { return t_; }
  SmrtInfo info() const {
    SmrtInfo i;
    check(smrt_info(t_, &i));
    return i;
  }
  SmrtRegionInfo region_info(int64_t region) const {
    SmrtRegionInfo r;
    check(smrt_region_info(t_, region, &r));
    return r;
  }

 private:
  explicit Tracker(SmrtTracker* t) : t_(t) {}
  void reset() noexcept {
    if (t_) smrt_tracker_free(t_);
    t_ = nullptr;
  }
  SmrtTracker* t_ = nullptr;
};

// One per thread; keeps its tracker alive.
class Ctx {
 public:
  explicit Ctx(const Tracker& t) { check(smrt_ctx_new(t.get(), &c_)); }
  Ctx(const Ctx&) = delete;
  Ctx& operator=(const Ctx&) = delete;
  Ctx(Ctx&& o) noexcept : c_(std::exchange(o.c_, nullptr)) {}
  Ctx& operator=(Ctx&& o) noexcept {
    if (this != &o) {
      reset();
      c_ = std::exchange(o.c_, nullptr);
    }
    return *this;
  }
  ~Ctx() { reset(); }

  // True when the point is inside the model.
  bool init(const double p[3], const double d[3], int64_t hint = SMRT_NONE) {
    return check(smrt_init_state(c_, p[0], p[1], p[2], d[0], d[1], d[2], hint)) == 0;
  }
  SmrtState state() const {
    SmrtState s;
    check(smrt_get_state(c_, &s));
    return s;
  }
  // The status is 0 for an event and 1 when the limit was reached first.
  int32_t step(double max_dist, SmrtStep& out) { return check(smrt_step(c_, max_dist, &out)); }
  int32_t peek(double limit, SmrtStep& out) { return check(smrt_peek(c_, limit, &out)); }
  int32_t fire(double limit, SmrtStep& out, int64_t max_steps = 0) { return check(smrt_fire(c_, limit, max_steps, &out)); }
  // True if the particle continues (out_dir holds its new direction), false if it is terminated.
  bool resolve_boundary(int64_t face, int32_t kind, double albedo, double u0, double u1, double out_dir[3]) {
    return check(smrt_resolve_boundary(c_, face, kind, albedo, u0, u1, out_dir)) == 0;
  }
  SmrtCtx* get() const noexcept { return c_; }

 private:
  void reset() noexcept {
    if (c_) smrt_ctx_free(c_);
    c_ = nullptr;
  }
  SmrtCtx* c_ = nullptr;
};

}  // namespace smrt

#endif  // SMRT_HPP
