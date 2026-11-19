#pragma once
#include <cstdint>

namespace fhr {

struct ControllerPresenceParams {
  double move_speed_m_s = 0.15;
  int64_t quiet_ns = 1'500'000'000;
  double hold_m = 0.25;
  double hold_release_m = 0.15;
};

class ControllerPresence {
 public:
  bool update(bool connected, bool tracked, double speed_m_s, double hand_distance_m, int64_t now_ns,
              const ControllerPresenceParams& p) {
    if (!connected) {
      *this = ControllerPresence();
      return false;
    }
    moving_ = tracked && speed_m_s > p.move_speed_m_s;
    const double hold = held_ ? p.hold_m + p.hold_release_m : p.hold_m;
    held_ = tracked && (hand_distance_m >= 0 ? hand_distance_m < hold : in_use_);
    if (moving_ || held_) {
      active_ns_ = now_ns;
      ever_active_ = true;
    }
    in_use_ = ever_active_ && now_ns - active_ns_ <= p.quiet_ns;
    return in_use_;
  }
  bool in_use() const { return in_use_; }
  bool moving() const { return moving_; }
  bool held() const { return held_; }
  void reset() { *this = ControllerPresence(); }

 private:
  int64_t active_ns_ = 0;
  bool ever_active_ = false, in_use_ = false, moving_ = false, held_ = false;
};

}  // namespace fhr
