#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <utility>

#include "hand_skeleton.h"

namespace fhr {

enum class HandPose { None, Point, System };
enum class PinchUse { None, Click, System, Ignored, NotPointing };

struct PoseParams {
  double system_enter_deg = 40, system_exit_deg = 60;
  double system_view_enter_deg = 45, system_view_exit_deg = 55;
  double system_open_curl = 0.8;
  double system_stay_curl = 0.7;
  double system_arm_s = 0.25;
  double system_hold_s = 5.0, system_tap_s = 0.2;
  double system_grace_s = 0.25;
  double point_min_palm_deg = 50;
  double point_min_distance_m = 0.2;
  double point_max_drop_m = 0.5;
  double point_max_down_deg = 40;
  double point_show_s = 0.1, point_hide_s = 0.35;
  double turn_ignore_deg = 25, turn_window_s = 0.3;
  double reset_gap_s = 0.3;
};

inline double palm_to_eyes_deg(const std::array<Xform, kXrJoints>& joints) {
  const Vec3 palm = joints[kPalm].p;
  const Vec3 normal = rotate(joints[kPalm].q, {0, -1, 0});
  const double c = dot(normal, normalized(palm * -1.0));
  return std::acos(std::clamp(c, -1.0, 1.0)) * 180 / M_PI;
}
inline double palm_off_view_deg(const std::array<Xform, kXrJoints>& joints) {
  const double c = dot(normalized(joints[kPalm].p), Vec3{0, 0, -1});
  return std::acos(std::clamp(c, -1.0, 1.0)) * 180 / M_PI;
}

class HandPoseTracker {
 public:
  void reset() { *this = HandPoseTracker{}; }

  HandPose update(const std::array<Xform, kXrJoints>& joints, bool pinch, int64_t t_ns, const PoseParams& p,
                  double ray_pitch_deg = 0) {
    if (seen_ && double(t_ns - last_ns_) * 1e-9 > p.reset_gap_s) reset();
    seen_ = true;
    last_ns_ = t_ns;
    const double to_eyes = palm_to_eyes_deg(joints), off_view = palm_off_view_deg(joints);
    const double curl = finger_curl(joints);
    const bool facing =
        facing_ ? to_eyes < p.system_exit_deg && off_view < p.system_view_exit_deg && curl > p.system_stay_curl
                : to_eyes < p.system_enter_deg && off_view < p.system_view_enter_deg && curl > p.system_open_curl;
    if (facing && !facing_) facing_since_ = t_ns;
    facing_ = facing;
    const Vec3 palm = joints[kPalm].p;
    const bool point_ok = !facing && to_eyes > p.point_min_palm_deg && norm(palm) > p.point_min_distance_m &&
                          palm.y > -p.point_max_drop_m && ray_pitch_deg > -p.point_max_down_deg;
    if (point_ok != point_ok_) point_changed_ = t_ns;
    point_ok_ = point_ok;
    angles_.emplace_back(t_ns, to_eyes);
    while (angles_.size() > 2 && double(t_ns - angles_[1].first) * 1e-9 >= p.turn_window_s) angles_.pop_front();
    const bool turning = to_eyes < 90 && angles_.front().second - to_eyes > p.turn_ignore_deg;

    if (pinch && !pinch_) {
      if (pose_ == HandPose::None && point_ok && !turning) pose_ = HandPose::Point;
      if (pose_ == HandPose::System)
        use_ = PinchUse::System;
      else if (turning)
        use_ = PinchUse::Ignored;
      else if (pose_ == HandPose::Point && to_eyes > p.point_min_palm_deg)
        use_ = PinchUse::Click;
      else
        use_ = PinchUse::NotPointing;
      if (use_ == PinchUse::System && !(hold_open_ && double(t_ns - released_ns_) * 1e-9 <= p.system_grace_s))
        hold_start_ns_ = t_ns;
      hold_open_ = false;
    }
    if (!pinch && pinch_) {
      hold_open_ = use_ == PinchUse::System && !fired_;
      released_ns_ = t_ns;
    }
    if (!pinch) {
      use_ = PinchUse::None;
      fired_ = false;
    }
    if (use_ == PinchUse::System && !fired_ && double(t_ns - hold_start_ns_) * 1e-9 >= p.system_hold_s) {
      fired_ = true;
      press_until_ns_ = t_ns + int64_t(p.system_tap_s * 1e9);
    }
    pressed_ = t_ns < press_until_ns_;
    progress_ = use_ != PinchUse::System ? 0
                : fired_                 ? 1
                                         : std::clamp(double(t_ns - hold_start_ns_) * 1e-9 / p.system_hold_s, 0.0, 1.0);
    pinch_ = pinch;
    ray_pitch_deg_ = ray_pitch_deg;
    palm_ = palm;
    if (pinch) return pose_;

    const double facing_s = facing ? double(t_ns - facing_since_) * 1e-9 : 0;
    const double steady_s = double(t_ns - point_changed_) * 1e-9;
    if (facing && facing_s >= p.system_arm_s) {
      pose_ = HandPose::System;
    } else if (pose_ == HandPose::System && !facing) {
      pose_ = HandPose::None;
    } else if (pose_ != HandPose::System) {
      if (point_ok && pose_ != HandPose::Point && steady_s >= p.point_show_s) pose_ = HandPose::Point;
      if (!point_ok && pose_ == HandPose::Point && steady_s >= p.point_hide_s) pose_ = HandPose::None;
    }
    return pose_;
  }
  HandPose pose() const { return pose_; }
  PinchUse pinch_use() const { return use_; }
  bool system_pressed() const { return pressed_; }
  bool system_fired() const { return fired_; }
  double system_progress() const { return progress_; }
  double ray_pitch_deg() const { return ray_pitch_deg_; }
  Vec3 palm() const { return palm_; }

 private:
  HandPose pose_ = HandPose::None;
  PinchUse use_ = PinchUse::None;
  bool seen_ = false, facing_ = false, point_ok_ = false, pinch_ = false;
  int64_t last_ns_ = 0, facing_since_ = 0, point_changed_ = 0;
  bool hold_open_ = false, fired_ = false, pressed_ = false;
  int64_t hold_start_ns_ = 0, released_ns_ = 0, press_until_ns_ = 0;
  double progress_ = 0;
  std::deque<std::pair<int64_t, double>> angles_;
  double ray_pitch_deg_ = 0;
  Vec3 palm_;
};

struct IntentParams {
  double enter_deg = 30, exit_deg = 45;
  double show_s = 0.15, hide_s = 0.6;
};
class RayIntent {
 public:
  void reset() { *this = RayIntent{}; }
  bool update(bool pointing, double look_deg, bool pinch, int64_t t_ns, const IntentParams& p) {
    const bool aligned = look_deg >= 0 && look_deg < (shown_ ? p.exit_deg : p.enter_deg);
    const bool want = pointing && aligned;
    if (want != want_) {
      want_ = want;
      changed_ns_ = t_ns;
    }
    const double steady_s = double(t_ns - changed_ns_) * 1e-9;
    if (!shown_ && want && (steady_s >= p.show_s || (pinch && !pinch_)))
      shown_ = true;
    else if (shown_ && !want && !pinch && steady_s >= p.hide_s)
      shown_ = false;
    pinch_ = pinch;
    look_deg_ = look_deg;
    return shown_;
  }
  bool shown() const { return shown_; }
  double look_deg() const { return look_deg_; }

 private:
  bool shown_ = false, want_ = false, pinch_ = false;
  int64_t changed_ns_ = 0;
  double look_deg_ = -1;
};

}  // namespace fhr
