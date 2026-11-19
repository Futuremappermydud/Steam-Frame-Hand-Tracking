#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

#include "hand_ray_model.h"

namespace fhr {

inline Vec3 rotation_vector(Quat q) {
  if (q.w < 0) q = {-q.w, -q.x, -q.y, -q.z};
  const Vec3 axis{q.x, q.y, q.z};
  const double s = norm(axis);
  if (s < 1e-12) return axis * 2.0;
  return axis * (2 * std::atan2(s, q.w) / s);
}
inline Quat from_rotation_vector(Vec3 v) {
  const double angle = norm(v);
  if (angle < 1e-12) return normalized(Quat{1, v.x / 2, v.y / 2, v.z / 2});
  const double s = std::sin(angle / 2) / angle;
  return {std::cos(angle / 2), v.x * s, v.y * s, v.z * s};
}

class RotationExtrapolator {
 public:
  void reset() {
    have_ = false;
    w_ = {};
  }
  void update(Quat q, int64_t t_ns, double smoothing) {
    if (have_ && t_ns > t_ns_) {
      const Vec3 w = rotation_vector(q * conjugate(q_)) * (1e9 / double(t_ns - t_ns_));
      w_ = w_ * smoothing + w * (1 - smoothing);
    } else {
      w_ = {};
    }
    q_ = q;
    t_ns_ = t_ns;
    have_ = true;
  }
  Quat ahead(double horizon_s) const { return have_ ? normalized(from_rotation_vector(w_ * horizon_s) * q_) : q_; }
  Vec3 velocity() const { return w_; }
  int64_t time_ns() const { return t_ns_; }
  bool valid() const { return have_; }

 private:
  bool have_ = false;
  Quat q_;
  Vec3 w_;
  int64_t t_ns_ = 0;
};

inline int xr_joint_parent(int joint) {
  if (joint == kWrist) return -1;
  if (joint == kPalm || joint == 2 || joint == 6 || joint == 11 || joint == 16 || joint == 21) return kWrist;
  return joint - 1;
}

struct FingerFilterParams {
  double min_cutoff_hz = 2;
  double beta = 3;
  double predict_max_s = 0.04;
  double velocity_smoothing = 0.5;
  double reset_gap_s = 0.25;
};

class FingerFilter {
 public:
  void reset() {
    for (int j = 0; j < kXrJoints; ++j) {
      rotation_[j].reset();
      offset_[j].reset();
      spin_[j].reset();
    }
    have_ = false;
  }
  void update(const std::array<Xform, kXrJoints>& joints, int64_t t_ns, const FingerFilterParams& p) {
    if (have_ && (t_ns <= t_ns_ || double(t_ns - t_ns_) * 1e-9 > p.reset_gap_s)) reset();
    const double dt = have_ ? double(t_ns - t_ns_) * 1e-9 : 0;
    for (int j = 0; j < kXrJoints; ++j) {
      const int parent = xr_joint_parent(j);
      const Xform local = parent < 0 ? joints[j] : inverse(joints[parent]) * joints[j];
      rotation_[j].set(p.min_cutoff_hz, p.beta);
      offset_[j].set(p.min_cutoff_hz, 0);
      local_[j] = {offset_[j].filter(local.p, dt), rotation_[j].filter(local.q, dt)};
      spin_[j].update(local_[j].q, t_ns, p.velocity_smoothing);
    }
    t_ns_ = t_ns;
    have_ = true;
  }
  std::array<Xform, kXrJoints> at(int64_t t_ns, const FingerFilterParams& p, double gain = 1) const {
    const double h = std::clamp(double(t_ns - t_ns_) * 1e-9, 0.0, p.predict_max_s) * gain;
    std::array<Xform, kXrJoints> out{};
    for (int k = 0; k < kXrJoints; ++k) {
      int j = k;
      if (k < 2) j = k == 0 ? kWrist : kPalm;
      const Xform local{local_[j].p, h > 0 ? spin_[j].ahead(h) : local_[j].q};
      const int parent = xr_joint_parent(j);
      out[j] = parent < 0 ? local : out[parent] * local;
    }
    return out;
  }
  bool valid() const { return have_; }
  int64_t time_ns() const { return t_ns_; }

 private:
  std::array<OneEuroRotation, kXrJoints> rotation_;
  std::array<OneEuro, kXrJoints> offset_;
  std::array<RotationExtrapolator, kXrJoints> spin_;
  std::array<Xform, kXrJoints> local_{};
  int64_t t_ns_ = 0;
  bool have_ = false;
};

}  // namespace fhr
