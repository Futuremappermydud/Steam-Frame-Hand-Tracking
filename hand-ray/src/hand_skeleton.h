#pragma once
#include "hand_ray_model.h"

namespace fhr {

constexpr int kBones = 31;

inline Quat axes_quat(Vec3 x, Vec3 y, Vec3 z) {
  const double m[3][3] = {{x.x, y.x, z.x}, {x.y, y.y, z.y}, {x.z, y.z, z.z}};
  return quat_from_matrix(m);
}
struct BoneAxes {
  Quat wrist, finger;
};
inline BoneAxes openvr_bone_axes(bool right) {
  if (right) return {axes_quat({0, -1, 0}, {-1, 0, 0}, {0, 0, -1}), axes_quat({0, 0, 1}, {0, 1, 0}, {-1, 0, 0})};
  return {axes_quat({0, 1, 0}, {1, 0, 0}, {0, 0, -1}), axes_quat({0, 0, -1}, {0, -1, 0}, {-1, 0, 0})};
}

inline int bone_parent(int bone) {
  if (bone <= 1) return 0;
  if (bone == 2 || bone == 6 || bone == 11 || bone == 16 || bone == 21) return 1;
  return bone - 1;
}

inline void openvr_bones(const std::array<Xform, kXrJoints>& joints, bool right, std::array<Xform, kBones>& out) {
  const BoneAxes axes = openvr_bone_axes(right);
  std::array<Xform, kXrJoints> model;
  for (int i = 1; i < kXrJoints; ++i)
    model[i] = {joints[i].p, normalized(joints[i].q * (i == kWrist ? axes.wrist : axes.finger))};
  out[0] = {};
  out[1] = model[1];
  for (int i = 2; i < kXrJoints; ++i) out[i] = inverse(model[bone_parent(i)]) * model[i];
  const int aux_source[5] = {4, 9, 14, 19, 24};
  for (int k = 0; k < 5; ++k) out[26 + k] = model[aux_source[k]];
}

inline Xform index_wrist_in_raw(bool right) {
  const Xform left{{-0.034038, 0.036503, 0.164722}, normalized(Quat{-0.055147, -0.078608, -0.920279, 0.379296})};
  return right ? Xform{{-left.p.x, left.p.y, left.p.z}, {left.q.w, left.q.x, -left.q.y, -left.q.z}} : left;
}
inline Xform hand_device_pose(const Xform& xr_wrist, bool right) {
  const Xform wrist{xr_wrist.p, normalized(xr_wrist.q * openvr_bone_axes(right).wrist)};
  return wrist * inverse(index_wrist_in_raw(right));
}

template <typename Joints>
inline double finger_curl(const Joints& j) {
  const double palm = norm(j[kMiddleProximal].p - j[kWrist].p);
  if (!(palm > 1e-4)) return 1;
  return (norm(j[kMiddleTip].p - j[kPalm].p) + norm(j[kRingTip].p - j[kPalm].p) + norm(j[kLittleTip].p - j[kPalm].p)) /
         (3 * palm);
}
inline double ramp(double v, double zero_at, double one_at) {
  return std::clamp((v - zero_at) / (one_at - zero_at), 0.0, 1.0);
}

class Hysteresis {
 public:
  bool update(double v, double begin, double end) {
    if (!on_ && v < begin)
      on_ = true;
    else if (on_ && v > end)
      on_ = false;
    return on_;
  }
  void reset() { on_ = false; }
  bool on() const { return on_; }

 private:
  bool on_ = false;
};

}  // namespace fhr
