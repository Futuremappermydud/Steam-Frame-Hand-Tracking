#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>

namespace fhr {

struct Vec3 {
  double x = 0, y = 0, z = 0;
};
inline Vec3 operator+(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3 operator-(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Vec3 operator*(Vec3 a, double s) { return {a.x * s, a.y * s, a.z * s}; }
inline double dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 cross(Vec3 a, Vec3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
inline double norm(Vec3 a) { return std::sqrt(dot(a, a)); }
inline Vec3 normalized(Vec3 a) {
  const double n = norm(a);
  return n > 1e-12 ? a * (1 / n) : Vec3{0, 0, -1};
}

struct Quat {
  double w = 1, x = 0, y = 0, z = 0;
};
inline Quat operator*(Quat a, Quat b) {
  return {a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z, a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
          a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x, a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}
inline Quat normalized(Quat q) {
  const double n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  return n > 1e-12 ? Quat{q.w / n, q.x / n, q.y / n, q.z / n} : Quat{};
}
inline Vec3 rotate(Quat q, Vec3 v) {
  const Vec3 u{q.x, q.y, q.z};
  const Vec3 t = cross(u, v) * 2.0;
  return v + t * q.w + cross(u, t);
}
inline Quat slerp(Quat a, Quat b, double t) {
  double c = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
  if (c < 0) {
    b = {-b.w, -b.x, -b.y, -b.z};
    c = -c;
  }
  if (c > 0.9995)
    return normalized(Quat{a.w + (b.w - a.w) * t, a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t});
  const double th = std::acos(c), s = std::sin(th), wa = std::sin((1 - t) * th) / s, wb = std::sin(t * th) / s;
  return {a.w * wa + b.w * wb, a.x * wa + b.x * wb, a.y * wa + b.y * wb, a.z * wa + b.z * wb};
}
inline Quat quat_from_matrix(const double m[3][3]) {
  Quat q;
  const double tr = m[0][0] + m[1][1] + m[2][2];
  if (tr > 0) {
    const double s = std::sqrt(tr + 1.0) * 2;
    q = {0.25 * s, (m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s};
  } else if (m[0][0] > m[1][1] && m[0][0] > m[2][2]) {
    const double s = std::sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2;
    q = {(m[2][1] - m[1][2]) / s, 0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s};
  } else if (m[1][1] > m[2][2]) {
    const double s = std::sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2;
    q = {(m[0][2] - m[2][0]) / s, (m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s};
  } else {
    const double s = std::sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2;
    q = {(m[1][0] - m[0][1]) / s, (m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s};
  }
  return normalized(q);
}

struct Pose {
  Vec3 position;
  Quat rotation;
};

class PoseHistory {
 public:
  explicit PoseHistory(int64_t keep_ns = 1'000'000'000) : keep_ns_(keep_ns) {}
  void push(int64_t t_ns, const Pose& pose) {
    if (!samples_.empty() && t_ns <= samples_.back().first) return;
    samples_.emplace_back(t_ns, pose);
    while (samples_.size() > 2 && samples_.back().first - samples_.front().first > keep_ns_) samples_.pop_front();
  }
  bool empty() const { return samples_.empty(); }
  Pose at(int64_t t_ns) const {
    if (samples_.empty()) return {};
    if (t_ns <= samples_.front().first) return samples_.front().second;
    if (t_ns >= samples_.back().first) return samples_.back().second;
    auto hi = std::lower_bound(samples_.begin(), samples_.end(), t_ns,
                               [](const std::pair<int64_t, Pose>& s, int64_t t) { return s.first < t; });
    auto lo = hi - 1;
    const double f = double(t_ns - lo->first) / double(hi->first - lo->first);
    return {lo->second.position + (hi->second.position - lo->second.position) * f,
            slerp(lo->second.rotation, hi->second.rotation, f)};
  }

 private:
  int64_t keep_ns_;
  std::deque<std::pair<int64_t, Pose>> samples_;
};

class OneEuro {
 public:
  OneEuro(double min_cutoff_hz = 1.0, double beta = 0.5, double d_cutoff_hz = 1.0)
      : min_cutoff_(min_cutoff_hz), beta_(beta), d_cutoff_(d_cutoff_hz) {}
  void reset() { initialised_ = false; }
  void set(double min_cutoff_hz, double beta) {
    min_cutoff_ = min_cutoff_hz;
    beta_ = beta;
  }
  Vec3 filter(Vec3 x, double dt) {
    if (!initialised_ || !(dt > 0)) {
      prev_ = x;
      dprev_ = {};
      initialised_ = true;
      return x;
    }
    const Vec3 dx = (x - prev_) * (1 / dt);
    dprev_ = lerp(dprev_, dx, alpha(d_cutoff_, dt));
    const double cutoff = min_cutoff_ + beta_ * norm(dprev_);
    prev_ = lerp(prev_, x, alpha(cutoff, dt));
    return prev_;
  }

 private:
  static double alpha(double cutoff, double dt) {
    const double tau = 1 / (2 * M_PI * cutoff);
    return 1 / (1 + tau / dt);
  }
  static Vec3 lerp(Vec3 a, Vec3 b, double t) { return a + (b - a) * t; }
  double min_cutoff_, beta_, d_cutoff_;
  bool initialised_ = false;
  Vec3 prev_, dprev_;
};

class OneEuroRotation {
 public:
  OneEuroRotation(double min_cutoff_hz = 1.5, double beta = 0.5, double d_cutoff_hz = 1.0)
      : min_cutoff_(min_cutoff_hz), beta_(beta), d_cutoff_(d_cutoff_hz) {}
  void reset() { initialised_ = false; }
  void set(double min_cutoff_hz, double beta) {
    min_cutoff_ = min_cutoff_hz;
    beta_ = beta;
  }
  Quat filter(Quat q, double dt) {
    if (!initialised_ || !(dt > 0)) {
      prev_ = q;
      speed_ = 0;
      initialised_ = true;
      return q;
    }
    double c = prev_.w * q.w + prev_.x * q.x + prev_.y * q.y + prev_.z * q.z;
    if (c < 0) {
      q = {-q.w, -q.x, -q.y, -q.z};
      c = -c;
    }
    const double angle = 2 * std::acos(std::min(1.0, c));
    speed_ += (angle / dt - speed_) * alpha(d_cutoff_, dt);
    prev_ = slerp(prev_, q, alpha(min_cutoff_ + beta_ * speed_, dt));
    return prev_;
  }

 private:
  static double alpha(double cutoff, double dt) {
    const double tau = 1 / (2 * M_PI * cutoff);
    return 1 / (1 + tau / dt);
  }
  double min_cutoff_, beta_, d_cutoff_;
  bool initialised_ = false;
  Quat prev_;
  double speed_ = 0;
};

class Extrapolator {
 public:
  void reset() {
    have_ = false;
    v_ = {};
  }
  void update(Vec3 x, int64_t t_ns, double smoothing) {
    if (have_ && t_ns > t_ns_) {
      const Vec3 v = (x - x_) * (1e9 / double(t_ns - t_ns_));
      v_ = v_ * smoothing + v * (1 - smoothing);
    } else {
      v_ = {};
    }
    x_ = x;
    t_ns_ = t_ns;
    have_ = true;
  }
  Vec3 ahead(double horizon_s) const { return have_ ? x_ + v_ * horizon_s : x_; }
  Vec3 value() const { return x_; }
  Vec3 velocity() const { return v_; }
  int64_t time_ns() const { return t_ns_; }
  bool valid() const { return have_; }

 private:
  bool have_ = false;
  Vec3 x_, v_;
  int64_t t_ns_ = 0;
};

struct Xform {
  Vec3 p;
  Quat q;
};
inline Quat conjugate(Quat q) { return {q.w, -q.x, -q.y, -q.z}; }
inline Xform operator*(const Xform& a, const Xform& b) { return {a.p + rotate(a.q, b.p), normalized(a.q * b.q)}; }
inline Xform inverse(const Xform& a) {
  const Quat c = conjugate(a.q);
  return {rotate(c, a.p * -1.0), c};
}

constexpr int kXrJoints = 26;
enum : int {
  kPalm = 0,
  kWrist = 1,
  kThumbTip = 5,
  kIndexTip = 10,
  kMiddleProximal = 12,
  kMiddleTip = 15,
  kRingTip = 20,
  kLittleTip = 25
};

constexpr char kMagic[4] = {'F', 'H', 'R', '1'};
struct HandSample {
  bool valid = false;
  bool pinch = false;
  float pinch_distance_m = -1;
  Vec3 anchor_head;
  Vec3 wrist_head;
  bool has_joints = false;
  std::array<Xform, kXrJoints> joints_head;
  float confidence = -1;
  std::array<float, 5> finger_confidence{-1, -1, -1, -1, -1};
};
struct Packet {
  int64_t capture_raw_ns = 0;
  int64_t send_raw_ns = 0;
  uint32_t sequence = 0;
  std::array<HandSample, 2> hands;
};
constexpr size_t kHandBytes = 4 + 4 + 12 + 12;
constexpr size_t kPacketBytesV1 = 4 + 4 + 8 + 8 + 4 + 2 * kHandBytes;
constexpr size_t kJointBytes = 4 * (3 + 4);
constexpr size_t kSkeletonBytes = 4 + kXrJoints * kJointBytes;
constexpr size_t kPacketBytesV2 = kPacketBytesV1 + 2 * kSkeletonBytes;
constexpr size_t kQualityBytes = 4 * 6;
constexpr size_t kPacketBytes = kPacketBytesV2 + 2 * kQualityBytes;

inline double confidence_trust(float confidence, double low, double high) {
  if (!(confidence >= 0)) return 1;
  if (!(high > low)) return confidence >= high ? 1 : 0;
  return std::clamp((confidence - low) / (high - low), 0.0, 1.0);
}

inline bool decode(const uint8_t* data, size_t size, Packet& out) {
  if (size < 8 || std::memcmp(data, kMagic, 4) != 0) return false;
  uint32_t version;
  std::memcpy(&version, data + 4, 4);
  if (!((version == 1 && size == kPacketBytesV1) || (version == 2 && size == kPacketBytesV2) ||
        (version == 3 && size == kPacketBytes)))
    return false;
  std::memcpy(&out.capture_raw_ns, data + 8, 8);
  std::memcpy(&out.send_raw_ns, data + 16, 8);
  std::memcpy(&out.sequence, data + 24, 4);
  const uint8_t* p = data + 28;
  for (auto& h : out.hands) {
    h.valid = p[0] != 0;
    h.pinch = p[1] != 0;
    std::memcpy(&h.pinch_distance_m, p + 4, 4);
    float v[6];
    std::memcpy(v, p + 8, 24);
    h.anchor_head = {v[0], v[1], v[2]};
    h.wrist_head = {v[3], v[4], v[5]};
    for (float f : v)
      if (!std::isfinite(f)) h.valid = false;
    if (!std::isfinite(h.pinch_distance_m)) h.valid = false;
    h.has_joints = false;
    h.confidence = -1;
    h.finger_confidence.fill(-1);
    p += kHandBytes;
  }
  if (version == 1) return true;
  for (auto& h : out.hands) {
    h.has_joints = h.valid && p[0] != 0;
    const uint8_t* j = p + 4;
    for (auto& joint : h.joints_head) {
      float f[7];
      std::memcpy(f, j, sizeof f);
      for (float x : f)
        if (!std::isfinite(x)) h.has_joints = false;
      joint.p = {f[0], f[1], f[2]};
      const Quat q{f[6], f[3], f[4], f[5]};
      const double n2 = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
      if (!(n2 > 0.9 && n2 < 1.1)) h.has_joints = false;
      joint.q = normalized(q);
      j += kJointBytes;
    }
    p += kSkeletonBytes;
  }
  if (version == 2) return true;
  for (auto& h : out.hands) {
    float q[6];
    std::memcpy(q, p, sizeof q);
    const auto value = [](float v) { return std::isfinite(v) && v >= 0 ? std::min(v, 1.0f) : -1.0f; };
    h.confidence = value(q[0]);
    for (int i = 0; i < 5; ++i) h.finger_confidence[i] = value(q[1 + i]);
    p += kQualityBytes;
  }
  return true;
}

struct RayParams {
  double shoulder_width_m = 0.17;
  double shoulder_drop_m = 0.20;
  double shoulder_back_m = 0.06;
};

inline Vec3 shoulder_world(const Pose& head, bool right, const RayParams& p) {
  const Vec3 gaze = rotate(head.rotation, {0, 0, -1});
  Vec3 forward{gaze.x, 0, gaze.z};
  if (norm(forward) < 0.05) {
    const Vec3 head_up = rotate(head.rotation, {0, 1, 0}) * (gaze.y < 0 ? 1.0 : -1.0);
    forward = {head_up.x, 0, head_up.z};
  }
  forward = normalized(forward);
  const Vec3 up{0, 1, 0};
  const Vec3 side = normalized(cross(forward, up));
  return head.position + side * (right ? p.shoulder_width_m : -p.shoulder_width_m) + up * -p.shoulder_drop_m +
         forward * -p.shoulder_back_m;
}

inline Quat look_rotation(Vec3 dir) {
  const Vec3 back = normalized(dir) * -1;
  Vec3 up_hint{0, 1, 0};
  if (std::abs(dot(back, up_hint)) > 0.99) up_hint = {0, 0, 1};
  const Vec3 x = normalized(cross(up_hint, back));
  const Vec3 y = cross(back, x);
  const double m[3][3] = {{x.x, y.x, back.x}, {x.y, y.y, back.y}, {x.z, y.z, back.z}};
  return quat_from_matrix(m);
}

struct Ray {
  Vec3 origin, direction;
};
inline Ray hand_ray(const Pose& head_at_capture, const HandSample& h, bool right, const RayParams& p) {
  const Vec3 anchor = head_at_capture.position + rotate(head_at_capture.rotation, h.anchor_head);
  const Vec3 shoulder = shoulder_world(head_at_capture, right, p);
  return {anchor, normalized(anchor - shoulder)};
}

struct SelectParams {
  int64_t switch_lost_ns = 300'000'000;
  int64_t forget_ns = 1'000'000'000;
  double raise_m = 0.06;
  int raise_frames = 8;
  int64_t min_hold_ns = 400'000'000;
};
class RaySelector {
 public:
  explicit RaySelector(SelectParams p = {}) : p_(p) {}
  int update(const Packet& packet, int64_t now_ns, const bool* ready = nullptr) {
    const auto& h = packet.hands;
    bool valid[2];
    for (int i = 0; i < 2; ++i) {
      valid[i] = h[i].valid && (ready == nullptr || ready[i]);
      if (valid[i]) seen_ns_[i] = now_ns;
    }
    if (active_ >= 0 && !valid[active_] && now_ns - seen_ns_[active_] > p_.forget_ns) active_ = -1;
    if (active_ < 0) {
      if (valid[0] && valid[1])
        active_ = h[0].anchor_head.y > h[1].anchor_head.y ? 0 : 1;
      else if (valid[0] || valid[1])
        active_ = valid[1] ? 1 : 0;
      raise_streak_ = 0;
      other_pinch_ = active_ >= 0 && valid[1 - active_] && h[1 - active_].pinch;
      return active_;
    }
    const int other = 1 - active_;
    const bool held = now_ns - switched_ns_ < p_.min_hold_ns;
    bool take = false;
    if (valid[other]) {
      if (!valid[active_]) {
        take = (ready != nullptr && h[active_].valid) || now_ns - seen_ns_[active_] > p_.switch_lost_ns;
      } else {
        if (h[other].pinch && !other_pinch_ && !h[active_].pinch) take = true;
        raise_streak_ = h[other].anchor_head.y - h[active_].anchor_head.y > p_.raise_m ? raise_streak_ + 1 : 0;
        if (raise_streak_ >= p_.raise_frames && !held && !h[active_].pinch) take = true;
      }
    } else {
      raise_streak_ = 0;
    }
    other_pinch_ = valid[other] && h[other].pinch;
    if (take) {
      active_ = other;
      raise_streak_ = 0;
      other_pinch_ = valid[1 - active_] && h[1 - active_].pinch;
      switched_ns_ = now_ns;
      ++switches_;
    }
    return active_;
  }
  int active() const { return active_; }
  int switches() const { return switches_; }

 private:
  SelectParams p_;
  int active_ = -1, raise_streak_ = 0, switches_ = 0;
  bool other_pinch_ = false;
  int64_t seen_ns_[2] = {0, 0};
  int64_t switched_ns_ = INT64_MIN / 2;
};

}  // namespace fhr
