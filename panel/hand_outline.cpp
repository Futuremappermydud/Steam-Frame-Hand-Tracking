#include "hand_outline.h"

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <ctime>

namespace {

struct V {
  double x, y, z;
};
V operator+(V a, V b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
V operator-(V a, V b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
V operator*(V a, double s) { return {a.x * s, a.y * s, a.z * s}; }
double dot(V a, V b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
V cross(V a, V b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
V unit(V a) {
  const double n = std::sqrt(dot(a, a));
  return n > 1e-9 ? a * (1 / n) : V{0, 0, 1};
}

constexpr double kWidthM = 0.36;
constexpr double kLinePer256 = 3.0;
constexpr double kForearmM = 0.11, kWristRadiusM = 0.024;
constexpr double kAtWrist = 0.7;
constexpr double kMaskAlpha = 0.75;
constexpr int kFingers[5][5] = {
    {1, 2, 3, 4, 5}, {6, 7, 8, 9, 10}, {11, 12, 13, 14, 15}, {16, 17, 18, 19, 20}, {21, 22, 23, 24, 25}};
constexpr double kFingerRadiusM[5] = {0.010, 0.008, 0.008, 0.008, 0.008};
constexpr int kPalm[] = {1, 2, 3, 7, 12, 17, 22, 21};
constexpr int kPalmPoints = int(sizeof kPalm / sizeof kPalm[0]);
constexpr int kTipZero = 2, kTipFull = 3, kTip = 4;

doodle::Color hue_mix(const doodle::Color& a, const doodle::Color& b, double t) {
  struct Hsv {
    double h, s, v;
  };
  const auto hsv = [](const doodle::Color& c) {
    const double hi = std::max({c.r, c.g, c.b}), lo = std::min({c.r, c.g, c.b}), range = hi - lo;
    double h = 0;
    if (range > 0) {
      h = hi == c.r   ? std::fmod((c.g - c.b) / range + 6, 6.0)
          : hi == c.g ? (c.b - c.r) / range + 2
                      : (c.r - c.g) / range + 4;
    }
    return Hsv{h * 60, hi > 0 ? range / hi : 0, hi};
  };
  const Hsv x = hsv(a), y = hsv(b);
  const double turn = std::fmod(y.h - x.h + 540, 360.0) - 180;
  const double h = std::fmod(x.h + turn * t + 360, 360.0) / 60, s = x.s + (y.s - x.s) * t, v = x.v + (y.v - x.v) * t;
  const double c = v * s, m = v - c, k = c * (1 - std::fabs(std::fmod(h, 2.0) - 1));
  const int sector = int(h) % 6;
  const double rgb[6][3] = {{c, k, 0}, {k, c, 0}, {0, c, k}, {0, k, c}, {k, 0, c}, {c, 0, k}};
  return {rgb[sector][0] + m, rgb[sector][1] + m, rgb[sector][2] + m, a.a + (b.a - a.a) * t};
}

doodle::Color ink_colour(uint8_t flags, uint8_t hold) {
  doodle::Color ink = doodle::kChalk;
  ink.a = (flags & fho::kLaser) ? 0.9 : 0.7;
  if (flags & fho::kSystemDown) {
    ink = doodle::kOrange;
  } else if (flags & fho::kSystemArmed) {
    ink = hold > 0 ? hue_mix(doodle::kBlue, doodle::kOrange, hold / 255.0) : doodle::kBlue;
  }
  return ink;
}

struct Fade {
  double from_u, from_v, to_u, to_v, wrist;
};
Fade fade_along(const OutlineLayout& layout) {
  const OutlineLayout::Point* pts = layout.joints;
  const OutlineLayout::Point& arm = layout.arm;
  double tip_u = 0, tip_v = 0;
  for (int tip : {10, 15, 20, 25}) {
    tip_u += pts[tip].u / 4;
    tip_v += pts[tip].v / 4;
  }
  const double ax = tip_u - arm.u, ay = tip_v - arm.v, length2 = std::max(ax * ax + ay * ay, 1.0);
  const double wrist = std::clamp(((pts[1].u - arm.u) * ax + (pts[1].v - arm.v) * ay) / length2, 0.05, 0.95);
  return {arm.u, arm.v, tip_u, tip_v, wrist};
}

double finger_radius_px(const OutlineLayout& layout, int finger) {
  double scale = 0;
  for (int joint : kFingers[finger])
    if (!(finger == 0 && joint == 1)) scale = std::max(scale, layout.joints[joint].scale);
  return kFingerRadiusM[finger] * scale;
}

void tip_stroke(cairo_t* cr, const OutlineLayout& layout, int f, double line) {
  const OutlineLayout::Point* pts = layout.joints;
  cairo_move_to(cr, pts[kFingers[f][kTipZero]].u, pts[kFingers[f][kTipZero]].v);
  cairo_line_to(cr, pts[kFingers[f][kTipFull]].u, pts[kFingers[f][kTipFull]].v);
  cairo_line_to(cr, pts[kFingers[f][kTip]].u, pts[kFingers[f][kTip]].v);
  cairo_set_line_width(cr, 2 * (finger_radius_px(layout, f) + line + 1));
  cairo_stroke(cr);
}

cairo_pattern_t* finger_gradient(const OutlineLayout::Point& a, const OutlineLayout::Point& b) {
  if (std::hypot(b.u - a.u, b.v - a.v) <= 0.5) return nullptr;
  cairo_pattern_t* along = cairo_pattern_create_linear(a.u, a.v, b.u, b.v);
  cairo_pattern_set_extend(along, CAIRO_EXTEND_PAD);
  return along;
}

void paint_tips(cairo_t* cr, const OutlineLayout& layout, double line) {
  cairo_set_line_cap(cr, CAIRO_LINE_CAP_ROUND);
  cairo_set_line_join(cr, CAIRO_LINE_JOIN_ROUND);
  for (int f = 0; f < 5; ++f) {
    cairo_pattern_t* along =
        finger_gradient(layout.joints[kFingers[f][kTipZero]], layout.joints[kFingers[f][kTipFull]]);
    if (along) {
      cairo_pattern_add_color_stop_rgba(along, 0, 0, 0, 0, 0);
      cairo_pattern_add_color_stop_rgba(along, 1, 0, 0, 0, 1);
      cairo_set_source(cr, along);
      cairo_pattern_destroy(along);
    } else {
      cairo_set_source_rgba(cr, 0, 0, 0, 1);
    }
    tip_stroke(cr, layout, f, line);
  }
}
}  // namespace

int64_t outline_now_ns() {
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
  return int64_t(ts.tv_sec) * 1'000'000'000 + ts.tv_nsec;
}

OutlineReceiver::~OutlineReceiver() { stop(); }

bool OutlineReceiver::start() {
  if (fd_ >= 0) return true;
  fd_ = socket(AF_UNIX, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
  if (fd_ < 0) return false;
  sockaddr_un addr{};
  addr.sun_family = AF_UNIX;
  std::memcpy(addr.sun_path + 1, fho::kSocketName, sizeof(fho::kSocketName) - 1);
  const socklen_t len = socklen_t(offsetof(sockaddr_un, sun_path) + sizeof(fho::kSocketName));
  const int one = 1;
  setsockopt(fd_, SOL_SOCKET, SO_PASSCRED, &one, sizeof one);
  if (bind(fd_, reinterpret_cast<const sockaddr*>(&addr), len) != 0) {
    close(fd_);
    fd_ = -1;
    return false;
  }
  return true;
}

void OutlineReceiver::stop() {
  if (fd_ >= 0) close(fd_);
  fd_ = -1;
}

bool OutlineReceiver::poll(fho::Packet& newest, int64_t& received_ns) {
  bool got = false;
  const uid_t me = getuid();
  for (;;) {
    uint8_t buf[fho::kBytes + 16];
    alignas(cmsghdr) char control[CMSG_SPACE(sizeof(ucred))];
    iovec iov{buf, sizeof buf};
    msghdr msg{};
    msg.msg_iov = &iov;
    msg.msg_iovlen = 1;
    msg.msg_control = control;
    msg.msg_controllen = sizeof control;
    const ssize_t n = fd_ >= 0 ? recvmsg(fd_, &msg, MSG_DONTWAIT) : -1;
    if (n <= 0) break;
    bool same_user = false;
    for (cmsghdr* c = CMSG_FIRSTHDR(&msg); c; c = CMSG_NXTHDR(&msg, c)) {
      if (c->cmsg_level == SOL_SOCKET && c->cmsg_type == SCM_CREDENTIALS) {
        ucred cred;
        std::memcpy(&cred, CMSG_DATA(c), sizeof cred);
        same_user = cred.uid == me;
      }
    }
    fho::Packet p;
    if (!same_user || !fho::decode(buf, size_t(n), p)) continue;
    newest = p;
    received_ns = outline_now_ns();
    got = true;
  }
  return got;
}

OutlineLayout layout_hand_outline(const fho::Hand& hand, const float eye_f[3], int size_px) {
  V joints[fho::kJoints] = {};
  for (int j = 0; j < fho::kJoints; ++j) joints[j] = {hand.joints[j][0], hand.joints[j][1], hand.joints[j][2]};
  const V eye{eye_f[0], eye_f[1], eye_f[2]}, centre = joints[0];
  const V z = unit(eye - centre);
  const V x = unit(cross(V{0, 1, 0}, z)), y = cross(z, x);
  const double size = size_px, px_per_m = size / kWidthM;
  const auto project = [&](V p) {
    const V d = p - eye;
    const double along = dot(d, z * -1.0), to_plane = dot(centre - eye, z * -1.0);
    const double t = along > 1e-3 ? to_plane / along : 1;
    const V on_plane = eye + d * t - centre;
    return OutlineLayout::Point{(dot(on_plane, x) / kWidthM + 0.5) * size, (0.5 - dot(on_plane, y) / kWidthM) * size,
                                t * px_per_m};
  };
  OutlineLayout out{};
  out.size = size_px;
  for (int j = 0; j < fho::kJoints; ++j) out.joints[j] = project(joints[j]);
  out.arm = project(joints[1] + unit(joints[1] - joints[12]) * kForearmM);
  const V axes[4] = {x, y, z, centre};
  for (int c = 0; c < 4; ++c) {
    out.placement.pose[0][c] = float(axes[c].x);
    out.placement.pose[1][c] = float(axes[c].y);
    out.placement.pose[2][c] = float(axes[c].z);
  }
  out.placement.width_m = float(kWidthM);
  return out;
}

OutlineLayout layout_hand_outline_eye(const fho::Hand& hand, const float eye_pose[3][4], int size_px) {
  const V x{eye_pose[0][0], eye_pose[1][0], eye_pose[2][0]}, y{eye_pose[0][1], eye_pose[1][1], eye_pose[2][1]};
  const V z{eye_pose[0][2], eye_pose[1][2], eye_pose[2][2]}, eye{eye_pose[0][3], eye_pose[1][3], eye_pose[2][3]};
  struct Seen {
    double right, up, ahead;
  };
  const auto see = [&](V p) {
    const V d = p - eye;
    const double ahead = std::max(dot(d, z * -1.0), 0.05);
    return Seen{dot(d, x) / ahead, dot(d, y) / ahead, ahead};
  };
  V joints[fho::kJoints] = {};
  for (int j = 0; j < fho::kJoints; ++j) joints[j] = {hand.joints[j][0], hand.joints[j][1], hand.joints[j][2]};
  const Seen palm = see(joints[0]);
  const double half = (kWidthM / 2) / palm.ahead, size = size_px;
  const auto place = [&](V p) {
    const Seen s = see(p);
    return OutlineLayout::Point{(s.right - (palm.right - half)) / (2 * half) * size,
                                ((palm.up + half) - s.up) / (2 * half) * size, size / (2 * half * s.ahead)};
  };
  OutlineLayout out{};
  out.size = size_px;
  for (int j = 0; j < fho::kJoints; ++j) out.joints[j] = place(joints[j]);
  out.arm = place(joints[1] + unit(joints[1] - joints[12]) * kForearmM);
  out.tangents[0] = float(palm.right - half);
  out.tangents[1] = float(palm.right + half);
  out.tangents[2] = float(-(palm.up + half));
  out.tangents[3] = float(-(palm.up - half));
  return out;
}

double outline_change_px(const OutlineLayout& a, const OutlineLayout& b) {
  if (a.size != b.size) return 1e9;
  double most = std::max(std::fabs(a.arm.u - b.arm.u), std::fabs(a.arm.v - b.arm.v));
  for (int j = 0; j < fho::kJoints; ++j)
    most = std::max({most, std::fabs(a.joints[j].u - b.joints[j].u), std::fabs(a.joints[j].v - b.joints[j].v)});
  return most;
}

void draw_hand_outline(doodle::Canvas& canvas, const OutlineLayout& layout, uint8_t flags, bool mask, uint8_t hold) {
  const OutlineLayout::Point* pts = layout.joints;
  const OutlineLayout::Point& arm = layout.arm;
  const double line = kLinePer256 * layout.size / 256.0;
  cairo_t* cr = canvas.cr();
  canvas.clear();
  cairo_save(cr);
  cairo_set_line_cap(cr, CAIRO_LINE_CAP_ROUND);
  cairo_set_line_join(cr, CAIRO_LINE_JOIN_ROUND);
  double u0 = arm.u, u1 = arm.u, v0 = arm.v, v1 = arm.v, reach = kWristRadiusM * arm.scale;
  for (int j = 0; j < fho::kJoints; ++j) {
    u0 = std::min(u0, pts[j].u);
    u1 = std::max(u1, pts[j].u);
    v0 = std::min(v0, pts[j].v);
    v1 = std::max(v1, pts[j].v);
    reach = std::max(reach, kWristRadiusM * pts[j].scale);
  }
  reach += line + 2;
  cairo_rectangle(cr, std::floor(u0 - reach), std::floor(v0 - reach), std::ceil(u1 - u0 + 2 * reach) + 1,
                  std::ceil(v1 - v0 + 2 * reach) + 1);
  cairo_clip(cr);
  cairo_set_operator(cr, CAIRO_OPERATOR_SOURCE);
  const auto silhouette = [&](double grow) {
    for (int f = 0; f < 5; ++f) {
      cairo_move_to(cr, pts[1].u, pts[1].v);
      for (int joint : kFingers[f]) {
        if (f == 0 && joint == 1) continue;
        cairo_line_to(cr, pts[joint].u, pts[joint].v);
      }
      cairo_set_line_width(cr, 2 * (finger_radius_px(layout, f) + grow));
      cairo_stroke(cr);
    }
    cairo_set_line_width(cr, 2 * (kWristRadiusM * arm.scale + grow));
    cairo_move_to(cr, pts[1].u, pts[1].v);
    cairo_line_to(cr, arm.u, arm.v);
    cairo_stroke(cr);
    cairo_move_to(cr, pts[kPalm[0]].u, pts[kPalm[0]].v);
    for (int k = 1; k < kPalmPoints; ++k) cairo_line_to(cr, pts[kPalm[k]].u, pts[kPalm[k]].v);
    cairo_close_path(cr);
    cairo_fill_preserve(cr);
    cairo_set_line_width(cr, std::max(2 * grow, 0.01));
    cairo_stroke(cr);
  };
  doodle::color(cr, ink_colour(flags, hold));
  silhouette(line);
  cairo_set_source_rgba(cr, 0, 0, 0, mask ? kMaskAlpha : 0.0);
  silhouette(0);
  const Fade along = fade_along(layout);
  cairo_pattern_t* fade = cairo_pattern_create_linear(along.from_u, along.from_v, along.to_u, along.to_v);
  cairo_pattern_set_extend(fade, CAIRO_EXTEND_PAD);
  cairo_pattern_add_color_stop_rgba(fade, 0, 0, 0, 0, 0);
  cairo_pattern_add_color_stop_rgba(fade, along.wrist, 0, 0, 0, kAtWrist);
  cairo_pattern_add_color_stop_rgba(fade, 1, 0, 0, 0, 1);
  cairo_set_operator(cr, CAIRO_OPERATOR_DEST_IN);
  cairo_set_source(cr, fade);
  cairo_paint(cr);
  cairo_pattern_destroy(fade);
  cairo_surface_t* tips = cairo_image_surface_create(CAIRO_FORMAT_A8, layout.size, layout.size);
  cairo_t* tips_cr = cairo_create(tips);
  paint_tips(tips_cr, layout, line);
  cairo_destroy(tips_cr);
  cairo_set_source_rgba(cr, 0, 0, 0, 1);
  cairo_mask_surface(cr, tips, 0, 0);
  cairo_surface_destroy(tips);
  cairo_restore(cr);
}

OutlineUniforms outline_uniforms(const OutlineLayout& layout, uint8_t flags, bool mask, uint8_t hold) {
  OutlineUniforms out{};
  for (int i = 0; i < 26; ++i) {
    const OutlineLayout::Point& point = i < 25 ? layout.joints[i + 1] : layout.arm;
    out.points[i][0] = float(point.u);
    out.points[i][1] = float(point.v);
  }
  for (int f = 0; f < 4; ++f) out.radius[f] = float(finger_radius_px(layout, f));
  out.more[0] = float(finger_radius_px(layout, 4));
  out.more[1] = float(kWristRadiusM * layout.arm.scale);
  out.more[2] = float(kLinePer256 * layout.size / 256.0);
  out.more[3] = mask ? float(kMaskAlpha) : 0.0f;
  const doodle::Color ink = ink_colour(flags, hold);
  out.ink[0] = float(ink.r * ink.a);
  out.ink[1] = float(ink.g * ink.a);
  out.ink[2] = float(ink.b * ink.a);
  out.ink[3] = float(ink.a);
  const Fade along = fade_along(layout);
  const double ax = along.to_u - along.from_u, ay = along.to_v - along.from_v;
  const double length2 = std::max(ax * ax + ay * ay, 1e-6);
  out.fade[0] = float(along.from_u);
  out.fade[1] = float(along.from_v);
  out.fade[2] = float(ax / length2);
  out.fade[3] = float(ay / length2);
  out.stops[0] = float(along.wrist);
  out.stops[1] = float(kAtWrist);
  return out;
}
