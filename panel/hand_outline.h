#pragma once

#include <cstdint>

#include "doodle_style.h"
#include "hand_outline_packet.h"

class OutlineReceiver {
 public:
  ~OutlineReceiver();
  bool start();
  void stop();
  bool running() const { return fd_ >= 0; }
  bool poll(fho::Packet& newest, int64_t& received_ns);

 private:
  int fd_ = -1;
};

struct OutlinePlacement {
  float pose[3][4];
  float width_m;
};

struct OutlineLayout {
  struct Point {
    double u, v;
    double scale;
  };
  Point joints[fho::kJoints];
  Point arm;
  OutlinePlacement placement;
  int size = 0;
  float tangents[4] = {};
};
OutlineLayout layout_hand_outline(const fho::Hand& hand, const float eye[3], int size_px);
OutlineLayout layout_hand_outline_eye(const fho::Hand& hand, const float eye_pose[3][4], int size_px);
double outline_change_px(const OutlineLayout& a, const OutlineLayout& b);
void draw_hand_outline(doodle::Canvas& canvas, const OutlineLayout& layout, uint8_t flags, bool mask = false,
                       uint8_t hold = 0);

struct OutlineUniforms {
  float points[26][4];
  float radius[4];
  float more[4];
  float ink[4];
  float fade[4];
  float stops[4];
};
OutlineUniforms outline_uniforms(const OutlineLayout& layout, uint8_t flags, bool mask = false, uint8_t hold = 0);

int64_t outline_now_ns();
