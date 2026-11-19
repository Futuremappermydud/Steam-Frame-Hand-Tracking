#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "gpu_outline.h"
#include "hand_outline.h"
#include "vk_texture.h"

struct PointerEvent {
  enum class Type { Move, Down, Up, Leave };
  Type type;
  double x = 0, y = 0;
};

struct VrInput {
  bool quit = false;
  std::vector<PointerEvent> panel;
};

class VrPanel {
 public:
  enum class Connect { Ok, NotRunning, Error };

  ~VrPanel();
  Connect connect(int width, int height, std::string& message);
  void shutdown();
  bool connected() const { return connected_; }

  VrInput poll();
  bool visible() const;
  bool set_panel(const uint8_t* rgba);
  bool set_thumbnail(const uint8_t* rgba, int size);
  bool outline() const;
  void save_outline(bool on);
  bool mask() const;
  double system_hold_s() const;
  void save_mask(bool on);
  bool dashboard_visible() const;
  bool show_outline(int hand, const uint8_t* rgba, int size, const OutlinePlacement& placement);
  bool paint_outline(int hand, const OutlineLayout& layout, uint8_t flags, bool mask, uint8_t hold);
  bool outline_on_gpu() const { return gpu_outline_.ready(); }
  void hide_outline(int hand);
  bool show_outline_eye(int hand, int eye, const uint8_t* rgba, int size, const float view_pose[3][4],
                        const float tangents[4], bool standing);
  void hide_outline_eyes(int hand);
  bool outline_shown() const {
    for (bool shown : outline_shown_)
      if (shown) return true;
    return false;
  }
  bool wait_frame_sync(uint32_t timeout_ms);
  bool head_position(float out[3]) const;
  bool eye_poses(float head[3][4], float eyes[2][3][4]) const;

 private:
  bool check(const char* what, int error) const;
  static constexpr int kOutlineSlots = 6;
  bool prepare_outline(int slot, const uint8_t* rgba, int size);
  bool create_outline_overlay(int slot);
  void hide_outline_slot(int slot);

  bool connected_ = false;
  VulkanContext vulkan_;
  OverlayTexture panel_texture_, thumbnail_texture_, outline_texture_[kOutlineSlots];
  GpuOutline gpu_outline_;
  bool gpu_outline_failed_ = false;
  uint64_t panel_ = 0, thumbnail_ = 0, outline_[kOutlineSlots] = {};
  bool outline_shown_[kOutlineSlots] = {}, outline_has_image_[kOutlineSlots] = {};
  float outline_width_[kOutlineSlots] = {};
  bool frame_sync_failed_ = false, projection_failed_ = false;
  mutable bool logged_eyes_ = false;
  bool logged_standing_ = false;
  int height_ = 0;
};
