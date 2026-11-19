#include "vr_panel.h"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <iterator>
#include <thread>
#include <vector>

#include "openvr.h"

namespace {

const char* const kPanelKey = "framehandtracking.panel";
const char* const kPanelName = "Hand tracking";
constexpr float kPanelWidthM = 2.0f;
const char* const kOutlineKeys[6] = {"framehandtracking.outline.left",       "framehandtracking.outline.right",
                                     "framehandtracking.outline.left.eye0",  "framehandtracking.outline.left.eye1",
                                     "framehandtracking.outline.right.eye0", "framehandtracking.outline.right.eye1"};
const char* const kOutlineNames[6] = {"Hand outline (left)",
                                      "Hand outline (right)",
                                      "Hand outline (left, left eye)",
                                      "Hand outline (left, right eye)",
                                      "Hand outline (right, left eye)",
                                      "Hand outline (right, right eye)"};

}  // namespace

VrPanel::~VrPanel() { shutdown(); }

bool VrPanel::check(const char* what, int error) const {
  if (error == vr::VROverlayError_None) return true;
  std::fprintf(stderr, "[VR] %s -> %s\n", what,
               vr::VROverlay()->GetOverlayErrorNameFromEnum(vr::EVROverlayError(error)));
  return false;
}

VrPanel::Connect VrPanel::connect(int width, int height, std::string& message) {
  if (connected_) return Connect::Ok;
  vr::EVRInitError error = vr::VRInitError_None;
  vr::VR_Init(&error, vr::VRApplication_Background);
  if (error != vr::VRInitError_None) {
    message = vr::VR_GetVRInitErrorAsEnglishDescription(error);
    return error == vr::VRInitError_Init_NoServerForBackgroundApp ? Connect::NotRunning : Connect::Error;
  }
  vr::VR_Shutdown();
  vr::VR_Init(&error, vr::VRApplication_Overlay);
  if (error != vr::VRInitError_None) {
    message = vr::VR_GetVRInitErrorAsEnglishDescription(error);
    return Connect::Error;
  }
  connected_ = true;
  if (!vulkan_.init(message)) {
    message = "Vulkan: " + message;
    shutdown();
    return Connect::Error;
  }
  vr::IVROverlay* overlay = vr::VROverlay();
  vr::VROverlayHandle_t panel = 0, thumbnail = 0;
  if (!check("CreateDashboardOverlay", overlay->CreateDashboardOverlay(kPanelKey, kPanelName, &panel, &thumbnail))) {
    message = "cannot create the dashboard panel (is another copy of the panel running?)";
    shutdown();
    return Connect::Error;
  }
  panel_ = panel;
  thumbnail_ = thumbnail;
  height_ = height;
  check("SetOverlayWidthInMeters", overlay->SetOverlayWidthInMeters(panel, kPanelWidthM));
  check("SetOverlayInputMethod", overlay->SetOverlayInputMethod(panel, vr::VROverlayInputMethod_Mouse));
  const vr::HmdVector2_t scale = {{float(width), float(height)}};
  check("SetOverlayMouseScale", overlay->SetOverlayMouseScale(panel, &scale));
  if (!panel_texture_.create(vulkan_, width, height, message)) {
    message = "texture: " + message;
    shutdown();
    return Connect::Error;
  }
  std::fprintf(stderr, "[VR] connected: dashboard panel \"%s\"\n", kPanelName);
  return Connect::Ok;
}

void VrPanel::shutdown() {
  if (!connected_) return;
  vr::IVROverlay* overlay = vr::VROverlay();
  std::vector<uint64_t> handles = {panel_, thumbnail_};
  handles.insert(handles.end(), std::begin(outline_), std::end(outline_));
  for (uint64_t handle : handles) {
    if (handle == 0) continue;
    overlay->HideOverlay(handle);
    overlay->ClearOverlayTexture(handle);
  }
  if (panel_ != 0) overlay->DestroyOverlay(panel_);
  for (uint64_t& handle : outline_) {
    if (handle != 0) overlay->DestroyOverlay(handle);
    handle = 0;
  }
  for (int slot = 0; slot < kOutlineSlots; ++slot) outline_shown_[slot] = outline_has_image_[slot] = false;
  panel_ = thumbnail_ = 0;
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  vr::VR_Shutdown();
  panel_texture_.destroy();
  thumbnail_texture_.destroy();
  for (OverlayTexture& texture : outline_texture_) texture.destroy();
  gpu_outline_.destroy();
  gpu_outline_failed_ = false;
  vulkan_.destroy();
  connected_ = false;
}

VrInput VrPanel::poll() {
  VrInput input;
  if (!connected_) return input;
  vr::VREvent_t event{};
  while (vr::VRSystem()->PollNextEvent(&event, sizeof event)) {
    if (event.eventType == vr::VREvent_Quit) {
      vr::VRSystem()->AcknowledgeQuit_Exiting();
      input.quit = true;
    }
  }
  while (panel_ != 0 && vr::VROverlay()->PollNextOverlayEvent(panel_, &event, sizeof event)) {
    const double x = event.data.mouse.x, y = height_ - event.data.mouse.y;
    switch (event.eventType) {
      case vr::VREvent_MouseMove:
        input.panel.push_back({PointerEvent::Type::Move, x, y});
        break;
      case vr::VREvent_MouseButtonDown:
        if (event.data.mouse.button == vr::VRMouseButton_Left) input.panel.push_back({PointerEvent::Type::Down, x, y});
        break;
      case vr::VREvent_MouseButtonUp:
        if (event.data.mouse.button == vr::VRMouseButton_Left) input.panel.push_back({PointerEvent::Type::Up, x, y});
        break;
      case vr::VREvent_FocusLeave:
        input.panel.push_back({PointerEvent::Type::Leave, 0, 0});
        break;
      default:
        break;
    }
  }
  return input;
}

bool VrPanel::visible() const { return connected_ && panel_ != 0 && vr::VROverlay()->IsOverlayVisible(panel_); }

bool VrPanel::set_panel(const uint8_t* rgba) {
  std::string error;
  if (!connected_ || !panel_texture_.update(panel_, rgba, error)) {
    if (!error.empty()) std::fprintf(stderr, "[VR] panel: %s\n", error.c_str());
    return false;
  }
  return true;
}

bool VrPanel::outline() const {
  if (!connected_) return false;
  vr::EVRSettingsError error = vr::VRSettingsError_None;
  const bool on = vr::VRSettings()->GetBool("driver_frame_hand_ray", "handOutline", &error);
  return error == vr::VRSettingsError_None && on;
}

void VrPanel::save_outline(bool on) {
  if (!connected_) return;
  vr::EVRSettingsError error = vr::VRSettingsError_None;
  vr::VRSettings()->SetBool("driver_frame_hand_ray", "handOutline", on, &error);
  if (error != vr::VRSettingsError_None)
    std::fprintf(stderr, "[VR] saving handOutline: %s\n", vr::VRSettings()->GetSettingsErrorNameFromEnum(error));
}

double VrPanel::system_hold_s() const {
  constexpr double kDriverDefault = 5.0;
  if (!connected_) return kDriverDefault;
  vr::EVRSettingsError error = vr::VRSettingsError_None;
  const float ms = vr::VRSettings()->GetFloat("driver_frame_hand_ray", "systemHoldMs", &error);
  return error == vr::VRSettingsError_None && ms >= 0 ? ms * 1e-3 : kDriverDefault;
}

bool VrPanel::mask() const {
  if (!connected_) return false;
  vr::EVRSettingsError error = vr::VRSettingsError_None;
  const bool on = vr::VRSettings()->GetBool("driver_frame_hand_ray", "handMask", &error);
  return error == vr::VRSettingsError_None && on;
}

void VrPanel::save_mask(bool on) {
  if (!connected_) return;
  vr::EVRSettingsError error = vr::VRSettingsError_None;
  vr::VRSettings()->SetBool("driver_frame_hand_ray", "handMask", on, &error);
  if (error != vr::VRSettingsError_None)
    std::fprintf(stderr, "[VR] saving handMask: %s\n", vr::VRSettings()->GetSettingsErrorNameFromEnum(error));
}

bool VrPanel::dashboard_visible() const { return connected_ && vr::VROverlay()->IsDashboardVisible(); }

bool VrPanel::create_outline_overlay(int slot) {
  if (outline_[slot] != 0) return true;
  vr::IVROverlay* overlay = vr::VROverlay();
  vr::VROverlayHandle_t handle = 0;
  if (!check("CreateOverlay(outline)", overlay->CreateOverlay(kOutlineKeys[slot], kOutlineNames[slot], &handle)))
    return false;
  outline_[slot] = handle;
  overlay->SetOverlayInputMethod(handle, vr::VROverlayInputMethod_None);
  overlay->SetOverlaySortOrder(handle, 200);
  overlay->SetOverlayFlag(handle, vr::VROverlayFlags_IsPremultiplied, true);
  outline_width_[slot] = 0;
  return true;
}

bool VrPanel::paint_outline(int hand, const OutlineLayout& layout, uint8_t flags, bool mask, uint8_t hold) {
  if (!connected_ || hand < 0 || hand > 1 || gpu_outline_failed_) return false;
  std::string error;
  if (!gpu_outline_.ready()) {
    const char* cairo = std::getenv("FRAME_HAND_OUTLINE_CAIRO");
    if (cairo != nullptr && cairo[0] != '\0' && cairo[0] != '0') {
      error = "FRAME_HAND_OUTLINE_CAIRO is set";
    } else if (gpu_outline_.create(vulkan_, layout.size, error)) {
      std::fprintf(stderr, "[VR] outline: painted on the GPU (Vulkan, %d px)\n", layout.size);
    }
  }
  if (error.empty() && create_outline_overlay(hand) &&
      gpu_outline_.paint(hand, outline_[hand], layout, flags, mask, hold, error)) {
    outline_has_image_[hand] = true;
    return true;
  }
  std::fprintf(stderr, "[VR] outline: not on the GPU (%s); painted with Cairo\n",
               error.empty() ? "no overlay" : error.c_str());
  gpu_outline_.destroy();
  gpu_outline_failed_ = true;
  return false;
}

bool VrPanel::prepare_outline(int slot, const uint8_t* rgba, int size) {
  std::string error;
  if (!create_outline_overlay(slot)) return false;
  if (rgba == nullptr && outline_has_image_[slot]) return true;
  if (!outline_texture_[slot].ready()) {
    outline_has_image_[slot] = false;
    if (!outline_texture_[slot].create(vulkan_, size, size, error)) {
      std::fprintf(stderr, "[VR] outline texture: %s\n", error.c_str());
      return false;
    }
  }
  if (rgba != nullptr) {
    if (!outline_texture_[slot].update(outline_[slot], rgba, error)) {
      std::fprintf(stderr, "[VR] outline: %s\n", error.c_str());
      return false;
    }
    outline_has_image_[slot] = true;
  }
  return outline_has_image_[slot];
}

bool VrPanel::show_outline(int hand, const uint8_t* rgba, int size, const OutlinePlacement& placement) {
  if (!connected_ || hand < 0 || hand > 1 || !prepare_outline(hand, rgba, size)) return false;
  vr::IVROverlay* overlay = vr::VROverlay();
  vr::HmdMatrix34_t m{};
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 4; ++c) m.m[r][c] = placement.pose[r][c];
  if (placement.width_m != outline_width_[hand]) {
    overlay->SetOverlayWidthInMeters(outline_[hand], placement.width_m);
    outline_width_[hand] = placement.width_m;
  }
  overlay->SetOverlayTransformAbsolute(outline_[hand], vr::TrackingUniverseRawAndUncalibrated, &m);
  if (!outline_shown_[hand]) outline_shown_[hand] = check("ShowOverlay(outline)", overlay->ShowOverlay(outline_[hand]));
  return true;
}

bool VrPanel::show_outline_eye(int hand, int eye, const uint8_t* rgba, int size, const float raw_pose[3][4],
                               const float tangents[4], bool standing) {
  if (!connected_ || hand < 0 || hand > 1 || eye < 0 || eye > 1) return false;
  const int slot = 2 + 2 * hand + eye;
  if (!prepare_outline(slot, rgba, size)) return false;
  vr::IVROverlay* overlay = vr::VROverlay();
  float view_pose[3][4];
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 4; ++c) view_pose[r][c] = raw_pose[r][c];
  if (standing) {
    const vr::HmdMatrix34_t s = vr::VRSystem()->GetRawZeroPoseToStandingAbsoluteTrackingPose();
    if (!logged_standing_)
      std::fprintf(
          stderr, "[VR] the raw tracking space in the standing one: at %+.3f %+.3f %+.3f m, x axis %+.3f %+.3f %+.3f\n",
          s.m[0][3], s.m[1][3], s.m[2][3], s.m[0][0], s.m[1][0], s.m[2][0]);
    logged_standing_ = true;
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 4; ++c)
        view_pose[r][c] = s.m[r][0] * raw_pose[0][c] + s.m[r][1] * raw_pose[1][c] + s.m[r][2] * raw_pose[2][c] +
                          (c == 3 ? s.m[r][3] : 0.0f);
  }
  vr::HmdMatrix34_t m{};
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) m.m[r][c] = view_pose[c][r];
    m.m[r][3] =
        -(view_pose[0][r] * view_pose[0][3] + view_pose[1][r] * view_pose[1][3] + view_pose[2][r] * view_pose[2][3]);
  }
  const vr::VROverlayProjection_t projection = {tangents[0], tangents[1], tangents[2], tangents[3]};
  const vr::EVROverlayError error = overlay->SetOverlayTransformProjection(
      outline_[slot], standing ? vr::TrackingUniverseStanding : vr::TrackingUniverseRawAndUncalibrated, &m, &projection,
      eye == 0 ? vr::Eye_Left : vr::Eye_Right);
  if (error != vr::VROverlayError_None) {
    if (!projection_failed_) check("SetOverlayTransformProjection", error);
    projection_failed_ = true;
    return false;
  }
  if (!outline_shown_[slot])
    outline_shown_[slot] = check("ShowOverlay(outline, eye)", overlay->ShowOverlay(outline_[slot]));
  return true;
}

void VrPanel::hide_outline_slot(int slot) {
  if (!connected_ || outline_[slot] == 0 || !outline_shown_[slot]) return;
  vr::VROverlay()->HideOverlay(outline_[slot]);
  outline_shown_[slot] = false;
}

void VrPanel::hide_outline(int hand) {
  if (hand >= 0 && hand <= 1) hide_outline_slot(hand);
}

void VrPanel::hide_outline_eyes(int hand) {
  if (hand < 0 || hand > 1) return;
  hide_outline_slot(2 + 2 * hand);
  hide_outline_slot(2 + 2 * hand + 1);
}

bool VrPanel::wait_frame_sync(uint32_t timeout_ms) {
  if (!connected_ || frame_sync_failed_) return false;
  const vr::EVROverlayError error = vr::VROverlay()->WaitFrameSync(timeout_ms);
  if (error == vr::VROverlayError_None || error == vr::VROverlayError_TimedOut) return true;
  std::fprintf(stderr, "[VR] WaitFrameSync -> %s; pacing the outline with a timer instead\n",
               vr::VROverlay()->GetOverlayErrorNameFromEnum(error));
  frame_sync_failed_ = true;
  return false;
}

bool VrPanel::eye_poses(float head[3][4], float eyes[2][3][4]) const {
  if (!connected_) return false;
  vr::TrackedDevicePose_t pose{};
  vr::VRSystem()->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseRawAndUncalibrated, 0.02f, &pose, 1);
  if (!pose.bPoseIsValid) return false;
  const auto& h = pose.mDeviceToAbsoluteTracking.m;
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 4; ++c) head[r][c] = h[r][c];
  for (int eye = 0; eye < 2; ++eye) {
    const vr::HmdMatrix34_t e = vr::VRSystem()->GetEyeToHeadTransform(eye == 0 ? vr::Eye_Left : vr::Eye_Right);
    if (!logged_eyes_)
      std::fprintf(stderr, "[VR] %s eye in the head: at %+.4f %+.4f %+.4f m, forward %+.3f %+.3f %+.3f\n",
                   eye == 0 ? "left" : "right", e.m[0][3], e.m[1][3], e.m[2][3], -e.m[0][2], -e.m[1][2], -e.m[2][2]);
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) eyes[eye][r][c] = h[r][0] * e.m[0][c] + h[r][1] * e.m[1][c] + h[r][2] * e.m[2][c];
      eyes[eye][r][3] = h[r][0] * e.m[0][3] + h[r][1] * e.m[1][3] + h[r][2] * e.m[2][3] + h[r][3];
    }
  }
  logged_eyes_ = true;
  return true;
}

bool VrPanel::head_position(float out[3]) const {
  if (!connected_) return false;
  vr::TrackedDevicePose_t pose{};
  vr::VRSystem()->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseRawAndUncalibrated, 0.02f, &pose, 1);
  if (!pose.bPoseIsValid) return false;
  for (int i = 0; i < 3; ++i) out[i] = pose.mDeviceToAbsoluteTracking.m[i][3];
  return true;
}

bool VrPanel::set_thumbnail(const uint8_t* rgba, int size) {
  std::string error;
  if (!connected_) return false;
  if (!thumbnail_texture_.ready() && !thumbnail_texture_.create(vulkan_, size, size, error)) {
    std::fprintf(stderr, "[VR] thumbnail texture: %s\n", error.c_str());
    return false;
  }
  return thumbnail_texture_.update(thumbnail_, rgba, error);
}
