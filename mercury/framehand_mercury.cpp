#include "framehand_mercury.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>

#include "hg_sync.hpp"
#include "tracking/t_tracking.h"
#include "util/u_frame.h"

using xrt::tracking::hand::mercury::HandTracking;
static_assert(sizeof(fhm_calibration) == 336);
static_assert(sizeof(fhm_joint) == 40);
static_assert(sizeof(fhm_result) == 2104);
static_assert(sizeof(fhm_diagnostics) == 240);
static_assert(sizeof(fhm_hand_quality) == 104);
static_assert(sizeof(fhm_quality) == 224);
namespace {
thread_local std::string error;
struct Tracker {
  t_hand_tracking_sync* sync = nullptr;
  uint32_t width = 0, height = 0;
  int num_views = 2;
  uint32_t widths[FHM_MAX_VIEWS] = {}, heights[FHM_MAX_VIEWS] = {};
  ~Tracker() { t_ht_sync_destroy(&sync); }
};
bool orientation_valid(int value) { return value == 0 || value == 90 || value == 180 || value == 270; }
}  // namespace
extern "C" {
uint32_t fhm_abi_version() { return 1; }
const char* fhm_source_revision() { return "ec188bb137b6af93120e015a8df100046a34d8f0"; }
const char* fhm_last_error() { return error.c_str(); }
uint32_t fhm_diagnostics_abi_version() { return 3; }
const char* fhm_diagnostics_patch_sha256() { return FHM_DIAGNOSTICS_PATCH_SHA256; }
const char* fhm_acceptance_variant() {
#ifdef FHM_EXPERIMENTAL_GATE
  return "experimental_acquisition_mean_gate";
#else
  return "stock_0.92";
#endif
}
const char* fhm_acceptance_patch_sha256() {
#ifdef FHM_EXPERIMENTAL_GATE
  return FHM_ACCEPTANCE_PATCH_SHA256;
#else
  return "none";
#endif
}
double fhm_get_acquisition_mean_threshold(void* handle) {
#ifdef FHM_EXPERIMENTAL_GATE
  if (!handle) return NAN;
  return HandTracking::fromC(static_cast<Tracker*>(handle)->sync).fhl_experimental_acquisition_threshold;
#else
  return 0.92;
#endif
}
void* fhm_create(const fhm_calibration* input, const char* models) {
  error.clear();
  if (!input || !models || input->views[0].width <= 0 || input->views[0].height <= 0 ||
      input->views[0].width != input->views[1].width || input->views[0].height != input->views[1].height) {
    error = "Two views require the same positive canvas dimensions";
    return nullptr;
  }
  for (const auto& view : input->views) {
    if (!orientation_valid(view.orientation_degrees)) {
      error = "Orientation must be 0/90/180/270";
      return nullptr;
    }
    for (double value : view.K)
      if (!std::isfinite(value)) {
        error = "Nonfinite K";
        return nullptr;
      }
    for (double value : view.kb4)
      if (!std::isfinite(value)) {
        error = "Nonfinite distortion";
        return nullptr;
      }
  }
  t_stereo_camera_calibration* calib = nullptr;
  try {
    auto owner = std::make_unique<Tracker>();
    t_stereo_camera_calibration_alloc(&calib, T_DISTORTION_FISHEYE_KB4);
    t_hand_tracking_create_info info{};
    for (int i = 0; i < 2; ++i) {
      calib->view[i].image_size_pixels = {input->views[i].width, input->views[i].height};
      std::memcpy(calib->view[i].intrinsics, input->views[i].K, sizeof(input->views[i].K));
      std::memcpy(&calib->view[i].kb4, input->views[i].kb4, sizeof(input->views[i].kb4));
      info.cams_info.views[i].boundary_type = HT_IMAGE_BOUNDARY_NONE;
      info.cams_info.views[i].camera_orientation =
          static_cast<t_camera_orientation>(input->views[i].orientation_degrees);
    }
    std::memcpy(calib->camera_rotation, input->second_from_first_rotation, sizeof(calib->camera_rotation));
    std::memcpy(calib->camera_translation, input->second_from_first_translation_m, sizeof(calib->camera_translation));
    owner->width = input->views[0].width;
    owner->height = input->views[0].height;
    owner->sync = t_hand_tracking_sync_mercury_create(calib, info, models);
    t_stereo_camera_calibration_reference(&calib, nullptr);
    if (!owner->sync) {
      error = "Upstream Mercury creation failed; inspect stderr and model paths";
      return nullptr;
    }
    return owner.release();
  } catch (const std::exception& e) {
    t_stereo_camera_calibration_reference(&calib, nullptr);
    error = e.what();
    return nullptr;
  }
}
int fhm_set_parameter(void* handle, const char* name, double value) {
  error.clear();
  if (!handle || !name || !std::isfinite(value)) {
    error = "Invalid parameter input";
    return -1;
  }
  auto& t = HandTracking::fromC(static_cast<Tracker*>(handle)->sync).tuneable_values;
  auto set_float = [&](u_var_draggable_f32& field) {
    if (value < field.min || value > field.max) {
      error = "Parameter outside upstream limits";
      return -1;
    }
    field.val = static_cast<float>(value);
    return 0;
  };
  const std::string key(name);
#ifdef FHM_EXPERIMENTAL_GATE
  if (key == "experimental_acquisition_mean_threshold") {
    if (value < 0 || value > 1) {
      error = "Experimental acquisition threshold must be 0..1";
      return -1;
    }
    HandTracking::fromC(static_cast<Tracker*>(handle)->sync).fhl_experimental_acquisition_threshold = value;
    return 0;
  }
#endif
  auto& h = HandTracking::fromC(static_cast<Tracker*>(handle)->sync);
  if (key == "max_views_per_hand") {
    if (value < 1 || value > xrt::tracking::hand::mercury::kMaxViews || value != std::floor(value)) {
      error = "max_views_per_hand must be 1..4";
      return -1;
    }
    h.max_views_per_hand = static_cast<int>(value);
    return 0;
  }
  if (key == "detector_views_per_frame") {
    if (value < 1 || value > xrt::tracking::hand::mercury::kMaxViews || value != std::floor(value)) {
      error = "detector_views_per_frame must be 1..4";
      return -1;
    }
    h.detector_views_per_frame = static_cast<int>(value);
    return 0;
  }
  if (key == "max_view_angle_deg") {
    if (value < 10 || value > 180) {
      error = "max_view_angle_deg must be 10..180";
      return -1;
    }
    h.max_view_angle_deg = static_cast<float>(value);
    return 0;
  }
  if (key == "view_fade_in_frames") {
    if (value < 1 || value > 60 || value != std::floor(value)) {
      error = "view_fade_in_frames must be 1..60";
      return -1;
    }
    h.view_fade_in_frames = static_cast<int>(value);
    return 0;
  }
  if (key == "acquire_ray_gap_m") {
    if (value <= 0 || value > 1) {
      error = "acquire_ray_gap_m must be 0..1";
      return -1;
    }
    h.acquire_ray_gap_m = static_cast<float>(value);
    return 0;
  }
  if (key == "strict_view_angle_deg") {
    if (value < 0 || value > 180) {
      error = "strict_view_angle_deg must be 0..180";
      return -1;
    }
    h.strict_view_angle_deg = static_cast<float>(value);
    return 0;
  }
  if (key == "fix_region_prediction" || key == "nview_policy" || key == "acquire_require_pair" ||
      key == "require_central_view" || key == "overlap_per_view") {
    if (value != 0 && value != 1) {
      error = "Boolean parameter must be 0 or 1";
      return -1;
    }
    bool& field = key == "nview_policy"           ? h.nview_policy
                  : key == "acquire_require_pair" ? h.acquire_require_pair
                  : key == "require_central_view" ? h.require_central_view
                  : key == "overlap_per_view"     ? h.overlap_per_view
                                                  : h.fix_region_prediction;
    field = value != 0;
    return 0;
  }
  const auto set_count = [&](int& field, int lo, int hi) {
    if (value < lo || value > hi || value != std::floor(value)) {
      error = key + " must be a whole number " + std::to_string(lo) + ".." + std::to_string(hi);
      return -1;
    }
    field = static_cast<int>(value);
    return 0;
  };
  const auto set_within = [&](float& field, double lo, double hi) {
    if (value < lo || value > hi) {
      error = key + " outside its range";
      return -1;
    }
    field = static_cast<float>(value);
    return 0;
  };
  if (key == "acquire_every_n_frames") return set_count(h.acquire_every_n_frames, 1, 30);
  if (key == "idle_acquire_every_n_frames") return set_count(h.idle_acquire_every_n_frames, 1, 30);
  if (key == "recent_loss_frames") return set_count(h.recent_loss_frames, 0, 600);
  if (key == "min_hand_eye_dist") return set_within(h.min_hand_eye_dist, 0, 0.5);
  if (key == "eye_x_m") return set_within(h.eye_point.x, -0.5, 0.5);
  if (key == "eye_y_m") return set_within(h.eye_point.y, -0.5, 0.5);
  if (key == "eye_z_m") return set_within(h.eye_point.z, -0.5, 0.5);
  if (key == "fp_suppress_ms") return set_within(h.fp_suppress_ms, 0, 5000);
  if (key == "min_detection_confidence") return set_float(t.min_detection_confidence);
  if (key == "max_reprojection_error") return set_float(t.max_reprojection_error);
  if (key == "opt_smooth_factor") return set_float(t.opt_smooth_factor);
  if (key == "max_hand_dist") return set_float(t.max_hand_dist);
  if (key == "after_detection_fac") return set_float(t.after_detection_fac);
  if (key == "dyn_radii_fac") return set_float(t.dyn_radii_fac);
  if (value != 0 && value != 1) {
    error = "Boolean parameter must be 0 or 1";
    return -1;
  }
  if (key == "always_run_detection_model")
    t.always_run_detection_model = value != 0;
  else if (key == "optimize_hand_size")
    t.optimize_hand_size = value != 0;
  else if (key == "enable_framerate_based_smoothing")
    t.enable_framerate_based_smoothing = value != 0;
  else if (key == "detection_model_in_both_views")
    t.detection_model_in_both_views = value != 0;
  else {
    error = "Unknown Mercury parameter";
    return -1;
  }
  return 0;
}
int fhm_get_native_second_from_first(void* handle, double* out) {
  if (!handle || !out) return -1;
  const auto pose = HandTracking::fromC(static_cast<Tracker*>(handle)->sync).left_in_right;
  Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  Eigen::Matrix3d r = q.normalized().toRotationMatrix();
  std::fill(out, out + 16, 0.0);
  out[15] = 1;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) out[i * 4 + j] = r(i, j);
  out[3] = pose.position.x;
  out[7] = pose.position.y;
  out[11] = pose.position.z;
  return 0;
}
int fhm_get_diagnostics(void* handle, fhm_diagnostics* out) {
  if (!handle || !out) return -1;
  const auto& hgt = HandTracking::fromC(static_cast<Tracker*>(handle)->sync);
  std::memset(out, 0, sizeof(*out));
  out->frames_before_display = hgt.tuneable_values.num_frames_before_display;
  out->target_hand_size_parameter = hgt.target_hand_size;
  for (int h = 0; h < 2; ++h) {
    out->tracked_frames[h] = hgt.hand_tracked_for_num_frames[h];
    out->this_frame_detected[h] = hgt.this_frame_hand_detected[h];
    out->previous_frame_detected[h] = hgt.last_frame_hand_detected[h];
    out->hand_seen_before[h] = hgt.hand_seen_before[h];
    out->rejection_stage[h] = hgt.fhl_rejection_stage[h];
    out->optimizer_ran[h] = hgt.fhl_optimizer_ran[h];
    out->reprojection_error[h] = hgt.fhl_reprojection_error[h];
    out->acquisition_confidence_average[h] = hgt.fhl_acquisition_confidence[h];
    for (int v = 0; v < 2; ++v) {
      const auto& roi = hgt.views[v].regions_of_interest_this_frame[h];
      int i = v * 2 + h;
      out->roi_confidence[i] = roi.hand_detection_confidence;
      out->roi_center_px[i * 2] = roi.center_px.x;
      out->roi_center_px[i * 2 + 1] = roi.center_px.y;
      out->roi_size_px[i] = roi.size_px;
      out->roi_found[i] = roi.found;
      out->roi_provenance[i] = static_cast<int32_t>(roi.provenance);
      out->roi_input_found[i] = hgt.fhl_roi_input_found[h][v];
      out->keypoint_active[i] = hgt.keypoint_outputs[h].views[v].active;
      out->raw_detector_confidence[i] = hgt.fhl_detector_confidence[v][h];
      out->detector_ran[v] = hgt.fhl_detector_ran[v];
    }
  }
  return 0;
}
int fhm_process(void* handle, const uint8_t* first, uint32_t first_stride, const uint8_t* second,
                uint32_t second_stride, int64_t timestamp, fhm_result* out) {
  error.clear();
  if (!handle || !first || !second || !out) {
    error = "Null process input";
    return -1;
  }
  auto& owner = *static_cast<Tracker*>(handle);
  if (first_stride < owner.width || second_stride < owner.width) {
    error = "Image stride is too small";
    return -1;
  }
  xrt_frame* frames[2] = {nullptr, nullptr};
  try {
    const uint8_t* sources[] = {first, second};
    uint32_t strides[] = {first_stride, second_stride};
    for (int i = 0; i < 2; ++i) {
      u_frame_create_one_off(XRT_FORMAT_L8, owner.width, owner.height, &frames[i]);
      if (!frames[i]) throw std::runtime_error("Frame allocation failed");
      frames[i]->timestamp = frames[i]->source_timestamp = timestamp;
      for (uint32_t y = 0; y < owner.height; ++y)
        std::memcpy(frames[i]->data + y * frames[i]->stride, sources[i] + y * strides[i], owner.width);
    }
    xrt_hand_joint_set hands[2]{};
    std::memset(out, 0, sizeof(*out));
    t_ht_sync_process(owner.sync, frames[0], frames[1], &hands[0], &hands[1], &out->timestamp_ns);
    for (int h = 0; h < 2; ++h) {
      out->hands[h].active = hands[h].is_active;
      for (int j = 0; j < 26; ++j) {
        const auto& native = hands[h].values.hand_joint_set_default[j];
        auto& joint = out->hands[h].joints[j];
        joint.position_m[0] = native.relation.pose.position.x;
        joint.position_m[1] = native.relation.pose.position.y;
        joint.position_m[2] = native.relation.pose.position.z;
        joint.quaternion_xyzw[0] = native.relation.pose.orientation.x;
        joint.quaternion_xyzw[1] = native.relation.pose.orientation.y;
        joint.quaternion_xyzw[2] = native.relation.pose.orientation.z;
        joint.quaternion_xyzw[3] = native.relation.pose.orientation.w;
        joint.radius_m = native.radius;
        joint.relation_flags = native.relation.relation_flags;
      }
    }
    for (auto& frame : frames) xrt_frame_reference(&frame, nullptr);
    return 0;
  } catch (const std::exception& e) {
    for (auto& frame : frames) xrt_frame_reference(&frame, nullptr);
    error = e.what();
    return -1;
  }
}
void fhm_destroy(void* handle) { delete static_cast<Tracker*>(handle); }

uint32_t fhm_nview_abi_version() { return 1; }

void* fhm_create_n(const fhm_rig* rig, const char* models) {
  using namespace xrt::tracking::hand::mercury;
  error.clear();
  if (!rig || !models || rig->num_views < 1 || rig->num_views > FHM_MAX_VIEWS) {
    error = "fhm_create_n needs 1..4 views and a models folder";
    return nullptr;
  }
  const int n = rig->num_views;
  t_camera_calibration calibrations[FHM_MAX_VIEWS] = {};
  t_camera_extra_info_one_view extra[FHM_MAX_VIEWS] = {};
  for (int v = 0; v < n; ++v) {
    const fhm_camera& view = rig->views[v];
    if (view.width <= 0 || view.height <= 0) {
      error = "View sizes must be positive";
      return nullptr;
    }
    if (!orientation_valid(view.orientation_degrees)) {
      error = "Orientation must be 0/90/180/270";
      return nullptr;
    }
    for (double value : view.K)
      if (!std::isfinite(value)) {
        error = "Nonfinite K";
        return nullptr;
      }
    for (double value : view.kb4)
      if (!std::isfinite(value)) {
        error = "Nonfinite distortion";
        return nullptr;
      }
    for (int i = 0; i < 9; ++i)
      if (!std::isfinite(rig->view_from_first_rotation[v][i])) {
        error = "Nonfinite rotation";
        return nullptr;
      }
    for (int i = 0; i < 3; ++i)
      if (!std::isfinite(rig->view_from_first_translation_m[v][i])) {
        error = "Nonfinite translation";
        return nullptr;
      }
    auto& c = calibrations[v];
    c.image_size_pixels = {view.width, view.height};
    std::memcpy(c.intrinsics, view.K, sizeof(view.K));
    c.kb4 = {view.kb4[0], view.kb4[1], view.kb4[2], view.kb4[3]};
    c.distortion_model = T_DISTORTION_FISHEYE_KB4;
    extra[v].camera_orientation = static_cast<t_camera_orientation>(view.orientation_degrees);
    const float* circle = rig->boundary_circle[v];
    if (circle[2] > 0) {
      extra[v].boundary_type = HT_IMAGE_BOUNDARY_CIRCLE;
      extra[v].boundary.circle.normalized_center = {circle[0], circle[1]};
      extra[v].boundary.circle.normalized_radius = circle[2];
    } else {
      extra[v].boundary_type = HT_IMAGE_BOUNDARY_NONE;
    }
  }
  try {
    auto owner = std::make_unique<Tracker>();
    HandTracking* hgt = mercury_create_nview(n, calibrations, rig->view_from_first_rotation,
                                             rig->view_from_first_translation_m, extra, models);
    if (!hgt) {
      error = "N-view Mercury creation failed; inspect stderr and model paths";
      return nullptr;
    }
    owner->sync = &hgt->base;
    owner->num_views = n;
    for (int v = 0; v < n; ++v) {
      owner->widths[v] = rig->views[v].width;
      owner->heights[v] = rig->views[v].height;
    }
    owner->width = owner->widths[0];
    owner->height = owner->heights[0];
    return owner.release();
  } catch (const std::exception& e) {
    error = e.what();
    return nullptr;
  }
}

int fhm_process_n(void* handle, const uint8_t* const* images, const uint32_t* strides, int64_t timestamp,
                  fhm_result* out) {
  error.clear();
  if (!handle || !images || !strides || !out) {
    error = "Null process input";
    return -1;
  }
  auto& owner = *static_cast<Tracker*>(handle);
  const int n = owner.num_views;
  for (int v = 0; v < n; ++v) {
    if (!images[v]) {
      error = "Null view image";
      return -1;
    }
    if (strides[v] < owner.widths[v]) {
      error = "Image stride is too small";
      return -1;
    }
  }
  xrt_frame* frames[FHM_MAX_VIEWS] = {};
  try {
    for (int v = 0; v < n; ++v) {
      u_frame_create_one_off(XRT_FORMAT_L8, owner.widths[v], owner.heights[v], &frames[v]);
      if (!frames[v]) throw std::runtime_error("Frame allocation failed");
      frames[v]->timestamp = frames[v]->source_timestamp = timestamp;
      for (uint32_t y = 0; y < owner.heights[v]; ++y)
        std::memcpy(frames[v]->data + y * frames[v]->stride, images[v] + y * strides[v], owner.widths[v]);
    }
    xrt_hand_joint_set hands[2]{};
    std::memset(out, 0, sizeof(*out));
    HandTracking::fromC(owner.sync).processViews(frames, &hands[0], &hands[1], &out->timestamp_ns);
    for (int h = 0; h < 2; ++h) {
      out->hands[h].active = hands[h].is_active;
      for (int j = 0; j < 26; ++j) {
        const auto& native = hands[h].values.hand_joint_set_default[j];
        auto& joint = out->hands[h].joints[j];
        joint.position_m[0] = native.relation.pose.position.x;
        joint.position_m[1] = native.relation.pose.position.y;
        joint.position_m[2] = native.relation.pose.position.z;
        joint.quaternion_xyzw[0] = native.relation.pose.orientation.x;
        joint.quaternion_xyzw[1] = native.relation.pose.orientation.y;
        joint.quaternion_xyzw[2] = native.relation.pose.orientation.z;
        joint.quaternion_xyzw[3] = native.relation.pose.orientation.w;
        joint.radius_m = native.radius;
        joint.relation_flags = native.relation.relation_flags;
      }
    }
    for (auto& frame : frames) xrt_frame_reference(&frame, nullptr);
    return 0;
  } catch (const std::exception& e) {
    for (auto& frame : frames) xrt_frame_reference(&frame, nullptr);
    error = e.what();
    return -1;
  }
}

int fhm_get_diagnostics_n(void* handle, fhm_diagnostics_n* out) {
  if (!handle || !out) return -1;
  const auto& hgt = HandTracking::fromC(static_cast<Tracker*>(handle)->sync);
  std::memset(out, 0, sizeof(*out));
  out->num_views = hgt.num_views;
  out->nview_policy = hgt.nview_policy;
  out->target_hand_size_parameter = hgt.target_hand_size;
  for (int h = 0; h < 2; ++h) {
    out->tracked_frames[h] = hgt.hand_tracked_for_num_frames[h];
    out->rejection_stage[h] = hgt.fhl_rejection_stage[h];
    out->optimizer_ran[h] = hgt.fhl_optimizer_ran[h];
    out->reprojection_error[h] = hgt.fhl_reprojection_error[h];
    out->acquisition_confidence[h] = hgt.fhl_acquisition_confidence[h];
    for (int v = 0; v < hgt.num_views && v < FHM_MAX_VIEWS; ++v) {
      const auto& roi = hgt.views[v].regions_of_interest_this_frame[h];
      out->view_used[h][v] = hgt.fhl_view_used[h][v];
      out->roi_found[h][v] = roi.found;
      out->roi_provenance[h][v] = static_cast<int32_t>(roi.provenance);
      out->roi_center_px[h][v][0] = roi.center_px.x;
      out->roi_center_px[h][v][1] = roi.center_px.y;
      out->roi_size_px[h][v] = roi.size_px;
    }
  }
  for (int v = 0; v < hgt.num_views && v < FHM_MAX_VIEWS; ++v) {
    out->detector_ran[v] = hgt.fhl_detector_ran[v];
    out->detector_confidence[v][0] = hgt.fhl_detector_confidence[v][0];
    out->detector_confidence[v][1] = hgt.fhl_detector_confidence[v][1];
    out->view_weight[v] = hgt.views[v].weight;
  }
  return 0;
}

uint32_t fhm_quality_abi_version() { return 1; }

int fhm_get_quality(void* handle, fhm_quality* out) {
  if (!handle || !out) return -1;
  const auto& hgt = HandTracking::fromC(static_cast<Tracker*>(handle)->sync);
  std::memset(out, 0, sizeof(*out));
  out->detector_ran = hgt.fhl_detector_dispatched ? 1 : 0;
  out->detections_suppressed = hgt.fhl_detections_suppressed;
  out->frame_index = hgt.fhl_frame_index;
  for (int h = 0; h < 2; ++h) {
    fhm_hand_quality& q = out->hands[h];
    q.rejection_stage = hgt.fhl_rejection_stage[h];
    q.reprojection_error = hgt.fhl_optimizer_ran[h] ? hgt.fhl_reprojection_error[h] : -1.0f;
    q.is_hand = -1.0f;
    float view_means = 0;
    for (int v = 0; v < hgt.num_views && v < FHM_MAX_VIEWS; ++v) {
      if (!hgt.fhl_keypoint_ran[h][v]) continue;
      q.is_hand = std::max(q.is_hand, hgt.fhl_is_hand[h][v]);
      const auto& view = hgt.keypoint_outputs[h].views[v];
      if (!view.active || !hgt.fhl_optimizer_ran[h]) continue;
      ++q.views_used;
      float sum = 0;
      for (int j = 0; j < 21; ++j) {
        const float c = view.keypoints_in_scaled_stereographic[j].confidence_xy;
        q.joint_confidence[j] = std::max(q.joint_confidence[j], c);
        sum += c;
      }
      view_means += sum / 21.0f;
    }
    if (q.views_used > 0)
      q.hand_confidence = view_means / float(q.views_used) / (1.0f + 10.0f * std::max(0.0f, q.reprojection_error));
  }
  return 0;
}

int fhm_get_native_view_from_first(void* handle, int view, double* out) {
  if (!handle || !out) return -1;
  const auto& hgt = HandTracking::fromC(static_cast<Tracker*>(handle)->sync);
  if (view < 0 || view >= hgt.num_views) return -1;
  const auto pose = hgt.views[view].view_from_ref;
  Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  Eigen::Matrix3d r = q.normalized().toRotationMatrix();
  std::fill(out, out + 16, 0.0);
  out[15] = 1;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) out[i * 4 + j] = r(i, j);
  out[3] = pose.position.x;
  out[7] = pose.position.y;
  out[11] = pose.position.z;
  return 0;
}

int fhm_set_detector_transform_n(void* handle, int num_views, const float* angles_deg, const float* crops, int enable) {
  error.clear();
  if (!handle || (enable != 0 && enable != 1) || (enable && (!angles_deg || !crops))) {
    error = "Invalid detector transform arguments";
    return -1;
  }
  auto& hgt = HandTracking::fromC(static_cast<Tracker*>(handle)->sync);
  if (num_views != hgt.num_views) {
    error = "View count does not match the tracker";
    return -1;
  }
  if (enable)
    for (int v = 0; v < num_views; ++v) {
      if (!std::isfinite(angles_deg[v]) || std::abs(angles_deg[v]) > 720 || !std::isfinite(crops[v]) ||
          crops[v] < 0.2f || crops[v] > 1.5f) {
        error = "Detector angle or crop out of range";
        return -1;
      }
      hgt.fhl_detector_rotation_deg[v] = angles_deg[v];
      hgt.fhl_detector_crop[v] = crops[v];
    }
  hgt.fhl_detector_custom = enable != 0;
  return 0;
}
}

extern "C" int fhm_set_external_regions(void* handle, const float* values, int enable) {
  using namespace xrt::tracking::hand::mercury;
  error.clear();
  if (!handle || (enable != 0 && enable != 1) || (enable && !values)) {
    error = "Invalid external ROI arguments";
    return -1;
  }
  auto& owner = *static_cast<Tracker*>(handle);
  auto& hgt = HandTracking::fromC(owner.sync);
  hand_region_of_interest checked[2][2] = {};
  if (enable)
    for (int v = 0; v < 2; ++v)
      for (int h = 0; h < 2; ++h) {
        const float* r = values + (v * 2 + h) * 5;
        for (int i = 0; i < 5; ++i)
          if (!std::isfinite(r[i])) {
            error = "Nonfinite external ROI";
            return -1;
          }
        if ((r[4] != 0 && r[4] != 1) || r[3] < 0 || r[3] > 1) {
          error = "Invalid external ROI flag/score";
          return -1;
        }
        auto& roi = checked[v][h];
        roi.provenance = ROIProvenance::HAND_DETECTION;
        roi.found = r[4] != 0;
        roi.hand_detection_confidence = r[3];
        if (roi.found) {
          if (r[0] < 0 || r[0] >= owner.width || r[1] < 0 || r[1] >= owner.height || r[2] <= 0 ||
              r[2] > 2 * std::max(owner.width, owner.height)) {
            error = "External ROI outside allowed range";
            return -1;
          }
          roi.center_px = {r[0], r[1]};
          roi.size_px = r[2];
        }
      }
  for (int v = 0; v < 2; ++v)
    for (int h = 0; h < 2; ++h) hgt.fhl_external_regions[v][h] = checked[v][h];
  hgt.fhl_external_roi_mode = enable != 0;
  return 0;
}
extern "C" int fhm_external_roi_mode(void* handle) {
  if (!handle) return -1;
  return HandTracking::fromC(static_cast<Tracker*>(handle)->sync).fhl_external_roi_mode ? 1 : 0;
}

extern "C" int fhm_set_external_crop_twists(void* handle, const float* values) {
  using namespace xrt::tracking::hand::mercury;
  if (!handle || !values) {
    error = "Null crop twist input";
    return -1;
  }
  for (int i = 0; i < 4; ++i)
    if (!std::isfinite(values[i]) || std::abs(values[i]) > 6.283186f) {
      error = "Invalid crop twist";
      return -1;
    }
  auto& hgt = HandTracking::fromC(static_cast<Tracker*>(handle)->sync);
  for (int v = 0; v < 2; ++v)
    for (int h = 0; h < 2; ++h) hgt.fhl_external_crop_twists[v][h] = values[v * 2 + h];
  return 0;
}

extern "C" int fhm_set_crop_twists_n(void* handle, int num_views, const float* values) {
  using namespace xrt::tracking::hand::mercury;
  if (!handle || !values) {
    error = "Null crop twist input";
    return -1;
  }
  auto& hgt = HandTracking::fromC(static_cast<Tracker*>(handle)->sync);
  if (num_views != hgt.num_views) {
    error = "View count does not match the tracker";
    return -1;
  }
  for (int i = 0; i < num_views * 2; ++i)
    if (!std::isfinite(values[i]) || std::abs(values[i]) > 6.283186f) {
      error = "Invalid crop twist";
      return -1;
    }
  for (int v = 0; v < num_views; ++v)
    for (int h = 0; h < 2; ++h) hgt.fhl_external_crop_twists[v][h] = values[v * 2 + h];
  return 0;
}

extern "C" const char* fhm_research_variant() { return "v7-external-acquisition-1"; }

extern "C" int fhm_set_external_policy(void* handle, int policy) {
  using namespace xrt::tracking::hand::mercury;
  error.clear();
  if (!handle || (policy != 0 && policy != 1)) {
    error = "External policy must be 0 or 1";
    return -1;
  }
  HandTracking::fromC(static_cast<Tracker*>(handle)->sync).fhl_external_policy = policy;
  return 0;
}

extern "C" int fhm_get_external_state(void* handle, int32_t* out) {
  using namespace xrt::tracking::hand::mercury;
  if (!handle || !out) return -1;
  const auto& hgt = HandTracking::fromC(static_cast<Tracker*>(handle)->sync);
  for (int h = 0; h < 2; ++h) {
    out[h] = hgt.last_frame_hand_detected[h] ? 1 : 0;
    out[2 + h] = hgt.fhl_external_roi_use[h];
  }
  return 0;
}

extern "C" uint32_t fhm_detector_transform_abi() { return 1; }
extern "C" int fhm_set_detector_transform(void* handle, const float* angles_deg, const float* crops, int enable) {
  using namespace xrt::tracking::hand::mercury;
  error.clear();
  if (!handle || (enable != 0 && enable != 1) || (enable && (!angles_deg || !crops))) {
    error = "Invalid detector transform arguments";
    return -1;
  }
  auto& hgt = HandTracking::fromC(static_cast<Tracker*>(handle)->sync);
  if (enable)
    for (int v = 0; v < 2; ++v) {
      if (!std::isfinite(angles_deg[v]) || std::abs(angles_deg[v]) > 720 || !std::isfinite(crops[v]) ||
          crops[v] < 0.2f || crops[v] > 1.5f) {
        error = "Detector angle or crop out of range";
        return -1;
      }
      hgt.fhl_detector_rotation_deg[v] = angles_deg[v];
      hgt.fhl_detector_crop[v] = crops[v];
    }
  hgt.fhl_detector_custom = enable != 0;
  return 0;
}
