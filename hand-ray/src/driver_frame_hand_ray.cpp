#include <fcntl.h>
#include <openvr_driver.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <deque>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "controller_watch.h"
#include "eye_gaze.h"
#include "hand_filter.h"
#include "hand_outline_packet.h"
#include "hand_pose.h"
#include "hand_ray_model.h"
#include "hand_skeleton.h"

using namespace vr;

namespace {

constexpr const char* kSection = "driver_frame_hand_ray";
constexpr const char* kVersion = "0.8.0-prototype";

int64_t now_raw_ns() {
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
  return int64_t(ts.tv_sec) * 1'000'000'000 + ts.tv_nsec;
}

void driver_log(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
void driver_log(const char* fmt, ...) {
  char buf[512];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof buf, fmt, args);
  va_end(args);
  VRDriverLog()->Log(buf);
}

constexpr size_t kMaxTuneFileBytes = 64 * 1024;
constexpr int64_t kRejectedLogNs = 10'000'000'000LL;

bool read_own_small_file(const std::string& path, size_t max_bytes, std::string& out) {
  const int fd = open(path.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC | O_NOCTTY);
  if (fd < 0) return false;
  struct stat file_status{};
  bool complete = false;
  if (fstat(fd, &file_status) == 0 && S_ISREG(file_status.st_mode) && file_status.st_uid == getuid() &&
      file_status.st_size >= 0 && size_t(file_status.st_size) <= max_bytes) {
    out.clear();
    char chunk[4096];
    for (;;) {
      const ssize_t count = read(fd, chunk, sizeof chunk);
      if (count < 0 && errno == EINTR) continue;
      complete = count == 0;
      if (count <= 0 || out.size() + size_t(count) > max_bytes) break;
      out.append(chunk, size_t(count));
    }
  }
  close(fd);
  return complete;
}

struct Settings {
  bool enable = true;
  fhr::RayParams ray;
  int connect_packets = 3;
  int lost_ms = 1000;
  int stale_ms = 250;
  int pinch_hold_ms = 200;
  bool single_ray = true;
  double filter_min_cutoff = 2.0, filter_beta = 4.0;
  double direction_min_cutoff = 1.5, direction_beta = 2.0;
  double predict_gain = 1.0;
  double predict_lead_ms = 20;
  double predict_max_ms = 150;
  double velocity_smoothing = 0.5;
  bool finger_filter = true;
  double finger_min_cutoff = 2.0, finger_beta = 3.0, finger_predict_max_ms = 40;
  double confidence_low = 0.05, confidence_high = 0.25;
  int pinch_rewind_ms = 120;
  int pinch_release_hold_ms = 150;
  bool hand_skeleton = true;
  double grip_begin = 0.55, grip_end = 0.70;
  double pinch_value_open_m = 0.08, pinch_value_closed_m = 0.025;
  int pose_mode = 2;
  double hand_rotation_min_cutoff = 1.5, hand_rotation_beta = 0.5;
  double intent_cone_deg = 30, intent_exit_deg = 45;
  int intent_show_ms = 150, intent_hide_ms = 600;
  bool intent_gaze = false;
  double gaze_min_cutoff = 1.0, gaze_beta = 1.5;
  bool hand_outline = false;
  bool system_gesture = true;
  double point_min_palm_deg = 50, point_max_down_deg = 40, point_hide_ms = 350;
  double system_enter_deg = 40, system_max_view_deg = 45, system_arm_ms = 250, system_hold_ms = 5000;
  double gaze_yaw_offset_deg = 0, gaze_pitch_offset_deg = 0;
  double gaze_blink_openness = 0.35;
  int gaze_stale_ms = 250;
  int controller_policy = 1;
  double controller_move_speed = 0.15;
  int controller_quiet_ms = 1500;
  double controller_hold_m = 0.25;

  struct Field {
    const char* name;
    double* d;
    int* i;
    bool* b;
    double lo, hi;
  };
  std::vector<Field> fields() {
    return {
        {"shoulderWidth", &ray.shoulder_width_m, nullptr, nullptr, 0, 0.5},
        {"shoulderDrop", &ray.shoulder_drop_m, nullptr, nullptr, -0.2, 0.6},
        {"shoulderBack", &ray.shoulder_back_m, nullptr, nullptr, -0.3, 0.3},
        {"connectFrames", nullptr, &connect_packets, nullptr, 1, 60},
        {"lostMs", nullptr, &lost_ms, nullptr, 50, 5000},
        {"staleMs", nullptr, &stale_ms, nullptr, 50, 5000},
        {"pinchHoldMs", nullptr, &pinch_hold_ms, nullptr, 0, 1000},
        {"singleRay", nullptr, nullptr, &single_ray, 0, 1},
        {"filterMinCutoff", &filter_min_cutoff, nullptr, nullptr, 0.05, 30},
        {"filterBeta", &filter_beta, nullptr, nullptr, 0, 100},
        {"directionMinCutoff", &direction_min_cutoff, nullptr, nullptr, 0.05, 30},
        {"directionBeta", &direction_beta, nullptr, nullptr, 0, 100},
        {"predictGain", &predict_gain, nullptr, nullptr, 0, 1.5},
        {"predictLeadMs", &predict_lead_ms, nullptr, nullptr, 0, 100},
        {"predictMaxMs", &predict_max_ms, nullptr, nullptr, 0, 300},
        {"velocitySmoothing", &velocity_smoothing, nullptr, nullptr, 0, 0.95},
        {"fingerFilter", nullptr, nullptr, &finger_filter, 0, 1},
        {"fingerMinCutoff", &finger_min_cutoff, nullptr, nullptr, 0.05, 30},
        {"fingerBeta", &finger_beta, nullptr, nullptr, 0, 100},
        {"fingerPredictMaxMs", &finger_predict_max_ms, nullptr, nullptr, 0, 100},
        {"confidenceLow", &confidence_low, nullptr, nullptr, 0, 1},
        {"confidenceHigh", &confidence_high, nullptr, nullptr, 0, 1},
        {"intentConeDeg", &intent_cone_deg, nullptr, nullptr, 5, 90},
        {"intentExitDeg", &intent_exit_deg, nullptr, nullptr, 5, 120},
        {"intentShowMs", nullptr, &intent_show_ms, nullptr, 0, 2000},
        {"intentHideMs", nullptr, &intent_hide_ms, nullptr, 0, 5000},
        {"intentGaze", nullptr, nullptr, &intent_gaze, 0, 1},
        {"handOutline", nullptr, nullptr, &hand_outline, 0, 1},
        {"systemGesture", nullptr, nullptr, &system_gesture, 0, 1},
        {"pointMinPalmDeg", &point_min_palm_deg, nullptr, nullptr, 0, 180},
        {"pointMaxDownDeg", &point_max_down_deg, nullptr, nullptr, 0, 90},
        {"pointHideMs", &point_hide_ms, nullptr, nullptr, 0, 2000},
        {"systemMaxViewDeg", &system_max_view_deg, nullptr, nullptr, 0, 90},
        {"systemEnterDeg", &system_enter_deg, nullptr, nullptr, 0, 90},
        {"systemArmMs", &system_arm_ms, nullptr, nullptr, 0, 2000},
        {"systemHoldMs", &system_hold_ms, nullptr, nullptr, 0, 15000},
        {"pinchRewindMs", nullptr, &pinch_rewind_ms, nullptr, 0, 300},
        {"pinchReleaseHoldMs", nullptr, &pinch_release_hold_ms, nullptr, 0, 1000},
        {"handSkeleton", nullptr, nullptr, &hand_skeleton, 0, 1},
        {"gripBegin", &grip_begin, nullptr, nullptr, 0.1, 1.5},
        {"gripEnd", &grip_end, nullptr, nullptr, 0.1, 1.5},
        {"pinchValueOpenM", &pinch_value_open_m, nullptr, nullptr, 0.02, 0.2},
        {"pinchValueClosedM", &pinch_value_closed_m, nullptr, nullptr, 0, 0.1},
        {"poseMode", nullptr, &pose_mode, nullptr, 0, 2},
        {"handRotationMinCutoff", &hand_rotation_min_cutoff, nullptr, nullptr, 0.05, 30},
        {"handRotationBeta", &hand_rotation_beta, nullptr, nullptr, 0, 100},
        {"gazeMinCutoff", &gaze_min_cutoff, nullptr, nullptr, 0.05, 30},
        {"gazeBeta", &gaze_beta, nullptr, nullptr, 0, 100},
        {"gazeYawOffsetDeg", &gaze_yaw_offset_deg, nullptr, nullptr, -15, 15},
        {"gazePitchOffsetDeg", &gaze_pitch_offset_deg, nullptr, nullptr, -15, 15},
        {"gazeBlinkOpenness", &gaze_blink_openness, nullptr, nullptr, 0, 1},
        {"gazeStaleMs", nullptr, &gaze_stale_ms, nullptr, 50, 2000},
        {"controllerPolicy", nullptr, &controller_policy, nullptr, 0, 2},
        {"controllerMoveSpeed", &controller_move_speed, nullptr, nullptr, 0.02, 2},
        {"controllerQuietMs", nullptr, &controller_quiet_ms, nullptr, 0, 10000},
        {"controllerHoldM", &controller_hold_m, nullptr, nullptr, 0.05, 1},
    };
  }
  int hand_selection_priority() const { return controller_policy == 1 ? 1 : controller_policy == 2 ? -1 : 0; }
  static void set_field(const Field& f, double v) {
    v = std::clamp(v, f.lo, f.hi);
    if (f.d) *f.d = v;
    if (f.i) *f.i = int(std::lround(v));
    if (f.b) *f.b = v != 0;
  }
  void load() {
    auto* s = VRSettings();
    EVRSettingsError e;
    const bool on = s->GetBool(kSection, "enable", &e);
    if (e == VRSettingsError_None) enable = on;
    for (const Field& f : fields()) {
      double v;
      if (f.b)
        v = s->GetBool(kSection, f.name, &e);
      else if (f.i)
        v = s->GetInt32(kSection, f.name, &e);
      else
        v = s->GetFloat(kSection, f.name, &e);
      if (e == VRSettingsError_None) set_field(f, v);
    }
  }
  std::string apply(const std::string& text) {
    std::istringstream in(text);
    std::string line, summary;
    const auto field_list = fields();
    while (std::getline(in, line)) {
      const auto eq = line.find('=');
      if (line.empty() || line[0] == '#' || eq == std::string::npos) continue;
      const std::string key = line.substr(0, eq);
      const char* text_value = line.c_str() + eq + 1;
      char* end = nullptr;
      const double v = std::strtod(text_value, &end);
      bool known = false;
      for (const Field& f : field_list) {
        if (key != f.name) continue;
        known = true;
        if (end != text_value && std::isfinite(v)) {
          set_field(f, v);
          summary += " " + key + "=" + std::to_string(v);
        } else
          summary += " " + key + "=(bad value)";
      }
      if (!known) summary += " " + key + "(unknown)";
    }
    return summary;
  }
};

class PacketReceiver {
 public:
  bool start() {
    fd_ = socket(AF_UNIX, SOCK_DGRAM | SOCK_CLOEXEC, 0);
    if (fd_ < 0) return false;
    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    static const char name[] = "frame_hand_ray";
    std::memcpy(addr.sun_path + 1, name, sizeof(name) - 1);
    const socklen_t len = socklen_t(offsetof(sockaddr_un, sun_path) + 1 + sizeof(name) - 1);
    const int one = 1;
    setsockopt(fd_, SOL_SOCKET, SO_PASSCRED, &one, sizeof one);
    timeval tv{0, 200'000};
    setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
    if (bind(fd_, reinterpret_cast<sockaddr*>(&addr), len) != 0) {
      close(fd_);
      fd_ = -1;
      return false;
    }
    running_ = true;
    thread_ = std::thread([this] { loop(); });
    return true;
  }
  void stop() {
    running_ = false;
    if (thread_.joinable()) thread_.join();
    if (fd_ >= 0) close(fd_);
    fd_ = -1;
  }
  ~PacketReceiver() { stop(); }
  bool latest(fhr::Packet& out, int64_t& received_ns) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!have_) return false;
    out = latest_;
    received_ns = received_ns_;
    return true;
  }
  uint64_t rejected() const { return rejected_; }

 private:
  void loop() {
    uint8_t buf[4096];
    alignas(cmsghdr) char control[CMSG_SPACE(sizeof(ucred))];
    const uid_t me = getuid();
    while (running_) {
      iovec iov{buf, sizeof buf};
      msghdr msg{};
      msg.msg_iov = &iov;
      msg.msg_iovlen = 1;
      msg.msg_control = control;
      msg.msg_controllen = sizeof control;
      const ssize_t n = recvmsg(fd_, &msg, 0);
      if (n <= 0) continue;
      bool same_user = false;
      for (cmsghdr* c = CMSG_FIRSTHDR(&msg); c; c = CMSG_NXTHDR(&msg, c)) {
        if (c->cmsg_level == SOL_SOCKET && c->cmsg_type == SCM_CREDENTIALS) {
          ucred cred;
          std::memcpy(&cred, CMSG_DATA(c), sizeof cred);
          same_user = cred.uid == me;
        }
      }
      fhr::Packet p;
      if (!same_user || !fhr::decode(buf, size_t(n), p)) {
        ++rejected_;
        continue;
      }
      std::lock_guard<std::mutex> lock(mutex_);
      latest_ = p;
      received_ns_ = now_raw_ns();
      have_ = true;
    }
  }
  int fd_ = -1;
  std::atomic<bool> running_{false};
  std::thread thread_;
  std::mutex mutex_;
  fhr::Packet latest_;
  int64_t received_ns_ = 0;
  bool have_ = false;
  std::atomic<uint64_t> rejected_{0};
};

class HandDevice : public ITrackedDeviceServerDriver {
 public:
  HandDevice(bool right, const Settings& s)
      : right_(right),
        role_(right ? TrackedControllerRole_RightHand : TrackedControllerRole_LeftHand),
        settings_(s),
        anchor_filter_(s.filter_min_cutoff, s.filter_beta),
        direction_filter_(s.direction_min_cutoff, s.direction_beta) {}

  const char* serial() const { return right_ ? "frame_hand_ray_right" : "frame_hand_ray_left"; }
  uint32_t id() const { return object_id_; }
  bool connected() const { return connected_; }
  bool world_joints(std::array<fhr::Vec3, fhr::kXrJoints>& out) const {
    if (!have_world_) return false;
    out = world_;
    return true;
  }
  void set_ray_role(ETrackedControllerRole role) {
    if (role == role_) return;
    driver_log("%s takes the %s-hand role", serial(), role == TrackedControllerRole_LeftHand ? "left" : "right");
    role_ = role;
    if (connected_) VRProperties()->SetInt32Property(container_, Prop_ControllerRoleHint_Int32, role_);
  }
  void set_yield(bool on, const char* why) {
    if (on == yielded_) return;
    yielded_ = on;
    driver_log("%s %s", serial(), on ? why : "may connect again: its controller has been quiet");
  }
  void set_priority(int priority) {
    if (container_ == k_ulInvalidPropertyContainer || priority == priority_) return;
    priority_ = priority;
    VRProperties()->SetInt32Property(container_, Prop_ControllerHandSelectionPriority_Int32, priority);
  }

  EVRInitError Activate(uint32_t object_id) override {
    object_id_ = object_id;
    container_ = VRProperties()->TrackedDeviceToPropertyContainer(object_id);
    auto* p = VRProperties();
    p->SetStringProperty(container_, Prop_ModelNumber_String, right_ ? "Hand Ray (right)" : "Hand Ray (left)");
    p->SetStringProperty(container_, Prop_ManufacturerName_String, "Frame Hand Tracking");
    p->SetStringProperty(container_, Prop_SerialNumber_String, serial());
    p->SetStringProperty(container_, Prop_ControllerType_String, "frame_hand_ray");
    p->SetStringProperty(container_, Prop_InputProfilePath_String,
                         "{frame_hand_ray}/input/frame_hand_ray_profile.json");
    p->SetStringProperty(container_, Prop_RenderModelName_String, "{frame_hand_ray}/rendermodels/frame_hand_ray_none");
    p->SetInt32Property(container_, Prop_DeviceClass_Int32, TrackedDeviceClass_Controller);
    p->SetInt32Property(container_, Prop_ControllerRoleHint_Int32, TrackedControllerRole_OptOut);
    p->SetBoolProperty(container_, Prop_NeverTracked_Bool, false);
    set_priority(settings_.hand_selection_priority());
    auto* in = VRDriverInput();
    in->CreateBooleanComponent(container_, "/input/trigger/click", &trigger_click_);
    in->CreateScalarComponent(container_, "/input/trigger/value", &trigger_value_, VRScalarType_Absolute,
                              VRScalarUnits_NormalizedOneSided);
    in->CreateBooleanComponent(container_, "/input/grip/click", &grip_click_);
    in->CreateScalarComponent(container_, "/input/grip/value", &grip_value_, VRScalarType_Absolute,
                              VRScalarUnits_NormalizedOneSided);
    const char* ext[3] = {"pinch_ext", "grasp_ext", "aim_activate_ext"};
    for (int i = 0; i < 3; ++i) {
      in->CreateScalarComponent(container_, ("/input/" + std::string(ext[i]) + "/value").c_str(), &ext_value_[i],
                                VRScalarType_Absolute, VRScalarUnits_NormalizedOneSided);
      in->CreateBooleanComponent(container_, ("/input/" + std::string(ext[i]) + "/ready_ext").c_str(), &ext_ready_[i]);
    }
    in->CreateBooleanComponent(container_, "/input/laser/click", &laser_click_);
    in->CreateBooleanComponent(container_, "/input/pointing/click", &pointing_click_);
    in->CreateBooleanComponent(container_, "/input/system/click", &system_click_);
    const EVRInputError e =
        in->CreateSkeletonComponent(container_, right_ ? "/input/skeleton/right" : "/input/skeleton/left",
                                    right_ ? "/skeleton/hand/right" : "/skeleton/hand/left", "/pose/raw",
                                    VRSkeletalTracking_Full, nullptr, 0, &skeleton_);
    if (e != VRInputError_None) {
      driver_log("%s: CreateSkeletonComponent failed (%d)", serial(), int(e));
      skeleton_ = k_ulInvalidInputComponentHandle;
    }
    pose_.qWorldFromDriverRotation = {1, 0, 0, 0};
    pose_.qDriverFromHeadRotation = {1, 0, 0, 0};
    pose_.qRotation = {1, 0, 0, 0};
    pose_.deviceIsConnected = false;
    pose_.poseIsValid = false;
    pose_.result = TrackingResult_Uninitialized;
    driver_log("%s activated as device %u (skeleton %s)", serial(), object_id,
               skeleton_ != k_ulInvalidInputComponentHandle ? "on" : "off");
    return VRInitError_None;
  }
  void Deactivate() override { object_id_ = k_unTrackedDeviceIndexInvalid; }
  void EnterStandby() override {}
  void* GetComponent(const char*) override { return nullptr; }
  void DebugRequest(const char*, char* response, uint32_t size) override {
    if (size) response[0] = 0;
  }
  DriverPose_t GetPose() override { return pose_; }

  void update(const fhr::Packet* packet, int64_t received_ns, const fhr::PoseHistory& history, int64_t now_ns,
              int source, bool laser, bool hand_pose, bool may_point = true, fhr::PinchUse use = fhr::PinchUse::None,
              bool system_button = false) {
    if (object_id_ == k_unTrackedDeviceIndexInvalid) return;
    const bool fresh = packet && now_ns - received_ns < int64_t(settings_.stale_ms) * 1'000'000;
    if (source != source_) {
      if (source >= 0 && source_ >= 0) driver_log("%s now follows the %s hand", serial(), source ? "right" : "left");
      source_ = source;
      anchor_filter_.reset();
      direction_filter_.reset();
      last_capture_ns_ = 0;
      origin_x_.reset();
      direction_x_.reset();
      pinch_ = false;
      wait_release_ = true;
      have_skeleton_ = false;
      grip_.reset();
      reset_hand_pose();
      trust_ = 1;
    }
    if (laser != laser_) {
      laser_ = laser;
      if (laser_) laser_wait_release_ = pinch_;
      history_.clear();
      offset_weight_ = 0;
    }
    const fhr::HandSample* hand = fresh && source_ >= 0 ? &packet->hands[source_] : nullptr;
    const bool new_packet = packet && packet->sequence != last_sequence_;
    const bool pinch_expired = now_ns - last_valid_ns_ > int64_t(settings_.pinch_hold_ms) * 1'000'000;
    if (new_packet && hand && hand->valid)
      trust_ = fhr::confidence_trust(hand->confidence, settings_.confidence_low, settings_.confidence_high);
    const double smooth = 0.5 + 0.5 * trust_;
    anchor_filter_.set(settings_.filter_min_cutoff * smooth, settings_.filter_beta);
    direction_filter_.set(settings_.direction_min_cutoff * smooth, settings_.direction_beta);
    hand_position_filter_.set(settings_.filter_min_cutoff * smooth, settings_.filter_beta);
    hand_rotation_filter_.set(settings_.hand_rotation_min_cutoff * smooth, settings_.hand_rotation_beta);
    if (new_packet) {
      last_sequence_ = packet->sequence;
      if (hand && hand->valid) {
        ++valid_streak_;
        last_valid_ns_ = now_ns;
        const fhr::Pose head = history.at(packet->capture_raw_ns);
        const fhr::Ray ray = fhr::hand_ray(head, *hand, source_ == 1, settings_.ray);
        const double dt = last_capture_ns_ ? double(packet->capture_raw_ns - last_capture_ns_) * 1e-9 : 0;
        last_capture_ns_ = packet->capture_raw_ns;
        origin_ = anchor_filter_.filter(ray.origin, dt);
        direction_ = fhr::normalized(direction_filter_.filter(ray.direction, dt));
        origin_x_.update(origin_, packet->capture_raw_ns, settings_.velocity_smoothing);
        direction_x_.update(direction_, packet->capture_raw_ns, settings_.velocity_smoothing);
        if (!hand->pinch) wait_release_ = false;
        pinch_ = hand->pinch && !wait_release_;
        pinch_value_ = fhr::ramp(hand->pinch_distance_m, settings_.pinch_value_open_m, settings_.pinch_value_closed_m);
        if (hand->has_joints && source_ == (right_ ? 1 : 0)) {
          const fhr::Xform head_x{head.position, head.rotation};
          const fhr::Xform ray_inv = fhr::inverse(fhr::Xform{ray.origin, fhr::look_rotation(ray.direction)});
          std::array<fhr::Xform, fhr::kXrJoints> world;
          for (int i = 0; i < fhr::kXrJoints; ++i) world[i] = head_x * hand->joints_head[i];
          const fhr::Xform hand_raw = fhr::hand_device_pose(world[fhr::kWrist], right_);
          const double hand_dt = hand_capture_ns_ ? double(packet->capture_raw_ns - hand_capture_ns_) * 1e-9 : 0;
          hand_capture_ns_ = packet->capture_raw_ns;
          hand_position_ = hand_position_filter_.filter(hand_raw.p, hand_dt);
          hand_rotation_ = hand_rotation_filter_.filter(hand_raw.q, hand_dt);
          hand_position_x_.update(hand_position_, packet->capture_raw_ns, settings_.velocity_smoothing);
          const fhr::Xform hand_inv = fhr::inverse(hand_raw);
          for (int i = 0; i < fhr::kXrJoints; ++i) hand_in_hand_[i] = hand_inv * world[i];
          ray_from_hand_ = ray_inv * hand_raw;
          finger_filter_.update(hand_in_hand_, packet->capture_raw_ns, finger_params());
          have_hand_pose_ = true;
          have_skeleton_ = true;
          curl_ = fhr::finger_curl(hand->joints_head);
          grip_.update(curl_, settings_.grip_begin, settings_.grip_end);
        }
      } else {
        valid_streak_ = 0;
        if (pinch_expired) pinch_ = false;
      }
    }
    if (!fresh) {
      valid_streak_ = 0;
      if (pinch_expired) pinch_ = false;
    }
    if (!pinch_) laser_wait_release_ = false;

    const bool lost_long = now_ns - last_valid_ns_ > int64_t(settings_.lost_ms) * 1'000'000;
    if (!connected_ && valid_streak_ >= settings_.connect_packets && !yielded_)
      set_connected(true);
    else if (connected_ && (lost_long || yielded_))
      set_connected(false);
    if (!connected_) {
      anchor_filter_.reset();
      direction_filter_.reset();
      last_capture_ns_ = 0;
      origin_x_.reset();
      direction_x_.reset();
      have_skeleton_ = false;
      grip_.reset();
      reset_hand_pose();
      trust_ = 1;
    }

    pose_.deviceIsConnected = connected_;
    pose_.poseIsValid = connected_;
    pose_.result = connected_ ? TrackingResult_Running_OK : TrackingResult_Uninitialized;
    const bool tracking = now_ns - last_valid_ns_ < 100'000'000;
    if (connected_ && tracking) {
      shown_origin_ = origin_;
      shown_direction_ = direction_;
      const double gain = settings_.predict_gain * trust_;
      if (gain > 0 && origin_x_.valid()) {
        const double age_s = double(now_ns - origin_x_.time_ns()) * 1e-9 + settings_.predict_lead_ms * 1e-3;
        const double h = std::clamp(age_s, 0.0, settings_.predict_max_ms * 1e-3) * gain;
        shown_origin_ = origin_x_.ahead(h);
        shown_direction_ = fhr::normalized(direction_x_.ahead(h));
      }
      if (have_hand_pose_) {
        shown_hand_ = {hand_position_, hand_rotation_};
        if (gain > 0 && hand_position_x_.valid()) {
          const double age_s = double(now_ns - hand_position_x_.time_ns()) * 1e-9 + settings_.predict_lead_ms * 1e-3;
          shown_hand_.p = hand_position_x_.ahead(std::clamp(age_s, 0.0, settings_.predict_max_ms * 1e-3) * gain);
        }
      }
    }
    const bool use_hand = hand_pose && have_hand_pose_;
    if (connected_ && use_hand != reported_hand_pose_) {
      driver_log("%s reports the %s pose", serial(), use_hand ? "hand" : "ray");
      reported_hand_pose_ = use_hand;
    }
    const bool system_press = connected_ && system_button;
    const bool click = connected_ && pinch_ && use != fhr::PinchUse::System && !system_press;
    const bool own_click = pinch_ && (use == fhr::PinchUse::Click || use == fhr::PinchUse::None);
    const bool laser_click = connected_ && own_click && laser_ && !laser_wait_release_;
    fhr::Xform reported{};
    if (connected_) {
      const fhr::Vec3 base_origin = shown_origin_;
      const fhr::Vec3 base_direction = shown_direction_;
      history_.push_back({now_ns, base_origin, base_direction});
      while (!history_.empty() && now_ns - history_.front().t_ns > 400'000'000) history_.pop_front();
      if (laser_click && !reported_laser_click_ && settings_.pinch_rewind_ms > 0) {
        const int64_t then = now_ns - int64_t(settings_.pinch_rewind_ms) * 1'000'000;
        const ShownPose* past = &history_.front();
        for (const ShownPose& h : history_)
          if (h.t_ns <= then) past = &h;
        offset_origin_ = past->origin - base_origin;
        offset_direction_ = past->direction - base_direction;
        offset_weight_ = 1;
      }
      if (laser_click) {
        released_ns_ = now_ns;
      } else if (offset_weight_ > 0) {
        const double since = double(now_ns - released_ns_) * 1e-6 - settings_.pinch_release_hold_ms;
        offset_weight_ = since <= 0 ? 1 : std::max(0.0, 1 - since / 200.0);
      }
      const fhr::Vec3 origin = base_origin + offset_origin_ * offset_weight_;
      const fhr::Vec3 direction = fhr::normalized(base_direction + offset_direction_ * offset_weight_);
      reported = use_hand ? shown_hand_ : fhr::Xform{origin, fhr::look_rotation(direction)};
      pose_.vecPosition[0] = reported.p.x;
      pose_.vecPosition[1] = reported.p.y;
      pose_.vecPosition[2] = reported.p.z;
      pose_.qRotation = {reported.q.w, reported.q.x, reported.q.y, reported.q.z};
      pose_.poseTimeOffset = 0;
    } else {
      history_.clear();
      offset_weight_ = 0;
    }
    const bool hide_ray = connected_ && !use_hand && !may_point;
    if (hide_ray) {
      pose_.poseIsValid = false;
      pose_.result = TrackingResult_Running_OutOfRange;
    }
    if (connected_ && hide_ray != reported_hidden_) {
      driver_log("%s ray %s", serial(), hide_ray ? "hidden (the hand is not pointing)" : "shown");
      reported_hidden_ = hide_ray;
    }
    VRServerDriverHost()->TrackedDevicePoseUpdated(object_id_, pose_, sizeof(DriverPose_t));

    if (system_press != reported_system_) {
      driver_log("%s system button %s (palm pinch)", serial(), system_press ? "down" : "up");
      reported_system_ = system_press;
    }
    if (click != reported_click_) {
      driver_log("%s pinch %s", serial(), click ? "down" : "up");
      reported_click_ = click;
    }
    if (laser_click != reported_laser_click_) reported_laser_click_ = laser_click;
    const bool fist = connected_ && have_skeleton_ && grip_.on();
    if (fist != reported_fist_) {
      driver_log("%s fist %s (curl %.2f)", serial(), fist ? "closed" : "open", curl_);
      reported_fist_ = fist;
    }
    const bool pointing = connected_ && laser_;
    if (pointing != reported_pointing_) {
      driver_log("%s %s the laser", serial(), pointing ? "carries" : "hands over");
      reported_pointing_ = pointing;
    }
    const float trigger = click ? 1.0f : connected_ ? float(std::min(pinch_value_, 0.6)) : 0.0f;
    const float grip = fist                           ? 1.0f
                       : connected_ && have_skeleton_ ? float(std::min(fhr::ramp(curl_, 0.95, 0.45), 0.6))
                                                      : 0.0f;
    const bool ready = connected_ && tracking && trust_ > 0;
    auto* in = VRDriverInput();
    in->UpdateBooleanComponent(trigger_click_, click, 0);
    in->UpdateScalarComponent(trigger_value_, trigger, 0);
    in->UpdateBooleanComponent(grip_click_, fist, 0);
    in->UpdateScalarComponent(grip_value_, grip, 0);
    const float ext_values[3] = {trigger, grip, trigger};
    for (int i = 0; i < 3; ++i) {
      in->UpdateScalarComponent(ext_value_[i], ready ? ext_values[i] : 0.0f, 0);
      in->UpdateBooleanComponent(ext_ready_[i], ready, 0);
    }
    in->UpdateBooleanComponent(laser_click_, laser_click, 0);
    in->UpdateBooleanComponent(pointing_click_, pointing, 0);
    in->UpdateBooleanComponent(system_click_, system_press, 0);
    update_skeleton(reported, use_hand, now_ns);
  }

 private:
  void reset_hand_pose() {
    finger_filter_.reset();
    hand_position_filter_.reset();
    hand_rotation_filter_.reset();
    hand_position_x_.reset();
    hand_capture_ns_ = 0;
    have_hand_pose_ = false;
  }

  void set_connected(bool on) {
    VRProperties()->SetInt32Property(container_, Prop_ControllerRoleHint_Int32,
                                     on ? role_ : TrackedControllerRole_OptOut);
    connected_ = on;
    driver_log("%s %s", serial(), on ? "connected (hand ray active)" : "disconnected (laser back to controllers)");
  }

  fhr::FingerFilterParams finger_params() const {
    fhr::FingerFilterParams p;
    p.min_cutoff_hz = settings_.finger_min_cutoff;
    p.beta = settings_.finger_beta;
    p.predict_max_s = settings_.finger_predict_max_ms * 1e-3;
    p.velocity_smoothing = settings_.velocity_smoothing;
    return p;
  }

  void update_skeleton(const fhr::Xform& reported, bool use_hand, int64_t now_ns) {
    have_world_ = false;
    if (skeleton_ == k_ulInvalidInputComponentHandle || !connected_ || !have_skeleton_) return;
    const std::array<fhr::Xform, fhr::kXrJoints> hand =
        settings_.finger_filter && finger_filter_.valid()
            ? finger_filter_.at(now_ns + int64_t(settings_.predict_lead_ms * 1e6), finger_params(),
                                settings_.predict_gain * trust_)
            : hand_in_hand_;
    std::array<fhr::Xform, fhr::kXrJoints> joints;
    if (use_hand) {
      joints = hand;
    } else {
      const fhr::Xform shown{shown_origin_, fhr::look_rotation(shown_direction_)};
      const fhr::Xform device_from_ray = fhr::inverse(reported) * shown;
      const fhr::Xform device_from_hand = device_from_ray * ray_from_hand_;
      for (int i = 0; i < fhr::kXrJoints; ++i) joints[i] = device_from_hand * hand[i];
    }
    for (int i = 0; i < fhr::kXrJoints; ++i) world_[i] = (reported * joints[i]).p;
    have_world_ = true;
    std::array<fhr::Xform, fhr::kBones> bones;
    fhr::openvr_bones(joints, right_, bones);
    VRBoneTransform_t out[fhr::kBones];
    for (int i = 0; i < fhr::kBones; ++i) {
      out[i].position = {float(bones[i].p.x), float(bones[i].p.y), float(bones[i].p.z), 1.0f};
      out[i].orientation = {float(bones[i].q.w), float(bones[i].q.x), float(bones[i].q.y), float(bones[i].q.z)};
    }
    auto* in = VRDriverInput();
    const EVRInputError e1 =
        in->UpdateSkeletonComponent(skeleton_, VRSkeletalMotionRange_WithoutController, out, fhr::kBones);
    const EVRInputError e2 =
        in->UpdateSkeletonComponent(skeleton_, VRSkeletalMotionRange_WithController, out, fhr::kBones);
    if ((e1 != VRInputError_None || e2 != VRInputError_None) && !logged_skeleton_error_) {
      driver_log("%s: UpdateSkeletonComponent failed (%d, %d)", serial(), int(e1), int(e2));
      logged_skeleton_error_ = true;
    }
    if (!logged_skeleton_) {
      driver_log("%s: first skeleton (wrist %.3f %.3f %.3f m from the device)", serial(), bones[1].p.x, bones[1].p.y,
                 bones[1].p.z);
      logged_skeleton_ = true;
    }
  }

  bool right_;
  ETrackedControllerRole role_;
  const Settings& settings_;
  uint32_t object_id_ = k_unTrackedDeviceIndexInvalid;
  PropertyContainerHandle_t container_ = k_ulInvalidPropertyContainer;
  VRInputComponentHandle_t trigger_click_ = k_ulInvalidInputComponentHandle,
                           trigger_value_ = k_ulInvalidInputComponentHandle,
                           grip_click_ = k_ulInvalidInputComponentHandle, grip_value_ = k_ulInvalidInputComponentHandle,
                           laser_click_ = k_ulInvalidInputComponentHandle,
                           pointing_click_ = k_ulInvalidInputComponentHandle,
                           system_click_ = k_ulInvalidInputComponentHandle, skeleton_ = k_ulInvalidInputComponentHandle;
  VRInputComponentHandle_t ext_value_[3] = {k_ulInvalidInputComponentHandle, k_ulInvalidInputComponentHandle,
                                            k_ulInvalidInputComponentHandle};
  VRInputComponentHandle_t ext_ready_[3] = {k_ulInvalidInputComponentHandle, k_ulInvalidInputComponentHandle,
                                            k_ulInvalidInputComponentHandle};
  DriverPose_t pose_{};
  fhr::OneEuro anchor_filter_, direction_filter_;
  fhr::Vec3 origin_, direction_{0, 0, -1}, shown_origin_, shown_direction_{0, 0, -1};
  fhr::Extrapolator origin_x_, direction_x_;
  uint32_t last_sequence_ = 0;
  int valid_streak_ = 0, source_ = -1;
  struct ShownPose {
    int64_t t_ns;
    fhr::Vec3 origin, direction;
  };
  std::deque<ShownPose> history_;
  fhr::Vec3 offset_origin_, offset_direction_;
  double offset_weight_ = 0;
  int64_t released_ns_ = 0;
  bool wait_release_ = false, laser_ = false, laser_wait_release_ = false;
  int64_t last_valid_ns_ = 0, last_capture_ns_ = 0;
  bool connected_ = false, pinch_ = false, reported_click_ = false, reported_laser_click_ = false;
  bool yielded_ = false;
  int priority_ = 0;
  bool reported_fist_ = false, reported_pointing_ = false, reported_system_ = false, reported_hidden_ = false;
  double pinch_value_ = 0, curl_ = 1;
  fhr::Hysteresis grip_;
  std::array<fhr::Xform, fhr::kXrJoints> hand_in_hand_;
  fhr::Xform ray_from_hand_;
  fhr::FingerFilter finger_filter_;
  fhr::OneEuro hand_position_filter_;
  fhr::OneEuroRotation hand_rotation_filter_;
  fhr::Extrapolator hand_position_x_;
  fhr::Vec3 hand_position_;
  fhr::Quat hand_rotation_;
  fhr::Xform shown_hand_;
  int64_t hand_capture_ns_ = 0;
  bool have_hand_pose_ = false, reported_hand_pose_ = false;
  double trust_ = 1;
  bool have_skeleton_ = false, logged_skeleton_ = false, logged_skeleton_error_ = false;
  std::array<fhr::Vec3, fhr::kXrJoints> world_;
  bool have_world_ = false;
};

class Provider : public IServerTrackedDeviceProvider {
 public:
  EVRInitError Init(IVRDriverContext* context) override {
    VR_INIT_SERVER_DRIVER_CONTEXT(context);
    settings_.load();
    driver_log(
        "frame_hand_ray %s (poseMode %d, handSkeleton %d, singleRay %d, controllerPolicy %d, lostMs %d, "
        "connectFrames %d, pinchHoldMs %d, predictGain %.2f, filterMinCutoff %.2f, directionMinCutoff %.2f, grip "
        "%.2f/%.2f)",
        kVersion, settings_.pose_mode, settings_.hand_skeleton, settings_.single_ray, settings_.controller_policy,
        settings_.lost_ms, settings_.connect_packets, settings_.pinch_hold_ms, settings_.predict_gain,
        settings_.filter_min_cutoff, settings_.direction_min_cutoff, settings_.grip_begin, settings_.grip_end);
    if (!settings_.enable) {
      driver_log("disabled by %s.enable", kSection);
      return VRInitError_None;
    }
    if (!receiver_.start()) {
      driver_log("cannot bind @frame_hand_ray; hand ray disabled");
      return VRInitError_None;
    }
    for (int i = 0; i < 2; ++i) {
      hands_[i] = std::make_unique<HandDevice>(i == 1, settings_);
      VRServerDriverHost()->TrackedDeviceAdded(hands_[i]->serial(), TrackedDeviceClass_Controller, hands_[i].get());
    }
    outline_fd_ = socket(AF_UNIX, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    return VRInitError_None;
  }
  void Cleanup() override {
    receiver_.stop();
    if (outline_fd_ >= 0) close(outline_fd_);
    outline_fd_ = -1;
    eye_.stop();
    VR_CLEANUP_SERVER_DRIVER_CONTEXT();
  }
  const char* const* GetInterfaceVersions() override { return k_InterfaceVersions; }
  void RunFrame() override {
    if (!hands_[0]) return;
    const int64_t now = now_raw_ns();
    if (now - tune_checked_ns_ > 500'000'000) {
      tune_checked_ns_ = now;
      poll_tune_file();
    }
    if (now - role_checked_ns_ > 1'000'000'000) {
      role_checked_ns_ = now;
      if (settings_.single_ray && !settings_.hand_skeleton)
        choose_ray_role();
      else
        hands_[1]->set_ray_role(TrackedControllerRole_RightHand);
    }
    TrackedDevicePose_t poses[k_unMaxTrackedDeviceCount]{};
    VRServerDriverHost()->GetRawTrackedDevicePoses(0, poses, k_unMaxTrackedDeviceCount);
    const TrackedDevicePose_t& hmd = poses[k_unTrackedDeviceIndex_Hmd];
    if (hmd.bPoseIsValid) {
      const auto& m = hmd.mDeviceToAbsoluteTracking.m;
      const double r[3][3] = {{m[0][0], m[0][1], m[0][2]}, {m[1][0], m[1][1], m[1][2]}, {m[2][0], m[2][1], m[2][2]}};
      history_.push(now, {{m[0][3], m[1][3], m[2][3]}, fhr::quat_from_matrix(r)});
    }
    fhr::Packet packet;
    int64_t received = 0;
    const bool have = receiver_.latest(packet, received) && !history_.empty();
    watch_controllers(poses, have ? &packet : nullptr, now);
    fhr::Ray eye_ray;
    const fhr::Ray* eye_look = nullptr;
    if (settings_.intent_gaze) {
      if (!eye_.running()) {
        eye_.start();
        driver_log("eye tracker on (laser by intent)");
      }
      if (update_gaze(now, eye_ray)) eye_look = &eye_ray;
    } else if (eye_.running()) {
      eye_.stop();
      gaze_filter_.reset();
      have_gaze_ = false;
      gaze_sample_ns_ = 0;
      driver_log("eye tracker off");
    }
    int source[2] = {0, 1};
    bool laser[2] = {true, true};
    if (have && packet.sequence != posed_sequence_) {
      posed_sequence_ = packet.sequence;
      fhr::PoseParams pp;
      pp.point_min_palm_deg = settings_.point_min_palm_deg;
      pp.point_max_down_deg = settings_.point_max_down_deg;
      pp.point_hide_s = settings_.point_hide_ms * 1e-3;
      pp.system_enter_deg = settings_.system_enter_deg;
      pp.system_exit_deg = settings_.system_enter_deg + 20;
      pp.system_view_enter_deg = settings_.system_max_view_deg;
      pp.system_view_exit_deg = settings_.system_max_view_deg + 10;
      pp.system_arm_s = settings_.system_arm_ms * 1e-3;
      pp.system_hold_s = settings_.system_hold_ms * 1e-3;
      fhr::IntentParams ip;
      ip.enter_deg = settings_.intent_cone_deg;
      ip.exit_deg = std::max(settings_.intent_exit_deg, settings_.intent_cone_deg);
      ip.show_s = settings_.intent_show_ms * 1e-3;
      ip.hide_s = settings_.intent_hide_ms * 1e-3;
      const fhr::Pose head = history_.at(packet.capture_raw_ns);
      const fhr::Vec3 look = eye_look ? eye_look->direction : fhr::rotate(head.rotation, fhr::Vec3{0, 0, -1});
      for (int i = 0; i < 2; ++i) {
        const fhr::HandSample& h = packet.hands[i];
        posed_[i] = h.valid && h.has_joints;
        if (!posed_[i]) {
          intent_[i].reset();
          continue;
        }
        const fhr::Ray ray = fhr::hand_ray(head, h, i == 1, settings_.ray);
        const double pitch = std::asin(std::clamp(ray.direction.y, -1.0, 1.0)) * 180 / M_PI;
        poses_[i].update(h.joints_head, h.pinch, packet.capture_raw_ns, pp, pitch);
        const bool holding = settings_.system_gesture && poses_[i].pinch_use() == fhr::PinchUse::System;
        if (holding && !holding_[i]) {
          driver_log("%s hand: system pinch, held %.1f s presses the system button", i ? "right" : "left",
                     pp.system_hold_s);
        } else if (!holding && holding_[i] && !hold_fired_[i]) {
          driver_log("%s hand: system pinch let go after %.1f s: no press", i ? "right" : "left",
                     hold_progress_[i] * pp.system_hold_s);
        }
        holding_[i] = holding;
        hold_fired_[i] = holding && poses_[i].system_fired();
        hold_progress_[i] = poses_[i].system_progress();
        const double c = fhr::dot(fhr::normalized(ray.direction), fhr::normalized(look));
        const double off_deg = std::acos(std::clamp(c, -1.0, 1.0)) * 180 / M_PI;
        const bool was = intent_[i].shown();
        const double trust = fhr::confidence_trust(h.confidence, settings_.confidence_low, settings_.confidence_high);
        if (h.confidence >= 0) {
          confidence_sum_[i] += h.confidence;
          ++confidence_n_[i];
          untrusted_n_[i] += trust <= 0;
        }
        intent_[i].update(poses_[i].pose() == fhr::HandPose::Point && trust > 0, off_deg, h.pinch,
                          packet.capture_raw_ns, ip);
        if (intent_[i].shown() != was) {
          const bool pointing = poses_[i].pose() == fhr::HandPose::Point;
          driver_log("%s hand %s (ray %.0f deg from %s%s)", i ? "right" : "left",
                     intent_[i].shown() ? "shows the laser: it points where you look" : "hides the laser", off_deg,
                     eye_look ? "the gaze" : "straight ahead",
                     intent_[i].shown() ? ""
                     : pointing         ? ": looking elsewhere"
                                        : "; the pointing pose ended");
        }
      }
    }
    bool may_point[2] = {true, true}, system_button[2] = {false, false};
    fhr::PinchUse use[2] = {fhr::PinchUse::None, fhr::PinchUse::None};
    for (int i = 0; i < 2; ++i) {
      if (!posed_[i]) continue;
      use[i] = poses_[i].pinch_use();
      if (use[i] == fhr::PinchUse::System && !settings_.system_gesture) use[i] = fhr::PinchUse::Click;
      system_button[i] = settings_.system_gesture && poses_[i].system_pressed();
      may_point[i] = intent_[i].shown();
    }
    if (settings_.single_ray) {
      if (have && packet.sequence != selected_sequence_) {
        selected_sequence_ = packet.sequence;
        selector_.update(packet, now, may_point);
      }
      if (settings_.hand_skeleton) {
        laser[0] = selector_.active() == 0 && may_point[0];
        laser[1] = selector_.active() == 1 && may_point[1];
      } else {
        source[0] = -1;
        source[1] = selector_.active();
        laser[0] = false;
        laser[1] = source[1] >= 0 && may_point[source[1]];
      }
    }
    const bool hand_pose = settings_.hand_skeleton &&
                           (settings_.pose_mode == 1 || (settings_.pose_mode == 2 && scene_app_ && !dashboard_));
    for (int i = 0; i < 2; ++i)
      hands_[i]->update(have ? &packet : nullptr, received, history_, now, source[i], laser[i], hand_pose,
                        source[i] < 0 || may_point[source[i]], source[i] < 0 ? fhr::PinchUse::None : use[source[i]],
                        source[i] >= 0 && system_button[source[i]]);
    if (settings_.hand_outline && settings_.hand_skeleton) send_outline(now, laser, use);
    if (have && packet.sequence != aged_sequence_) {
      aged_sequence_ = packet.sequence;
      const double age = double(received - packet.capture_raw_ns) * 1e-6;
      const double sent = double(packet.send_raw_ns - packet.capture_raw_ns) * 1e-6;
      if (age > 0 && age < 1000) {
        age_sum_ += age;
        sent_sum_ += sent;
        age_max_ = std::max(age_max_, age);
        ++age_n_;
      }
      if (now - age_logged_ns_ > 10'000'000'000LL && age_n_) {
        char confidence[160] = "";
        for (int i = 0; i < 2; ++i) {
          if (!confidence_n_[i]) continue;
          const size_t used = std::strlen(confidence);
          std::snprintf(confidence + used, sizeof confidence - used, "; %s confidence %.2f (%.0f%% untrusted)",
                        i ? "right" : "left", confidence_sum_[i] / confidence_n_[i],
                        100.0 * untrusted_n_[i] / confidence_n_[i]);
        }
        driver_log("data age: camera -> driver mean %.1f ms (max %.1f), camera -> worker send %.1f ms, %d packets%s",
                   age_sum_ / age_n_, age_max_, sent_sum_ / age_n_, age_n_, confidence);
        age_logged_ns_ = now;
        age_sum_ = sent_sum_ = age_max_ = 0;
        age_n_ = 0;
        for (int i = 0; i < 2; ++i) {
          confidence_sum_[i] = 0;
          confidence_n_[i] = untrusted_n_[i] = 0;
        }
      }
    }
    if (have && !logged_first_) {
      driver_log("first hand packet received (seq %u)", packet.sequence);
      logged_first_ = true;
    }
    if (receiver_.rejected() != logged_rejected_ && now - rejected_logged_ns_ >= kRejectedLogNs) {
      logged_rejected_ = receiver_.rejected();
      rejected_logged_ns_ = now;
      driver_log("rejected datagrams: %llu so far (wrong user, size or version)", (unsigned long long)logged_rejected_);
    }
    VREvent_t event;
    while (VRServerDriverHost()->PollNextEvent(&event, sizeof event)) handle_event(event);
  }
  bool ShouldBlockStandbyMode() override { return false; }

  void handle_event(const VREvent_t& e) {
    if (seen_events_.size() < 64 && seen_events_.insert(e.eventType).second)
      driver_log("event type %u first seen (device %u)", e.eventType, e.trackedDeviceIndex);
    switch (e.eventType) {
      case VREvent_SceneApplicationChanged:
        scene_app_ = e.data.process.pid != 0;
        driver_log("scene application changed: pid %u (old %u) -> %s", e.data.process.pid, e.data.process.oldPid,
                   scene_app_ ? "app running" : "no app");
        break;
      case VREvent_DashboardActivated:
        dashboard_ = true;
        driver_log("dashboard opened");
        break;
      case VREvent_DashboardDeactivated:
        dashboard_ = false;
        driver_log("dashboard closed");
        break;
      case VREvent_TrackedDeviceRoleChanged:
        driver_log("SteamVR changed a hand role (device %u)", e.trackedDeviceIndex);
        break;
      default:
        break;
    }
  }

  struct Known {
    bool checked = false, controller = false;
    int side = -1, logged_side = -2;
    int64_t checked_ns = 0;
  };
  void watch_controllers(const TrackedDevicePose_t* poses, const fhr::Packet* packet, int64_t now) {
    if (settings_.controller_policy != 1) {
      for (int s = 0; s < 2; ++s) {
        presence_[s].reset();
        hands_[s]->set_yield(false, "");
      }
      return;
    }
    fhr::Vec3 hand_points[2][2];
    int hand_count[2] = {0, 0};
    if (packet) {
      const fhr::Pose head = history_.at(packet->capture_raw_ns);
      for (int s = 0; s < 2; ++s) {
        const fhr::HandSample& h = packet->hands[s];
        if (!h.valid) continue;
        hand_points[s][hand_count[s]++] = head.position + fhr::rotate(head.rotation, h.wrist_head);
        if (h.has_joints)
          hand_points[s][hand_count[s]++] = head.position + fhr::rotate(head.rotation, h.joints_head[fhr::kPalm].p);
      }
    }
    bool seen[2] = {false, false}, tracked[2] = {false, false};
    double speed[2] = {0, 0}, distance[2] = {-1, -1};
    for (uint32_t d = 1; d < k_unMaxTrackedDeviceCount; ++d) {
      if (d == hands_[0]->id() || d == hands_[1]->id()) continue;
      const TrackedDevicePose_t& p = poses[d];
      Known& k = known_[d];
      if (!p.bDeviceIsConnected) {
        k.checked = false;
        continue;
      }
      if (!k.checked || now - k.checked_ns > 5'000'000'000LL) identify(d, k, now);
      if (!k.controller || k.side < 0) continue;
      const int s = k.side;
      seen[s] = true;
      const bool ok = p.bPoseIsValid && p.eTrackingResult == TrackingResult_Running_OK;
      tracked[s] = tracked[s] || ok;
      const auto& v = p.vVelocity.v;
      speed[s] = std::max(speed[s], std::sqrt(double(v[0]) * v[0] + double(v[1]) * v[1] + double(v[2]) * v[2]));
      if (ok)
        for (int j = 0; j < hand_count[s]; ++j) {
          const auto& m = p.mDeviceToAbsoluteTracking.m;
          const double dd = fhr::norm(fhr::Vec3{m[0][3], m[1][3], m[2][3]} - hand_points[s][j]);
          distance[s] = distance[s] < 0 ? dd : std::min(distance[s], dd);
        }
    }
    fhr::ControllerPresenceParams cp;
    cp.move_speed_m_s = settings_.controller_move_speed;
    cp.quiet_ns = int64_t(settings_.controller_quiet_ms) * 1'000'000;
    cp.hold_m = settings_.controller_hold_m;
    for (int s = 0; s < 2; ++s) {
      const bool in_use = presence_[s].update(seen[s], tracked[s], speed[s], distance[s], now, cp);
      char why[160];
      if (distance[s] >= 0)
        std::snprintf(why, sizeof why, "yields: its controller is %s (%.2f m from the hand, %.2f m/s)",
                      presence_[s].held()     ? "in the hand"
                      : presence_[s].moving() ? "moving"
                                              : "in use",
                      distance[s], speed[s]);
      else
        std::snprintf(why, sizeof why, "yields: its controller is %s (hand not seen, %.2f m/s)",
                      presence_[s].held()     ? "in the hand"
                      : presence_[s].moving() ? "moving"
                                              : "in use",
                      speed[s]);
      hands_[s]->set_yield(in_use, why);
    }
  }
  void identify(uint32_t d, Known& k, int64_t now) {
    k.checked = true;
    k.checked_ns = now;
    k.controller = false;
    k.side = -1;
    const PropertyContainerHandle_t c = VRProperties()->TrackedDeviceToPropertyContainer(d);
    ETrackedPropertyError e = TrackedProp_Success;
    if (VRProperties()->GetInt32Property(c, Prop_DeviceClass_Int32, &e) != TrackedDeviceClass_Controller ||
        e != TrackedProp_Success)
      return;
    const std::string type = VRProperties()->GetStringProperty(c, Prop_ControllerType_String, &e);
    if (e != TrackedProp_Success || type == "frame_hand_ray") return;
    const int32_t hint = VRProperties()->GetInt32Property(c, Prop_ControllerRoleHint_Int32, &e);
    if (e != TrackedProp_Success) return;
    k.controller = true;
    k.side = hint == TrackedControllerRole_LeftHand ? 0 : hint == TrackedControllerRole_RightHand ? 1 : -1;
    if (k.side != k.logged_side) {
      k.logged_side = k.side;
      driver_log("real controller: device %u, %s, %s", d, type.c_str(),
                 k.side < 0 ? "no hand hint (ignored)"
                 : k.side   ? "right hand"
                            : "left hand");
    }
  }

  void choose_ray_role() {
    TrackedDevicePose_t poses[k_unMaxTrackedDeviceCount]{};
    VRServerDriverHost()->GetRawTrackedDevicePoses(0, poses, k_unMaxTrackedDeviceCount);
    bool taken[3] = {false, false, false};
    std::string seen;
    for (uint32_t i = 1; i < k_unMaxTrackedDeviceCount; ++i) {
      if (!poses[i].bDeviceIsConnected || i == hands_[0]->id() || i == hands_[1]->id()) continue;
      const PropertyContainerHandle_t c = VRProperties()->TrackedDeviceToPropertyContainer(i);
      ETrackedPropertyError e = TrackedProp_Success;
      const int32_t cls = VRProperties()->GetInt32Property(c, Prop_DeviceClass_Int32, &e);
      if (e != TrackedProp_Success || cls != TrackedDeviceClass_Controller) continue;
      const int32_t role = VRProperties()->GetInt32Property(c, Prop_ControllerRoleHint_Int32, &e);
      if (e != TrackedProp_Success) continue;
      if (role == TrackedControllerRole_LeftHand || role == TrackedControllerRole_RightHand) taken[role] = true;
      seen += " " + std::to_string(i) + ":" + std::to_string(role);
    }
    const ETrackedControllerRole want = !taken[TrackedControllerRole_RightHand]  ? TrackedControllerRole_RightHand
                                        : !taken[TrackedControllerRole_LeftHand] ? TrackedControllerRole_LeftHand
                                                                                 : TrackedControllerRole_RightHand;
    if (seen != roles_seen_) {
      driver_log("real controllers (device:role):%s", seen.empty() ? " none" : seen.c_str());
      roles_seen_ = seen;
    }
    hands_[1]->set_ray_role(want);
  }

  bool update_gaze(int64_t now, fhr::Ray& out) {
    fhr::GazeSample s;
    const bool fresh = eye_.latest(s) && now - s.received_ns < int64_t(settings_.gaze_stale_ms) * 1'000'000;
    if (fresh != gaze_fresh_) {
      gaze_fresh_ = fresh;
      if (fresh)
        driver_log("eye gaze: receiving");
      else
        driver_log("eye gaze: none (%s); the intent uses the head direction",
                   eye_.problem().empty() ? "no recent sample" : eye_.problem().c_str());
    }
    if (!fresh || history_.empty()) return false;
    if (s.received_ns != gaze_sample_ns_) {
      const double dt = gaze_sample_ns_ ? double(s.received_ns - gaze_sample_ns_) * 1e-9 : 0;
      gaze_sample_ns_ = s.received_ns;
      if (s.openness >= settings_.gaze_blink_openness) {
        gaze_filter_.set(settings_.gaze_min_cutoff, settings_.gaze_beta);
        gaze_head_ = fhr::normalized(gaze_filter_.filter(s.direction, dt));
        have_gaze_ = true;
      }
    }
    if (!have_gaze_) return false;
    const double yaw = std::atan2(gaze_head_.x, -gaze_head_.z) + settings_.gaze_yaw_offset_deg * M_PI / 180;
    const double pitch =
        std::atan2(gaze_head_.y, std::hypot(gaze_head_.x, gaze_head_.z)) + settings_.gaze_pitch_offset_deg * M_PI / 180;
    const fhr::Vec3 dir{std::sin(yaw) * std::cos(pitch), std::sin(pitch), -std::cos(yaw) * std::cos(pitch)};
    const fhr::Pose head = history_.at(now);
    out.origin = head.position;
    out.direction = fhr::normalized(fhr::rotate(head.rotation, dir));
    return true;
  }

  void poll_tune_file() {
    const char* dir = std::getenv("XDG_RUNTIME_DIR");
    const std::string base = dir && *dir ? std::string(dir) : "/run/user/" + std::to_string(getuid());
    const std::string path = base + "/frame_hand_ray.tune";
    struct stat file_status{};
    if (stat(path.c_str(), &file_status) != 0) return;
    const int64_t stamp = int64_t(file_status.st_mtim.tv_sec) * 1'000'000'000 + file_status.st_mtim.tv_nsec;
    if (stamp == tune_stamp_) return;
    tune_stamp_ = stamp;
    std::string text;
    if (!read_own_small_file(path, kMaxTuneFileBytes, text)) {
      driver_log("tune file %s: not read (it must be a regular file of this user, at most %zu bytes)", path.c_str(),
                 kMaxTuneFileBytes);
      return;
    }
    driver_log("tune file %s:%s", path.c_str(), settings_.apply(text).c_str());
    for (auto& hand : hands_) hand->set_priority(settings_.hand_selection_priority());
  }
  void EnterStandby() override {}
  void LeaveStandby() override {}

 private:
  void send_outline(int64_t now, const bool laser[2], const fhr::PinchUse use[2]) {
    if (outline_fd_ < 0) return;
    fho::Packet p;
    p.time_ns = now;
    for (int i = 0; i < 2; ++i) {
      std::array<fhr::Vec3, fhr::kXrJoints> joints;
      fho::Hand& h = p.hands[i];
      h.valid = hands_[i]->connected() && hands_[i]->world_joints(joints);
      if (!h.valid) continue;
      for (int j = 0; j < fhr::kXrJoints; ++j)
        h.joints[j] = {float(joints[j].x), float(joints[j].y), float(joints[j].z)};
      const bool hold = settings_.system_gesture && use[i] == fhr::PinchUse::System;
      h.flags = (laser[i] ? fho::kLaser : 0) | (poses_[i].pose() == fhr::HandPose::System ? fho::kSystemArmed : 0) |
                (hold && poses_[i].system_fired() ? fho::kSystemDown : 0);
      h.hold = hold ? uint8_t(std::lround(poses_[i].system_progress() * 255)) : 0;
    }
    uint8_t buf[fho::kBytes];
    fho::encode(p, buf);
    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    std::memcpy(addr.sun_path + 1, fho::kSocketName, sizeof(fho::kSocketName) - 1);
    const socklen_t len = socklen_t(offsetof(sockaddr_un, sun_path) + sizeof(fho::kSocketName));
    sendto(outline_fd_, buf, sizeof buf, MSG_DONTWAIT | MSG_NOSIGNAL, reinterpret_cast<const sockaddr*>(&addr), len);
  }

  Settings settings_;
  PacketReceiver receiver_;
  fhr::PoseHistory history_;
  std::unique_ptr<HandDevice> hands_[2];
  fhr::RaySelector selector_;
  fhr::HandPoseTracker poses_[2];
  fhr::RayIntent intent_[2];
  bool posed_[2] = {false, false};
  fhr::ControllerPresence presence_[2];
  bool holding_[2] = {false, false}, hold_fired_[2] = {false, false};
  double hold_progress_[2] = {0, 0};
  Known known_[k_unMaxTrackedDeviceCount];
  int outline_fd_ = -1;
  uint32_t selected_sequence_ = 0, aged_sequence_ = 0, posed_sequence_ = 0;
  int64_t tune_checked_ns_ = 0, tune_stamp_ = 0, age_logged_ns_ = 0, role_checked_ns_ = 0;
  std::string roles_seen_ = "?";
  double age_sum_ = 0, sent_sum_ = 0, age_max_ = 0;
  int age_n_ = 0;
  double confidence_sum_[2] = {0, 0};
  int confidence_n_[2] = {0, 0}, untrusted_n_[2] = {0, 0};
  bool logged_first_ = false;
  uint64_t logged_rejected_ = 0;
  int64_t rejected_logged_ns_ = 0;
  bool scene_app_ = false, dashboard_ = false;
  std::set<uint32_t> seen_events_;
  fhr::EyeGaze eye_;
  fhr::OneEuro gaze_filter_;
  fhr::Vec3 gaze_head_{0, 0, -1};
  int64_t gaze_sample_ns_ = 0;
  bool have_gaze_ = false, gaze_fresh_ = false;
};

Provider g_provider;

}  // namespace

// NOLINTNEXTLINE(readability-identifier-naming)
extern "C" __attribute__((visibility("default"))) void* HmdDriverFactory(const char* interface_name, int* return_code) {
  if (std::strcmp(interface_name, IServerTrackedDeviceProvider_Version) == 0) return &g_provider;
  if (return_code) *return_code = VRInitError_Init_InterfaceNotFound;
  return nullptr;
}
