#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>
#include <thread>

#include "hand_outline.h"
#include "hand_ui.h"
#include "service.h"
#include "vr_panel.h"

using doodle::Button;
using doodle::Canvas;

namespace {

volatile std::sig_atomic_t g_stop = 0;
void on_signal(int) { g_stop = 1; }

constexpr int kPanelWidth = 1000, kPanelHeight = 410;
constexpr int kOutlineSize = 192;
constexpr int64_t kOutlineMinPaintNs = 22'000'000;
constexpr double kOutlineRedrawPx = 0.75;
constexpr uint32_t kFrameSyncTimeoutMs = 25;
constexpr int kThumbnailSize = 256;

using Clock = std::chrono::steady_clock;

struct Pointer {
  int hover = -1, pressed = -1;
  int handle(const PointerEvent& e, const std::vector<Button>& buttons, bool& redraw) {
    int at = -1;
    for (size_t i = 0; i < buttons.size(); ++i)
      if (buttons[i].contains(e.x, e.y)) at = int(i);
    switch (e.type) {
      case PointerEvent::Type::Move:
        break;
      case PointerEvent::Type::Down:
        pressed = at;
        break;
      case PointerEvent::Type::Up: {
        const int clicked = pressed >= 0 && pressed == at ? at : -1;
        pressed = -1;
        if (at != hover) {
          hover = at;
          redraw = true;
        }
        return clicked;
      }
      case PointerEvent::Type::Leave:
        at = -1;
        pressed = -1;
        break;
    }
    if (at != hover) {
      hover = at;
      redraw = true;
    }
    return -1;
  }
};

std::string read_outline_mode() {
  const char* runtime = std::getenv("XDG_RUNTIME_DIR");
  if (runtime == nullptr) return "";
  std::ifstream in(std::string(runtime) + "/frame-hand-tracking/outline-mode");
  std::string mode;
  in >> mode;
  return mode;
}

bool same(const HandState& a, const HandState& b) {
  return a.outline == b.outline && a.mask == b.mask && a.system_hold_s == b.system_hold_s &&
         a.installed == b.installed && a.active == b.active && a.enabled == b.enabled && a.switching == b.switching &&
         a.problem == b.problem && a.worker.fresh == b.worker.fresh && a.worker.state == b.worker.state &&
         a.worker.message == b.worker.message && a.worker.hands == b.worker.hands &&
         int(a.worker.rate_hz) == int(b.worker.rate_hz) && a.worker.left == b.worker.left &&
         a.worker.right == b.worker.right;
}

int preview(const std::string& dir) {
  Canvas panel(kPanelWidth, kPanelHeight), icon(kThumbnailSize, kThumbnailSize);
  std::vector<Button> buttons;
  HandState s;
  s.installed = true;
  const auto write = [&](const char* name, int hover) {
    draw_panel(panel, s, buttons, hover);
    const std::string path = dir + "/" + name + ".png";
    panel.write_png(path);
    std::printf("wrote %s\n", path.c_str());
  };
  s.active = "inactive";
  write("panel-off", 0);
  s.active = "active";
  s.enabled = true;
  s.worker = {true, "streaming", "streaming", 1, 33.1, "tracked", "too_close"};
  write("panel-tracking", -1);
  s.worker = {true, "waiting", "too dark: the cameras see no bright frames", 0, 0, "too_dark", "too_dark"};
  write("panel-too-dark", 0);
  s.worker = {true, "waiting", "waiting for camera permission: allow this app in the prompt in the headset", 0, 0,
              "",   ""};
  write("panel-waiting", 0);
  draw_thumbnail(icon);
  icon.write_png(dir + "/icon.png");
  std::printf("wrote %s/icon.png\n", dir.c_str());
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  std::string unit = "frame-hand-tracking.service", preview_dir;
  for (int i = 1; i < argc; ++i) {
    if (!std::strcmp(argv[i], "--unit") && i + 1 < argc)
      unit = argv[++i];
    else if (!std::strcmp(argv[i], "--preview") && i + 1 < argc)
      preview_dir = argv[++i];
    else {
      std::fprintf(stderr,
                   "usage: frame-hand-tracking-panel [--unit NAME.service] [--preview DIR]\n"
                   "  --preview DIR  draw the panel in a few states and the icon as PNGs in DIR, then exit\n");
      return 2;
    }
  }
  if (!preview_dir.empty()) return preview(preview_dir);
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);

  Canvas panel(kPanelWidth, kPanelHeight), thumbnail(kThumbnailSize, kThumbnailSize),
      outline_canvas(kOutlineSize, kOutlineSize);
  OutlineReceiver outline;
  struct OutlineDrawn {
    bool valid = false;
    uint8_t flags = 0, hold = 0;
    bool mask = false;
    OutlineLayout layout;
    int64_t painted_ns = 0;
  };
  OutlineDrawn outline_drawn[6];
  int outline_mode = 0;
  Clock::time_point next_outline_mode{};
  int outline_frames = 0, outline_paints = 0;
  int64_t outline_paint_ns = 0, outline_logged_ns = 0;
  fho::Packet hands{};
  int64_t hands_ns = 0;
  std::vector<Button> buttons;
  Pointer pointer;
  Service service(unit);
  HandState state, shown;
  VrPanel vr;
  std::string last_logged;
  bool dirty = true, was_visible = false;
  Clock::time_point next_vr_try{}, next_refresh{};

  const auto log_once = [&](const std::string& message) {
    if (message != last_logged) std::fprintf(stderr, "%s\n", message.c_str());
    last_logged = message;
  };

  while (!g_stop) {
    const Clock::time_point now = Clock::now();

    if (!vr.connected() && now >= next_vr_try) {
      std::string message;
      switch (vr.connect(kPanelWidth, kPanelHeight, message)) {
        case VrPanel::Connect::Ok:
          draw_thumbnail(thumbnail);
          vr.set_thumbnail(thumbnail.rgba(), kThumbnailSize);
          state.outline = vr.outline();
          state.mask = vr.mask();
          state.system_hold_s = vr.system_hold_s();
          set_driver_tune("handOutline", state.outline ? "1" : "0");
          dirty = true;
          break;
        case VrPanel::Connect::NotRunning:
          log_once("waiting for SteamVR");
          next_vr_try = now + std::chrono::seconds(3);
          break;
        case VrPanel::Connect::Error:
          log_once("SteamVR: " + message);
          next_vr_try = now + std::chrono::seconds(5);
          break;
      }
    }
    if (!vr.connected()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      continue;
    }

    const VrInput input = vr.poll();
    if (input.quit) {
      vr.shutdown();
      next_vr_try = now + std::chrono::seconds(3);
      continue;
    }
    for (const PointerEvent& e : input.panel) {
      const int clicked = pointer.handle(e, buttons, dirty);
      if (clicked < 0) continue;
      const std::string& decision = buttons[size_t(clicked)].decision;
      if (decision == "outline:on" || decision == "outline:off" || decision == "outline:mask") {
        const bool on = decision != "outline:off", mask = decision == "outline:mask";
        if (on != state.outline || mask != state.mask) {
          if (on != state.outline && !set_driver_tune("handOutline", on ? "1" : "0"))
            std::fprintf(stderr, "cannot write the hand ray tune file\n");
          state.outline = on;
          state.mask = mask;
          vr.save_outline(on);
          vr.save_mask(mask);
          std::fprintf(stderr, "hands shown: %s\n", !on ? "off" : mask ? "outline and mask" : "outline");
          dirty = true;
        }
      } else if (service.start_switch(decision == "on", state)) {
        pointer = Pointer{};
        dirty = true;
      }
    }
    if (service.finished(state)) next_refresh = now;

    const bool visible = vr.visible();
    if (visible && !was_visible) {
      next_refresh = now;
      dirty = true;
    }
    was_visible = visible;
    if (now >= next_refresh) {
      service.refresh(state);
      next_refresh = now + std::chrono::milliseconds(visible || service.switching() ? 500 : 5000);
      if (!same(state, shown)) dirty = true;
    }
    if (dirty) {
      draw_panel(panel, state, buttons, pointer.hover);
      vr.set_panel(panel.rgba());
      shown = state;
      dirty = false;
    }

    if (state.outline && !outline.running() && !outline.start())
      log_once("cannot listen for the hand outline (@frame_hand_outline taken?)");
    if (!state.outline && outline.running()) outline.stop();
    if (now >= next_outline_mode) {
      next_outline_mode = now + std::chrono::seconds(1);
      const std::string mode = read_outline_mode();
      const int wanted = mode == "projection-eye" ? 1 : mode == "projection-standing" ? 2 : mode == "hidden" ? 3 : 0;
      if (wanted != outline_mode) {
        outline_mode = wanted;
        std::fprintf(stderr, "outline mode: %s\n", wanted == 0 ? "flat (one image at the hand)" : mode.c_str());
        for (OutlineDrawn& drawn : outline_drawn) drawn.valid = false;
      }
    }
    if (outline.running() && outline.poll(hands, hands_ns)) {
      float head[3], head_pose[3][4], eye_poses[2][3][4];
      const bool have_eye = outline_mode == 0 ? vr.head_position(head) : vr.eye_poses(head_pose, eye_poses);
      const bool mask = state.mask && vr.dashboard_visible();
      for (int h = 0; h < 2; ++h) {
        const bool show = hands.hands[h].valid && have_eye;
        const uint8_t flags = hands.hands[h].flags, hold = hands.hands[h].hold;
        for (int view = 0; view < 3; ++view) {
          OutlineDrawn& was = outline_drawn[3 * h + view];
          const bool used = show && outline_mode != 3 && (outline_mode == 0 ? view == 0 : view != 0);
          if (!used) {
            was.valid = false;
            continue;
          }
          const OutlineLayout layout = view == 0
                                           ? layout_hand_outline(hands.hands[h], head, kOutlineSize)
                                           : layout_hand_outline_eye(hands.hands[h], eye_poses[view - 1], kOutlineSize);
          const bool paint = !was.valid || was.flags != flags || was.hold != hold || was.mask != mask ||
                             (outline_change_px(layout, was.layout) > kOutlineRedrawPx &&
                              outline_now_ns() - was.painted_ns >= kOutlineMinPaintNs);
          ++outline_frames;
          const int64_t t0 = outline_now_ns();
          const uint8_t* image = nullptr;
          if (paint && !(view == 0 && vr.paint_outline(h, layout, flags, mask, hold))) {
            draw_hand_outline(outline_canvas, layout, flags, mask, hold);
            image = outline_canvas.rgba_premultiplied();
          }
          const bool ok = view == 0 ? vr.show_outline(h, image, kOutlineSize, layout.placement)
                                    : vr.show_outline_eye(h, view - 1, image, kOutlineSize, eye_poses[view - 1],
                                                          layout.tangents, outline_mode == 2);
          if (paint) {
            outline_paint_ns += outline_now_ns() - t0;
            ++outline_paints;
            was = {true, flags, hold, mask, layout, outline_now_ns()};
          }
          if (!ok) was.valid = false;
        }
        if (!show || outline_mode != 0) vr.hide_outline(h);
        if (!show || outline_mode == 0) vr.hide_outline_eyes(h);
      }
    }
    if (!outline.running() || outline_now_ns() - hands_ns > 150'000'000) {
      for (int h = 0; h < 2; ++h) {
        vr.hide_outline(h);
        vr.hide_outline_eyes(h);
      }
      for (OutlineDrawn& drawn : outline_drawn) drawn.valid = false;
    }
    if (outline_frames > 0 && outline_now_ns() - outline_logged_ns > 10'000'000'000LL) {
      std::fprintf(stderr, "outline: %d hand frames, %d painted (%.0f%%), %.2f ms per paint (%s)\n", outline_frames,
                   outline_paints, 100.0 * outline_paints / outline_frames,
                   outline_paints ? double(outline_paint_ns) * 1e-6 / outline_paints : 0.0,
                   vr.outline_on_gpu() ? "GPU" : "Cairo");
      outline_frames = outline_paints = 0;
      outline_paint_ns = 0;
      outline_logged_ns = outline_now_ns();
    }
    if (!(vr.outline_shown() && vr.wait_frame_sync(kFrameSyncTimeoutMs)))
      std::this_thread::sleep_for(std::chrono::milliseconds(state.outline ? 8 : visible ? 20 : 100));
  }
  vr.shutdown();
  return 0;
}
