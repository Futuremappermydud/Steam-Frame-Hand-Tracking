#include "hand_ui.h"

#include <cmath>
#include <cstdio>

using namespace doodle;

namespace {

struct Summary {
  Chip chip;
  std::string word;
  std::string status;
  Color status_color;
};

Summary describe(const HandState& s) {
  if (!s.installed)
    return {Chip::Off, "Not installed", "Run scripts/install-panel.sh in frame-hand-tracking.", kChalkMuted};
  if (s.switching == "on") return {Chip::Neutral, "Turning on…", "Starting the tracker.", kChalkMuted};
  if (s.switching == "off") return {Chip::Neutral, "Turning off…", "Stopping the tracker.", kChalkMuted};
  if (s.active == "failed") return {Chip::Error, "Stopped", "The tracker exited with an error.", kRed};
  if (!s.on()) return {Chip::Off, "Off", "Not running.", kChalkMuted};
  if (s.active == "deactivating") return {Chip::Neutral, "Stopping…", "Stopping the tracker.", kChalkMuted};
  if (s.active != "active") return {Chip::Warn, "Waiting", "Waiting for SteamVR.", kOrange};
  const WorkerStatus& w = s.worker;
  if (!w.fresh || w.state == "starting") return {Chip::Neutral, "Starting…", "Starting the tracker.", kChalkMuted};
  if (w.state == "streaming") return {Chip::Good, "Tracking", "I can see the cameras. All good.", kChalkMuted};
  if (w.state == "waiting") {
    const auto has = [&](const char* text) { return w.message.find(text) != std::string::npos; };
    if (has("waiting for camera permission"))
      return {Chip::Warn, "Waiting", "Allow it in the camera access prompt.", kOrange};
    if (has("camera access denied")) return {Chip::Error, "Denied", "Cameras denied. Allow it in Camera access.", kRed};
    if (has("standby")) return {Chip::Warn, "Waiting", "No camera pictures. Is the headset asleep?", kOrange};
    if (has("too dark")) return {Chip::Warn, "Too dark", "Too dark for the cameras. More light helps.", kOrange};
    return {Chip::Warn, "Waiting", w.message.empty() ? "Waiting for the cameras." : w.message, kOrange};
  }
  if (w.state == "paused") return {Chip::Warn, "Paused", "Paused.", kOrange};
  if (w.state == "error") return {Chip::Error, "Error", w.message, kRed};
  return {Chip::Neutral, w.state, w.state, kChalkMuted};
}

std::string hand_words(const std::string& status) {
  static const char* const kWords[][2] = {{"tracked", "tracked"},     {"unsure", "fingers unclear"},
                                          {"starting", "found"},      {"not_seen", "not seen"},
                                          {"not_a_hand", "unclear"},  {"lost", "lost"},
                                          {"too_far", "too far"},     {"leaving_view", "leaving view"},
                                          {"too_close", "too close"}, {"too_dark", "too dark"}};
  for (const auto& w : kWords)
    if (status == w[0]) return w[1];
  return status;
}

}  // namespace

void draw_panel(Canvas& canvas, const HandState& s, std::vector<Button>& buttons, int hover) {
  cairo_t* cr = canvas.cr();
  const double w = canvas.width(), h = canvas.height(), m = 44;
  buttons.clear();
  canvas.clear();
  page(cr, w, h);

  Summary summary = describe(s);
  if (!s.problem.empty()) summary = {Chip::Error, "Error", s.problem, kRed};
  const double title_y = 82;
  const double title_w = text(cr, "Hand tracking", m, title_y, kDisplayLg, Weight::Display, kChalk);
  const double switch_x = w - m - kSwitchWidth, switch_y = title_y - 40;
  const bool can_switch = s.installed && s.switching.empty();
  const bool shown_on = s.switching.empty() ? s.on() : s.switching == "on";
  const double word_w = text(cr, shown_on ? "On" : "Off", switch_x - 14, title_y - 9, kLabel, Weight::ExtraBold,
                             can_switch ? kChalk : kChalkDim, 0, 1);
  chip(cr, summary.word, summary.chip, m + title_w + 16, title_y - 32, switch_x - 14 - word_w - 20 - m - title_w - 16);
  toggle(cr, buttons, hover, shown_on, can_switch, switch_x, switch_y, shown_on ? "off" : "on");

  const double top = 116, row_h = 66, label_w = 220, tw = w - 2 * m, bx = m + label_w, control_h = 48, gap = 12;
  const auto row = [&](int index, const char* label) {
    const double y = top + index * row_h;
    rule(cr, m, y, tw, index == 0 ? kChalk : kChalkDim);
    text(cr, label, m, y + row_h / 2 + 7, kLabel, Weight::ExtraBold, kChalk, label_w - 16);
    return y;
  };
  double y = row(0, "Status");
  text(cr, summary.status, bx, y + row_h / 2 + 7, kBody, Weight::Bold, summary.status_color, tw - label_w);

  std::string hands = "—";
  if (s.switching.empty() && s.active == "active" && s.worker.fresh && s.worker.state == "streaming") {
    char rate[64];
    std::snprintf(rate, sizeof rate, "   ·   %.0f frames/s", s.worker.rate_hz);
    if (!s.worker.left.empty() && !s.worker.right.empty())
      hands = "Left " + hand_words(s.worker.left) + "  ·  Right " + hand_words(s.worker.right) + rate;
    else
      hands = (s.worker.hands == 0   ? std::string("none in view")
               : s.worker.hands == 1 ? "1 in view"
                                     : std::to_string(s.worker.hands) + " in view") +
              rate;
  }
  y = row(1, "Hands");
  text(cr, hands, bx, y + row_h / 2 + 7, kBody, Weight::Bold, kChalkMuted, tw - label_w);

  const double choice_w = 190;
  y = row(2, "Hands shown");
  const double by = y + (row_h - control_h) / 2;
  const int shown = !s.outline ? 0 : s.mask ? 2 : 1;
  const char* const labels[3] = {"Off", "Outline", "Mask"};
  const char* const decisions[3] = {"outline:off", "outline:on", "outline:mask"};
  for (int i = 0; i < 3; ++i)
    button(cr, buttons, hover, labels[i], shown == i ? Style::Selected : Style::Outline, bx + i * (choice_w + gap), by,
           choice_w, control_h, decisions[i], "", (i % 2 == 1) != (shown == i));
  rule(cr, m, top + 3 * row_h, tw);

  const double hint_y = top + 3 * row_h + 34;
  text(cr, "Point at what you look at: the laser shows. Pinch: click.", m, hint_y, kCaption, Weight::Bold, kChalkMuted,
       tw);
  char palm[96];
  std::snprintf(palm, sizeof palm, "Look at your open palm, pinch and hold %.3g s: dashboard.", s.system_hold_s);
  text(cr, palm, m, hint_y + 25, kCaption, Weight::Bold, kChalkMuted, tw);
}

void draw_thumbnail(Canvas& canvas) { sticker_icon(canvas, Pose::ThumbsUp); }
