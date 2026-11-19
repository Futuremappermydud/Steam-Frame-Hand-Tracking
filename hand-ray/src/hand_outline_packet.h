#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace fho {

constexpr char kMagic[4] = {'F', 'H', 'O', '1'};
constexpr uint32_t kVersion = 1;
constexpr int kJoints = 26;
constexpr const char kSocketName[] = "frame_hand_outline";

constexpr uint8_t kLaser = 1;
constexpr uint8_t kSystemArmed = 2;
constexpr uint8_t kSystemDown = 4;

struct Hand {
  bool valid = false;
  uint8_t flags = 0;
  uint8_t hold = 0;
  std::array<std::array<float, 3>, kJoints> joints{};
};
struct Packet {
  int64_t time_ns = 0;
  std::array<Hand, 2> hands{};
};
constexpr size_t kHandBytes = 4 + kJoints * 3 * 4;
constexpr size_t kBytes = 4 + 4 + 8 + 2 * kHandBytes;

inline void encode(const Packet& p, uint8_t (&out)[kBytes]) {
  std::memset(out, 0, kBytes);
  std::memcpy(out, kMagic, 4);
  std::memcpy(out + 4, &kVersion, 4);
  std::memcpy(out + 8, &p.time_ns, 8);
  for (int h = 0; h < 2; ++h) {
    uint8_t* o = out + 16 + h * kHandBytes;
    o[0] = p.hands[h].valid ? 1 : 0;
    o[1] = p.hands[h].flags;
    o[2] = p.hands[h].hold;
    std::memcpy(o + 4, p.hands[h].joints.data(), kJoints * 3 * 4);
  }
}

inline bool decode(const uint8_t* data, size_t size, Packet& p) {
  uint32_t version = 0;
  if (size != kBytes || std::memcmp(data, kMagic, 4) != 0) return false;
  std::memcpy(&version, data + 4, 4);
  if (version != kVersion) return false;
  std::memcpy(&p.time_ns, data + 8, 8);
  for (int h = 0; h < 2; ++h) {
    const uint8_t* d = data + 16 + h * kHandBytes;
    p.hands[h].valid = d[0] == 1;
    p.hands[h].flags = d[1];
    p.hands[h].hold = d[2];
    std::memcpy(p.hands[h].joints.data(), d + 4, kJoints * 3 * 4);
    for (const auto& joint : p.hands[h].joints)
      for (float v : joint)
        if (!(v > -1e4f && v < 1e4f)) p.hands[h].valid = false;
  }
  return true;
}

}  // namespace fho
