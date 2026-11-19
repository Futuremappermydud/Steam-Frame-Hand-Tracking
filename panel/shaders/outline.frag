#version 450

layout(std140, set = 0, binding = 0) uniform Hand {
  vec4 points[26];
  vec4 radius;
  vec4 more;
  vec4 ink;
  vec4 fade;
  vec4 stops;
} hand;

layout(location = 0) out vec4 colour;

const int kPalm[8] = int[8](0, 1, 2, 6, 11, 16, 21, 20);

float segment2(vec2 p, vec2 a, vec2 b) {
  const vec2 pa = p - a, ba = b - a;
  const vec2 off = pa - ba * clamp(dot(pa, ba) / max(dot(ba, ba), 1e-6), 0.0, 1.0);
  return dot(off, off);
}

void main() {
  const vec2 p = gl_FragCoord.xy;
  const vec2 wrist = hand.points[0].xy;

  float d = sqrt(segment2(p, wrist, hand.points[25].xy)) - hand.more.y;
  for (int f = 0; f < 5; ++f) {
    const int base = 5 * f;
    float d2 = f > 0 ? segment2(p, wrist, hand.points[base].xy) : 1e12;
    for (int k = 0; k < 4; ++k) d2 = min(d2, segment2(p, hand.points[base + k].xy, hand.points[base + k + 1].xy));
    d = min(d, sqrt(d2) - (f < 4 ? hand.radius[f] : hand.more.x));
  }
  float edge2 = 1e12;
  int winding = 0;
  for (int k = 0; k < 8; ++k) {
    const vec2 a = hand.points[kPalm[k]].xy, b = hand.points[kPalm[(k + 1) & 7]].xy;
    edge2 = min(edge2, segment2(p, a, b));
    const float side = (b.x - a.x) * (p.y - a.y) - (p.x - a.x) * (b.y - a.y);
    if (a.y <= p.y) {
      if (b.y > p.y && side > 0.0) ++winding;
    } else if (b.y <= p.y && side < 0.0) {
      --winding;
    }
  }
  d = min(d, winding != 0 ? -sqrt(edge2) : sqrt(edge2));

  const float wide = clamp(0.5 - (d - hand.more.z), 0.0, 1.0), inside = clamp(0.5 - d, 0.0, 1.0);
  const vec4 drawn = hand.ink * wide * (1.0 - inside) + vec4(0.0, 0.0, 0.0, hand.more.w) * inside;

  const float t = dot(p - hand.fade.xy, hand.fade.zw), at = hand.stops.x;
  const float fade = t < at ? hand.stops.y * clamp(t / at, 0.0, 1.0)
                            : mix(hand.stops.y, 1.0, clamp((t - at) / (1.0 - at), 0.0, 1.0));

  float hidden = 1.0;
  for (int f = 0; f < 5; ++f) {
    const int base = 5 * f;
    const vec2 zero = hand.points[base + 2].xy, full = hand.points[base + 3].xy, tip = hand.points[base + 4].xy;
    const float reach = (f < 4 ? hand.radius[f] : hand.more.x) + hand.more.z + 1.0;
    const float near = clamp(0.5 - (sqrt(min(segment2(p, zero, full), segment2(p, full, tip))) - reach), 0.0, 1.0);
    const vec2 axis = full - zero;
    const float shown = dot(axis, axis) > 0.25 ? clamp(dot(p - zero, axis) / dot(axis, axis), 0.0, 1.0) : 1.0;
    hidden *= 1.0 - near * shown;
  }
  colour = drawn * fade * (1.0 - hidden);
}
