#pragma once

#include <vector>

#include "doodle_style.h"
#include "service.h"

void draw_panel(doodle::Canvas& canvas, const HandState& state, std::vector<doodle::Button>& buttons, int hover);
void draw_thumbnail(doodle::Canvas& canvas);
