#pragma once
// `kestrel sitl` -- the aircraft's own autonomy flying ArduPilot SITL, with
// the cameras rendered from SITL's true pose. See kestrel_sitl.cpp.
#include <string>
#include <vector>

namespace ksitl {
// args: everything after `sitl`.
int run(const std::vector<std::string>& args);
// The usage text, for `kestrel --help` and the window's panel.
const char* usageText();
}  // namespace ksitl
