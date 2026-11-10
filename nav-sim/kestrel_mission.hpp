#pragma once
// `kestrel mission check|compile|sim|editor` -- see kestrel_mission.cpp.
#include <string>
#include <vector>

namespace kms { struct Program; }

namespace kmission {
// A .kms compiled (diagnostics printed) or a .kmb loaded and verified, exactly
// as `mission sim` and the Pi do. `kestrel sitl --script` uses this too.
bool loadAny(const std::string& path, kms::Program& out, bool quietOk);
// args: everything after `mission`. exeDir finds the visual editor.
int run(const std::vector<std::string>& args, const std::string& exeDir);
}  // namespace kmission
