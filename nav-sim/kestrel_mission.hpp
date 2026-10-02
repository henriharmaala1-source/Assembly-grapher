#pragma once
// `kestrel mission check|compile|sim|editor` -- see kestrel_mission.cpp.
#include <string>
#include <vector>

namespace kmission {
// args: everything after `mission`. exeDir finds the visual editor.
int run(const std::vector<std::string>& args, const std::string& exeDir);
}  // namespace kmission
