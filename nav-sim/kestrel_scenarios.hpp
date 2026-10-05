#pragma once
// `kestrel mission scenarios FILE.kms` -- the behaviour, SEEN from the
// aircraft. See kestrel_scenarios.cpp.
#include <string>
#include <vector>

namespace kscen {
// args: everything after `scenarios`. Returns 0 when every run passed.
int run(const std::vector<std::string>& args);
}  // namespace kscen
