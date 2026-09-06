#pragma once
// Finding the RIGHT python on a machine that has several.
//
// "python" on PATH is not the answer and never was. A Windows box routinely
// carries a Store build, a python.org install, a conda base and whatever an
// IDE dropped in, and `pip install` into the wrong one succeeds -- it just
// installs where nothing will look. The user then has the packages and the
// error at the same time, which is the most confusing state available.
//
// THE DECIDING TEST IS voxelenv, not the version number and not PATH order.
// voxelenv is the C++ environment the trainer steps; it is a compiled
// extension, so it loads into exactly one CPython minor version. An
// interpreter that can import it is by definition the right one, and an
// interpreter that cannot is useless for training no matter what else it has.
// So every python on the machine is enumerated and asked directly.
#include <string>
#include <vector>

namespace kpy {

struct Py {
    std::string exe;             // absolute path -- never "python"
    int   major = 0, minor = 0;
    bool  runs = false;
    bool  voxelenv = false;      // can import the extension beside the exe
    bool  rl = false;            // gymnasium + stable_baselines3 + sb3_contrib
    std::string origin;          // how it was found, for the listing
};

// Every interpreter this machine can offer, deduplicated by real path and
// probed against exeDir. Ordered best-first by the ranking above.
std::vector<Py> discover(const std::string& exeDir);

// The one to use, or nullptr if none can work. Prefers an interpreter that
// imports voxelenv; among those, one that already has the RL stack.
const Py* best(const std::vector<Py>& v);

// What CPython version the packaged extension needs, from its own filename
// (voxelenv.cp311-win_amd64.pyd -> "3.11"), or "" if there is no module here.
// This is what turns "module not found" back into "wrong python".
std::string moduleAbi(const std::string& exeDir);

// pip install into ONE named interpreter, by absolute path. Returns its exit
// code. reqs may be empty, in which case the package names are used.
int install(const Py& p, const std::string& reqs);

// Prints the full listing: every python found, its version, and whether it can
// import voxelenv and the RL stack. This is the thing to look at when there
// are several and it is not obvious which one anything is using.
void report(const std::vector<Py>& v, const std::string& exeDir);

}  // namespace kpy
