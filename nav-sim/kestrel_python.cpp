#include "kestrel_python.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <sstream>

namespace fs = std::filesystem;

#ifdef _WIN32
#define POPEN  _popen
#define PCLOSE _pclose
#define NULLDEV "NUL"
#else
#define POPEN  popen
#define PCLOSE pclose
#define NULLDEV "/dev/null"
#endif

namespace kpy {
namespace {

// cmd.exe strips the first and last quote of its argument when the string
// begins with one, so a quoted interpreter path plus a quoted -c argument
// comes apart. Wrapping the WHOLE line in one more pair is the documented way
// round it. On POSIX the string is already correct.
std::string shell(const std::string& cmd) {
#ifdef _WIN32
    return "\"" + cmd + "\"";
#else
    return cmd;
#endif
}

// NOT called `quoted`. It was, and MSVC rejected it: the argument is a
// std::string, so ADL adds std::quoted (declared transitively via <sstream>)
// to the overload set, and MSVC resolved the call to the stream manipulator --
// the error named the argument it built as std::_Quote_in rather than a
// string. GCC prefers this non-template over the std template and compiles the
// same code happily, so the two toolchains genuinely disagree here. The name
// is simply not worth arguing about.
std::string dq(const std::string& p) { return "\"" + p + "\""; }

// Runs a command and returns its stdout, trimmed. Empty on failure -- for a
// version probe "produced nothing" and "did not run" are the same answer.
std::string capture(const std::string& cmd) {
    const std::string full = shell(cmd + " 2> " NULLDEV);
    std::FILE* f = POPEN(full.c_str(), "r");
    if (!f) return "";
    std::string out;
    std::array<char, 512> buf{};
    while (std::fgets(buf.data(), int(buf.size()), f)) out += buf.data();
    PCLOSE(f);
    while (!out.empty() && (out.back() == '\n' || out.back() == '\r' || out.back() == ' '))
        out.pop_back();
    return out;
}

bool quiet(const std::string& cmd) {
    return std::system(shell(cmd + " > " NULLDEV " 2>&1").c_str()) == 0;
}

// ONE LIST FOR EVERY PLATFORM, not an #ifdef per platform. Each entry is only
// a way IN: it is tried, and what gets recorded is the sys.executable it
// reports, so a name that does not exist here simply produces nothing and is
// skipped. `py` is a Windows launcher and `python3.11` is a POSIX convention,
// and each is harmlessly absent on the other.
//
// THE ORDER MATTERS ON WINDOWS: `python` comes before `python3` because
// `python3` there is usually a Microsoft Store stub, and invoking it pops the
// Store open. The probe would reject it anyway -- it prints nothing -- but
// only after putting a shop window on the user's screen.
std::vector<std::pair<std::string, std::string>> seeds() {
    return {{"py -3.11",   "py -3.11"},
            {"py -3",      "py launcher"},
            {"python3.11", "PATH"},
            {"python",     "PATH"},
            {"python3",    "PATH"}};
}

// Interpreters that are installed but not on PATH and not the launcher
// default. `py -0p` lists them with their paths, which is the only way to see
// a python nothing points at -- and on a machine with several, that is usually
// where the one you need is hiding.
//
// NOT INSIDE AN #ifdef, though only Windows has the launcher. The body is
// plain string and filesystem code, and elsewhere `py -0p` is simply not a
// command, so capture() returns nothing and the loop does not run: the same
// behaviour, without a block that the machine this is written on never
// compiles. That distinction is not academic -- the previous version put this
// body behind #ifdef _WIN32, so a type error on the push_back below survived
// every local build and clean test run, and was found only by the Windows
// runner. Code that is never compiled is never checked.
void addRegistered(std::vector<std::pair<std::string, std::string>>& out) {
    std::istringstream ls(capture("py -0p"));
    std::string line;
    while (std::getline(ls, line)) {
        const size_t c = line.find(":\\");
        if (c == std::string::npos || c < 1) continue;
        std::string path = line.substr(c - 1);
        while (!path.empty() && (path.back() == '\r' || path.back() == ' ')) path.pop_back();
        std::error_code ec;
        if (fs::exists(path, ec)) out.push_back({dq(path), "py -0p"});
    }
}

// Is this a source checkout or an unzipped release? The two need OPPOSITE
// advice and the old message only knew how to give one of them: it told
// everybody to run `cmake --build build --target voxelenv`, which in a
// download folder means installing a C++ toolchain to fix a missing file that
// should have been in the zip. Telling someone to build a compiler stack when
// the real answer is "re-download" is worse than saying nothing.
bool inSourceTree(const std::string& exeDir) {
    std::error_code ec;
    const std::string up[] = {exeDir + "/CMakeLists.txt",
                              exeDir + "/../CMakeLists.txt",
                              exeDir + "/../../CMakeLists.txt"};
    for (const std::string& c : up) if (fs::exists(c, ec)) return true;
    return false;
}

}  // namespace

std::string moduleAbi(const std::string& exeDir) {
    std::error_code ec;
    if (!fs::is_directory(exeDir, ec)) return "";
    for (const auto& e : fs::directory_iterator(exeDir, ec)) {
        if (ec) break;
        const std::string n = e.path().filename().string();
        if (n.rfind("voxelenv", 0) != 0) continue;
        const std::string x = e.path().extension().string();
        if (x != ".pyd" && x != ".so") continue;
        // pybind11 tags the file with the ABI it was built for:
        //   voxelenv.cp311-win_amd64.pyd   voxelenv.cpython-311-x86_64-linux.so
        const size_t p = n.find("cp");
        if (p == std::string::npos) continue;
        std::string d;
        for (size_t i = p; i < n.size() && d.size() < 3; ++i)
            if (std::isdigit(static_cast<unsigned char>(n[i]))) d += n[i];
        if (d.size() >= 3) return d.substr(0, 1) + "." + d.substr(1);
    }
    return "";
}

std::vector<Py> discover(const std::string& exeDir) {
    auto cands = seeds();
    addRegistered(cands);

    std::vector<Py> out;
    for (const auto& c : cands) {
        const std::string real =
            capture(c.first + " -c \"import sys;print(sys.executable)\"");
        if (real.empty()) continue;
        std::error_code ec;
        if (!fs::exists(real, ec)) continue;

        // DEDUPE BY REAL PATH, not by the name used to reach it. python,
        // python3 and python3.11 are routinely three symlinks to one binary,
        // and listing that install three times is the opposite of useful on a
        // machine where the question is which of several pythons is in play.
        // The names it answers to are kept, since that is what you type.
        const std::string canon = fs::canonical(real, ec).string();
        const std::string key = ec ? real : canon;
        auto seen = std::find_if(out.begin(), out.end(),
                                 [&](const Py& q) { return q.exe == key; });
        if (seen != out.end()) {
            if (seen->origin.find(c.second) == std::string::npos)
                seen->origin += ", " + c.second;
            continue;
        }

        Py p;
        p.exe = key;
        p.origin = c.second;
        p.runs = true;
        std::istringstream(capture(dq(real) +
            " -c \"import sys;print('%d %d'%sys.version_info[:2])\"")) >> p.major >> p.minor;
        p.voxelenv = quiet(dq(real) + " -c \"import sys;sys.path.insert(0,r'" +
                           exeDir + "');import voxelenv\"");
        p.rl = quiet(dq(real) +
                     " -c \"import gymnasium,stable_baselines3,sb3_contrib\"");
        out.push_back(p);
    }

    // voxelenv first, then the RL stack, then newest. An interpreter that
    // cannot load the extension cannot train, however new it is.
    std::stable_sort(out.begin(), out.end(), [](const Py& a, const Py& b) {
        if (a.voxelenv != b.voxelenv) return a.voxelenv;
        if (a.rl != b.rl) return a.rl;
        return std::make_pair(a.major, a.minor) > std::make_pair(b.major, b.minor);
    });
    return out;
}

const Py* best(const std::vector<Py>& v) {
    for (const Py& p : v) if (p.voxelenv) return &p;
    return nullptr;
}

int install(const Py& p, const std::string& reqs) {
    std::string cmd = dq(p.exe) + " -m pip install ";
    cmd += reqs.empty()
         ? "gymnasium stable-baselines3 sb3-contrib numpy tensorboard"
         : "-r " + dq(reqs);
    // THE FULL PATH IS PRINTED, not "python". On a machine with several that
    // difference is the whole point: the line is unambiguous about which
    // interpreter is being changed, and can be pasted to repeat it by hand.
    std::printf("[kestrel] %s\n", cmd.c_str());
    std::fflush(stdout);
    return std::system(shell(cmd).c_str());
}

void report(const std::vector<Py>& v, const std::string& exeDir) {
    const std::string abi = moduleAbi(exeDir);
    std::printf("python interpreters found (best first)\n");
    if (abi.empty())
        std::printf("  no voxelenv module beside the exe, so nothing here can train yet\n");
    else
        std::printf("  the voxelenv module here was built for CPython %s -- ONLY that\n"
                    "  minor version can load it\n", abi.c_str());
    std::printf("\n%-9s %-9s %-9s %-22s %s\n",
                "version", "voxelenv", "RL stack", "found via", "path");
    if (v.empty()) std::printf("  (none found)\n");
    for (const Py& p : v)
        std::printf("%d.%-7d %-9s %-9s %-22s %s\n", p.major, p.minor,
                    p.voxelenv ? "yes" : "NO", p.rl ? "yes" : "no",
                    p.origin.c_str(), p.exe.c_str());

    const Py* b = best(v);
    std::printf("\n");
    if (b) {
        std::printf("kestrel train will use:\n  %s\n", b->exe.c_str());
        if (!b->rl)
            std::printf("\nIt does not have the RL stack yet. `kestrel train --install`\n"
                        "installs into THAT interpreter and no other.\n");
    } else if (!abi.empty()) {
        std::printf("NONE of these can load voxelenv. That is a version mismatch, not a\n"
                    "missing file: the module needs CPython %s. Install %s, or rebuild the\n"
                    "module against a python you already have.\n", abi.c_str(), abi.c_str());
    } else if (inSourceTree(exeDir)) {
        std::printf("There is no voxelenv module beside the exe. Training needs it -- it\n"
                    "is the C++ environment the trainer steps. This is a source tree, so\n"
                    "build it:\n"
                    "  python -m pip install pybind11\n"
                    "  cmake -B build -Dpybind11_DIR=(python -m pybind11 --cmakedir)\n"
                    "  cmake --build build --config Release --target voxelenv\n");
    } else {
        // A download folder. There is no build system here and no compiler is
        // implied by having unzipped something, so build instructions would
        // send you after a toolchain to fix a packaging bug.
        std::printf("There is no voxelenv module in this download, and no source tree here\n"
                    "to build one from. DO NOT install cmake -- this is a packaging\n"
                    "problem, not something to fix locally. Get a build whose zip\n"
                    "contains voxelenv and python/: the Actions tab of the repository,\n"
                    "latest run, Artifacts.\n\n"
                    "The other three commands do not need any of this and work from this\n"
                    "folder as it stands.\n");
    }
    std::fflush(stdout);
}

}  // namespace kpy
