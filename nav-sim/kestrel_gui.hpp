#pragma once
// The point-and-click front end for kestrel.
//
// It is a SEPARATE translation unit that knows nothing about what the four
// commands do -- it collects settings, shows the command line it built, and
// calls back. That is deliberate: the GUI cannot drift away from the CLI,
// because it drives exactly the same entry points with exactly the arguments
// it puts on screen. Anything you can click, you can also type.
#include <functional>
#include <string>
#include <vector>

namespace kgui {

// Bound by kestrel.cpp to its own cmdTrack / cmdBench / voxelLiveMain / cmdTrain.
struct Actions {
    std::function<int(std::vector<std::string>)> track;
    std::function<int(std::vector<std::string>)> bench;
    std::function<int(std::vector<std::string>)> sim;
    std::function<int(std::vector<std::string>)> train;
};

// Runs the window until the user quits. Returns a process exit code, or -1 if
// this build has no highgui -- the caller then falls back to the text menu
// rather than exiting, because a headless build is a supported build here.
int run(const Actions& act, const std::string& exeDir);

// Renders every panel to <prefix>_<mode>.png with NO display attached, and
// returns how many were written (0 if this build has no highgui). The window
// is the one artefact here that cannot be inspected over ssh or in CI; this is
// how it gets looked at anyway. `kestrel gui --shot out` calls it.
int shot(const std::string& exeDir, const std::string& prefix);

// Lays out every panel and asserts the things that go wrong in a hand-placed
// layout: buttons that overlap each other, buttons off the canvas, two buttons
// sharing an id, and a label squeezed until it says nothing. Prints each
// violation and returns how many it found. `kestrel gui --check` calls it, and
// it needs no display, so it runs in ctest.
int check();

}  // namespace kgui
