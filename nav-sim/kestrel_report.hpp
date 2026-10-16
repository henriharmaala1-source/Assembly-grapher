// HOW THE POLICY FAILS, as pictures you can look at.
//
// Everything else in this tree reports the policy as a table of numbers, and a
// table cannot answer the question that actually matters. Six episodes scored
// on a real checkpoint came back as five rows all labelled "ran out of steps":
// one had never left the spawn, one had flown the wrong way, one was still
// closing when the budget ran out, and two were identical to three significant
// figures in two differently-generated worlds -- which means the policy was not
// reading the world at all. Five different failures, five different fixes, one
// label.
//
// So: classify the failure, and draw it.
//
// DRAWING LIVES IN C++, on purpose. The alternative was matplotlib, and
// matplotlib is not in python/requirements.txt while kestrel's own preflight
// only probes gymnasium, stable_baselines3 and sb3_contrib -- so a matplotlib
// report would pass the check the whole program is built around and then die on
// an ImportError, which is precisely the failure kestrel.cpp works hardest to
// prevent. OpenCV is already a hard dependency here and already draws
// everything else. It also means `report --plot`, `--shot` and `--check` need
// no python at all, so the layout is testable in CI exactly like `gui --check`.
//
// THE CSV IS THE ARTEFACT. Flying the episodes is the expensive part; redrawing
// must never require re-flying, so python writes report.csv + trace.csv and the
// drawing reads only those.
#pragma once

#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace krep {

// The taxonomy. ARRIVED and the two collisions are facts; the rest are the
// five distinct things "out of steps" was hiding.
enum Outcome {
    ARRIVED = 0,
    COLLIDED_MAPPED,    // flew into a cell its own map had marked OCCUPIED.
                        // With the veto on this should be unreachable -- if it
                        // happens, the veto has a bug.
    COLLIDED_BLIND,     // flew into something nothing had measured. The honest
                        // failure; the fix is sensing range, not the policy.
    STALLED_HELD,       // never left the spawn, and chose to hold
    STALLED_STUCK,      // never left the spawn while commanding speed
    ORBITED,            // travelled far, went nowhere
    WRONG_WAY,          // never got closer than it started
    DRIFTED_OFF,        // got close, then ended well outside its own best
    CLOSING_BUDGET,     // still closing, and a longer episode would arrive
    CLOSING_TOO_SLOW,   // still closing, but never going to arrive
    NOUTCOMES
};
const char* outcomeName(int o);
const char* outcomeShort(int o);
cv::Scalar  outcomeColour(int o);

// One flown episode. Mirrors the columns of report.csv exactly.
struct Episode {
    std::string world;
    int   seed = 0, repeat = 0;
    long long checkpointSteps = 0;      // 0 when the run scored one policy
    int   steps = 0, maxSteps = 0;
    float travelM = 0, startDistM = 0, distM = 0, minDistM = 0;
    int   minDistStep = 0;
    float netDispM = 0;
    int   cellsVisited = 0;
    float minClearM = 0;
    int   stoppedSteps = 0, collisions = 0;
    bool  hitUnknown = false, reachedGoal = false;
    float rProgress = 0, rCoverage = 0, rTime = 0, rStop = 0, rClear = 0,
          rTerminal = 0;
    float goalTolM = 3.f, robotR = 0.6f;
    float goalX = 0.f, goalY = 0.f;     // world metres, so the map can show it
    // Filled from trace.csv. Empty is legal -- the bar and reward panels do not
    // need it, only the map and the curves do.
    std::vector<cv::Point2f> trail;
    std::vector<float>       dist;      // distance to goal, at tstep[k]
    // THE STEP NUMBER OF EACH SAMPLE. A 3000-step episode does not need 3000
    // trace rows and report.py strides them, so the index into these vectors is
    // not the step. Plotting k/maxSteps instead of tstep[k]/maxSteps squeezed
    // every curve into the first few percent of its panel.
    std::vector<int>         tstep;
};

// ONE classifier, so the table and the pictures can never disagree.
int classify(const Episode& e);
// True when the journey cannot be flown in the budget at the cruise speed a
// planner actually holds -- an annotation, not an outcome: it says the episode
// was unwinnable before it started.
bool impossible(const Episode& e);

bool readCsv(const std::string& dir, std::vector<Episode>& out, std::string& err);

// Each returns the canvas it drew, so a caller can write it or measure it.
cv::Mat drawOutcomes(const std::vector<Episode>& eps);
cv::Mat drawCurves(const std::vector<Episode>& eps);
cv::Mat drawReward(const std::vector<Episode>& eps);
cv::Mat drawProgress(const std::vector<Episode>& eps);
cv::Mat drawMap(const std::vector<Episode>& eps, const std::string& world,
                int seed);

// Write every panel this record set supports, as PREFIX_<name>.png.
int writeAll(const std::vector<Episode>& eps, const std::string& prefix);

// A synthetic record set covering all ten outcomes, for --shot and --check.
// Nothing is flown and no policy is loaded.
std::vector<Episode> synthetic();

int shot(const std::string& prefix);   // draw every panel from synthetic()
int check();                           // assert the layout; returns violations

}  // namespace krep
