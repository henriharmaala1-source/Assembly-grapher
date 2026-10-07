#include "kestrel_report.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <map>
#include <set>
#include <sstream>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace krep {
namespace {

// The GUI's canvas and palette, so a report sits beside a panel screenshot and
// looks like the same program.
const int W = 1060, H = 660;
const cv::Scalar BG(26, 26, 30), INK(235, 235, 235), DIM(150, 150, 150),
                 EDGE(70, 70, 78), GRID(52, 52, 58);

std::vector<cv::Rect>* g_boxes = nullptr;   // set during check()

void txt(cv::Mat& im, const std::string& s, int x, int y, double sc = 0.44,
         cv::Scalar c = INK, int th = 1) {
    if (g_boxes && !s.empty()) {
        int base = 0;
        const cv::Size z = cv::getTextSize(s, cv::FONT_HERSHEY_SIMPLEX, sc, th,
                                           &base);
        g_boxes->push_back(cv::Rect(x, y - z.height, z.width, z.height));
    }
    cv::putText(im, s, {x, y}, cv::FONT_HERSHEY_SIMPLEX, sc, c, th,
                cv::LINE_AA);
}

int textW(const std::string& s, double sc, int th = 1) {
    int base = 0;
    return cv::getTextSize(s, cv::FONT_HERSHEY_SIMPLEX, sc, th, &base).width;
}

cv::Mat canvas(const std::string& title, const std::string& sub) {
    cv::Mat im(H, W, CV_8UC3, BG);
    txt(im, title, 28, 44, 0.66, INK, 1);
    if (!sub.empty()) txt(im, sub, 28, 68, 0.42, DIM);
    return im;
}

std::string f1(float v) {
    char b[32]; std::snprintf(b, sizeof b, "%.1f", v); return b;
}
std::string pct(float v) {
    char b[32]; std::snprintf(b, sizeof b, "%.0f%%", 100.f * v); return b;
}

// Worlds in the order they were flown, not alphabetical: a report should read
// in the order the run produced.
std::vector<std::string> worldsOf(const std::vector<Episode>& eps) {
    std::vector<std::string> out;
    for (const Episode& e : eps)
        if (std::find(out.begin(), out.end(), e.world) == out.end())
            out.push_back(e.world);
    return out;
}

// Measured with the best classical planner over 3000-step episodes: path length
// saturates at 260-278 m in every world here, so this is a property of the
// vehicle rather than of a map. Kept in step with voxel_gym.CRUISE_M_PER_STEP.
const float CRUISE_M_PER_STEP = 0.09f;

}  // namespace

// ---------------------------------------------------------------- taxonomy

const char* outcomeName(int o) {
    switch (o) {
        case ARRIVED:          return "arrived";
        case COLLIDED_MAPPED:  return "collided (it had mapped that)";
        case COLLIDED_BLIND:   return "collided (nothing had seen it)";
        case STALLED_HELD:     return "never left the spawn (held)";
        case STALLED_STUCK:    return "never left the spawn (stuck)";
        case ORBITED:          return "orbited - travelled, went nowhere";
        case WRONG_WAY:        return "wrong way - never got closer";
        case DRIFTED_OFF:      return "got close, then drifted off";
        case CLOSING_BUDGET:   return "still closing - needed more steps";
        case CLOSING_TOO_SLOW: return "still closing - too slow to arrive";
    }
    return "?";
}

const char* outcomeShort(int o) {
    switch (o) {
        case ARRIVED:          return "arrived";
        case COLLIDED_MAPPED:  return "hit mapped";
        case COLLIDED_BLIND:   return "hit blind";
        case STALLED_HELD:     return "held";
        case STALLED_STUCK:    return "stuck";
        case ORBITED:          return "orbited";
        case WRONG_WAY:        return "wrong way";
        case DRIFTED_OFF:      return "drifted";
        case CLOSING_BUDGET:   return "needed more";
        case CLOSING_TOO_SLOW: return "too slow";
    }
    return "?";
}

// BGR. Arrived is the only green; the two collisions are the only reds, and
// they differ in brightness because they need opposite fixes. The four
// "went nowhere" classes are blues, the two "was closing" ones are warm --
// so a bar reads as a shape before any label is parsed.
cv::Scalar outcomeColour(int o) {
    switch (o) {
        case ARRIVED:          return cv::Scalar( 90, 180,  90);
        case COLLIDED_MAPPED:  return cv::Scalar( 60,  60, 235);
        case COLLIDED_BLIND:   return cv::Scalar( 90, 110, 175);
        case STALLED_HELD:     return cv::Scalar(150, 110,  80);
        case STALLED_STUCK:    return cv::Scalar(190, 150, 110);
        case ORBITED:          return cv::Scalar(175, 130, 190);
        case WRONG_WAY:        return cv::Scalar(120,  90, 140);
        case DRIFTED_OFF:      return cv::Scalar(190, 175,  90);
        case CLOSING_BUDGET:   return cv::Scalar( 70, 175, 215);
        case CLOSING_TOO_SLOW: return cv::Scalar( 60, 130, 165);
    }
    return DIM;
}

int classify(const Episode& e) {
    if (e.reachedGoal) return ARRIVED;
    if (e.collisions)  return e.hitUnknown ? COLLIDED_BLIND : COLLIDED_MAPPED;

    const float J = std::max(1e-3f, e.startDistM);
    const int   S = std::max(1, e.steps);

    // Never left: travel under two body radii. Held vs stuck is the difference
    // between a policy that chose to stop and one that asked for speed and did
    // not get it -- one is a reward problem, the other is a control problem.
    if (e.travelM < 2.f * e.robotR)
        return e.stoppedSteps > S / 2 ? STALLED_HELD : STALLED_STUCK;

    // Orbiting: a lot of path, very little displacement. distToGoal alone
    // cannot see this -- it only measures motion along the goal axis, so a
    // circle around the spawn and a straight line sideways look identical.
    if (e.travelM > 10.f && e.travelM > 3.f * e.netDispM) return ORBITED;

    // Never got closer than it started, having actually flown somewhere.
    if (e.minDistM > 0.95f * J && e.travelM > 10.f) return WRONG_WAY;

    // Got close, then ended well outside its own best. "Never got there" and
    // "got there and left" are different failures: one is navigation, the
    // other is termination.
    if (e.minDistM < 0.5f * J &&
        e.distM > e.minDistM + std::max(e.goalTolM, 0.15f * J) &&
        e.minDistStep < (4 * S) / 5)
        return DRIFTED_OFF;

    // Still closing at the buzzer. Split by whether MORE STEPS would have been
    // enough: one wants a longer episode, the other wants a better policy, and
    // they look identical in every column this project had.
    //
    // The first version of this compared the rate achieved against the rate
    // needed to cover the whole journey in the budget -- which for a truncated
    // episode (steps == maxSteps, always) reduces to minDist <= 0, so
    // CLOSING_BUDGET could never occur. `report --check` caught it by asserting
    // every class is reachable, before any of this had been run on real data.
    //
    // What the question actually is: at the rate it DID close, how many more
    // steps would the remainder take? Within another full budget means the
    // episode was too short; beyond that means the policy is too slow.
    const float closedPerStep = (J - e.minDistM) / float(S);
    if (closedPerStep <= 0.f) return CLOSING_TOO_SLOW;
    const float extraSteps = e.minDistM / closedPerStep;
    return extraSteps <= float(std::max(1, e.maxSteps)) ? CLOSING_BUDGET
                                                        : CLOSING_TOO_SLOW;
}

bool impossible(const Episode& e) {
    return e.maxSteps > 0 &&
           e.startDistM / CRUISE_M_PER_STEP > float(e.maxSteps);
}

// ---------------------------------------------------------------------- csv
namespace {

std::vector<std::string> split(const std::string& line, char sep) {
    std::vector<std::string> out;
    std::string cur;
    std::istringstream is(line);
    while (std::getline(is, cur, sep)) out.push_back(cur);
    return out;
}

float fnum(const std::vector<std::string>& v, const std::map<std::string,int>& h,
           const char* k, float dflt = 0.f) {
    auto it = h.find(k);
    if (it == h.end() || it->second >= int(v.size())) return dflt;
    try { return std::stof(v[it->second]); } catch (...) { return dflt; }
}
std::string fstr(const std::vector<std::string>& v,
                 const std::map<std::string,int>& h, const char* k) {
    auto it = h.find(k);
    if (it == h.end() || it->second >= int(v.size())) return "";
    return v[it->second];
}

}  // namespace

bool readCsv(const std::string& dir, std::vector<Episode>& out,
             std::string& err) {
    const std::string path = dir + "/report.csv";
    std::ifstream fh(path);
    if (!fh) { err = "cannot open " + path; return false; }
    std::string line;
    if (!std::getline(fh, line)) { err = path + " is empty"; return false; }
    std::map<std::string, int> h;
    {
        const std::vector<std::string> cols = split(line, ',');
        for (size_t i = 0; i < cols.size(); ++i) h[cols[i]] = int(i);
    }
    std::map<int, size_t> byId;
    while (std::getline(fh, line)) {
        if (line.empty()) continue;
        const std::vector<std::string> v = split(line, ',');
        Episode e;
        e.world = fstr(v, h, "world");
        e.seed = int(fnum(v, h, "seed"));
        e.repeat = int(fnum(v, h, "repeat"));
        e.checkpointSteps = (long long)fnum(v, h, "checkpoint_steps");
        e.steps = int(fnum(v, h, "steps"));
        e.maxSteps = int(fnum(v, h, "max_steps"));
        e.travelM = fnum(v, h, "travel_m");
        e.startDistM = fnum(v, h, "start_dist_m");
        e.distM = fnum(v, h, "dist_to_goal_m");
        e.minDistM = fnum(v, h, "min_dist_to_goal_m");
        e.minDistStep = int(fnum(v, h, "min_dist_step"));
        e.netDispM = fnum(v, h, "net_disp_m");
        e.cellsVisited = int(fnum(v, h, "cells_visited"));
        e.minClearM = fnum(v, h, "min_clear_m");
        e.stoppedSteps = int(fnum(v, h, "stopped_steps"));
        e.collisions = int(fnum(v, h, "collisions"));
        e.hitUnknown = fnum(v, h, "hit_unknown") != 0.f;
        e.reachedGoal = fnum(v, h, "reached_goal") != 0.f;
        e.rProgress = fnum(v, h, "r_progress");
        e.rCoverage = fnum(v, h, "r_coverage");
        e.rTime = fnum(v, h, "r_time");
        e.rStop = fnum(v, h, "r_stop");
        e.rClear = fnum(v, h, "r_clear");
        e.rTerminal = fnum(v, h, "r_terminal");
        e.goalTolM = fnum(v, h, "goal_tol_m", 3.f);
        e.robotR = fnum(v, h, "robot_r", 0.6f);
        e.goalX = fnum(v, h, "goal_x");
        e.goalY = fnum(v, h, "goal_y");
        byId[int(fnum(v, h, "id", -1))] = out.size();
        out.push_back(e);
    }
    if (out.empty()) { err = path + " has a header but no rows"; return false; }

    // Traces are optional: the bar and reward panels do not need them, so a
    // report without them is degraded rather than broken.
    std::ifstream tf(dir + "/trace.csv");
    if (tf && std::getline(tf, line)) {
        std::map<std::string, int> th;
        const std::vector<std::string> cols = split(line, ',');
        for (size_t i = 0; i < cols.size(); ++i) th[cols[i]] = int(i);
        while (std::getline(tf, line)) {
            if (line.empty()) continue;
            const std::vector<std::string> v = split(line, ',');
            auto it = byId.find(int(fnum(v, th, "id", -1)));
            if (it == byId.end()) continue;
            Episode& e = out[it->second];
            e.trail.push_back({fnum(v, th, "x"), fnum(v, th, "y")});
            e.dist.push_back(fnum(v, th, "dist"));
            e.tstep.push_back(int(fnum(v, th, "step",
                                       float(e.tstep.size()))));
        }
    }
    return true;
}

// ------------------------------------------------------------------ panels
namespace {

// A legend column shared by the panels that colour by outcome. Drawn only for
// classes actually present, because ten rows of which three occurred is a
// legend that hides its own answer.
void legend(cv::Mat& im, int x, int y, const std::set<int>& present) {
    txt(im, "outcome", x, y - 10, 0.44, DIM);
    int row = 0;
    for (int o = 0; o < NOUTCOMES; ++o) {
        if (!present.count(o)) continue;
        const int yy = y + row * 20;
        cv::rectangle(im, cv::Rect(x, yy, 12, 12), outcomeColour(o), cv::FILLED);
        txt(im, outcomeShort(o), x + 20, yy + 11, 0.40, DIM);
        ++row;
    }
}

std::set<int> presentIn(const std::vector<Episode>& eps) {
    std::set<int> p;
    for (const Episode& e : eps) p.insert(classify(e));
    return p;
}

}  // namespace

// P1 -- which failure is THIS world's failure.
cv::Mat drawOutcomes(const std::vector<Episode>& eps) {
    cv::Mat im = canvas("how it fails, per world",
                        "one bar per world over every episode flown. The "
                        "question a table of means cannot answer.");
    const std::vector<std::string> ws = worldsOf(eps);
    const std::set<int> present = presentIn(eps);
    const int x0 = 28, barW = 720, top = 110, pitch = 78;

    for (size_t i = 0; i < ws.size() && i < 6; ++i) {
        const int y = top + int(i) * pitch;
        int n = 0, counts[NOUTCOMES] = {0};
        float travel = 0;
        int goals = 0, hits = 0;
        for (const Episode& e : eps) {
            if (e.world != ws[i]) continue;
            ++n; ++counts[classify(e)];
            travel += e.travelM;
            goals += e.reachedGoal ? 1 : 0;
            hits += e.collisions ? 1 : 0;
        }
        if (!n) continue;
        txt(im, ws[i], x0, y - 8, 0.50, INK);
        int cx = x0;
        for (int o = 0; o < NOUTCOMES; ++o) {
            if (!counts[o]) continue;
            const int w = std::max(2, counts[o] * barW / n);
            cv::rectangle(im, cv::Rect(cx, y, w, 30), outcomeColour(o),
                          cv::FILLED);
            const std::string lab = std::to_string(counts[o]);
            if (w > textW(lab, 0.42) + 10)
                txt(im, lab, cx + (w - textW(lab, 0.42)) / 2, y + 21, 0.42,
                    cv::Scalar(20, 20, 20));
            cx += w;
        }
        cv::rectangle(im, cv::Rect(x0, y, barW, 30), EDGE, 1);
        // Reconciles with evaluate's table on purpose: a new picture that
        // disagrees with the established numbers is not evidence, it is noise.
        txt(im, std::to_string(n) + " ep   goal " + pct(float(goals) / n) +
                "   crash " + pct(float(hits) / n) +
                "   travel " + f1(travel / n) + " m",
            x0, y + 50, 0.40, DIM);
    }
    legend(im, 800, 130, present);

    int imp = 0;
    for (const Episode& e : eps) imp += impossible(e) ? 1 : 0;
    if (imp)
        txt(im, std::to_string(imp) + " of " + std::to_string(eps.size()) +
                " episodes had a goal further away than the step budget can "
                "fly. Those are not policy failures.",
            x0, H - 40, 0.42, cv::Scalar(70, 175, 215));
    return im;
}

// P3 -- the SHAPE of the failure.
cv::Mat drawCurves(const std::vector<Episode>& eps) {
    cv::Mat im = canvas("distance to the goal, over the episode",
                        "flat at 1.0 is stalled - sawtooth is orbiting - a V "
                        "got close and left - a line that stops short was "
                        "still closing.");
    const std::vector<std::string> ws = worldsOf(eps);
    const int cols = 3, cw = 320, ch = 200, x0 = 28, y0 = 110;

    for (size_t i = 0; i < ws.size() && i < 6; ++i) {
        const int cx = x0 + int(i % cols) * (cw + 20);
        const int cy = y0 + int(i / cols) * (ch + 56);
        const cv::Rect box(cx, cy, cw, ch);
        cv::rectangle(im, box, GRID, cv::FILLED);
        cv::rectangle(im, box, EDGE, 1);
        txt(im, ws[i], cx, cy - 8, 0.46, INK);

        // 1.0 is "as far away as it started". Above it means it went backwards.
        const int yOne = cy + ch - int(ch / 1.2f);
        cv::line(im, {cx, yOne}, {cx + cw, yOne}, cv::Scalar(90, 90, 96), 1);
        txt(im, "start", cx + cw - 38, yOne - 4, 0.34, DIM);

        for (const Episode& e : eps) {
            if (e.world != ws[i] || e.dist.size() < 2) continue;
            const float J = std::max(1e-3f, e.startDistM);
            const int N = int(e.dist.size());
            std::vector<cv::Point> pts;
            pts.reserve(N);
            int closestIdx = 0, closestErr = 1 << 30;
            for (int k = 0; k < N; ++k) {
                const int st = k < int(e.tstep.size()) ? e.tstep[size_t(k)] : k;
                const float fx = float(st) / float(std::max(1, e.maxSteps));
                const float fy = std::min(1.2f, e.dist[size_t(k)] / J);
                pts.push_back({cx + int(fx * cw),
                               cy + ch - int(fy / 1.2f * ch)});
                const int err = std::abs(st - e.minDistStep);
                if (err < closestErr) { closestErr = err; closestIdx = k; }
            }
            cv::polylines(im, pts, false, outcomeColour(classify(e)), 1,
                          cv::LINE_AA);
            // The nearest SAMPLE to the closest approach -- the exact step may
            // not have been kept.
            if (e.minDistStep > 0)
                cv::circle(im, pts[size_t(closestIdx)], 3,
                           cv::Scalar(235, 235, 235), 1, cv::LINE_AA);
            if (e.collisions && !pts.empty()) {
                const cv::Point p = pts.back();
                cv::line(im, {p.x - 4, p.y - 4}, {p.x + 4, p.y + 4},
                         cv::Scalar(60, 60, 235), 1, cv::LINE_AA);
                cv::line(im, {p.x - 4, p.y + 4}, {p.x + 4, p.y - 4},
                         cv::Scalar(60, 60, 235), 1, cv::LINE_AA);
            }
        }
        txt(im, "0", cx - 10, cy + ch + 14, 0.34, DIM);
        txt(im, "end of budget", cx + cw - textW("end of budget", 0.34),
            cy + ch + 14, 0.34, DIM);
    }
    txt(im, "o marks the closest approach and when.   x marks a collision.",
        x0, H - 26, 0.40, DIM);
    return im;
}

// P4 -- which reward term actually drove the episode.
cv::Mat drawReward(const std::vector<Episode>& eps) {
    cv::Mat im = canvas("where the reward went",
                        "what an ARRIVED episode was paid against a FAILED "
                        "one. A return says the policy improved; not which "
                        "term it improved.");
    const std::vector<std::string> ws = worldsOf(eps);
    const char* names[6] = {"progress", "coverage", "time", "stop", "clear",
                            "terminal"};
    const int cols = 3, cw = 320, chh = 190, x0 = 28, y0 = 116;

    // ONE SCALE ACROSS EVERY WORLD. Per-cell autoscaling would make two bars of
    // equal length mean different numbers, which is the standard way a panel
    // like this misleads.
    float peak = 1.f;
    for (const Episode& e : eps) {
        const float v[6] = {e.rProgress, e.rCoverage, e.rTime, e.rStop,
                            e.rClear, e.rTerminal};
        for (int k = 0; k < 6; ++k) peak = std::max(peak, std::fabs(v[k]));
    }

    for (size_t i = 0; i < ws.size() && i < 6; ++i) {
        const int cx = x0 + int(i % cols) * (cw + 20);
        const int cy = y0 + int(i / cols) * (chh + 60);
        cv::rectangle(im, cv::Rect(cx, cy, cw, chh), GRID, cv::FILLED);
        cv::rectangle(im, cv::Rect(cx, cy, cw, chh), EDGE, 1);
        txt(im, ws[i], cx, cy - 8, 0.46, INK);

        const int lab = 66;                 // term-name column
        const int half = (cw - lab - 16) / 2;
        const int zero = cx + lab + half;   // bars diverge from here
        cv::line(im, {zero, cy + 4}, {zero, cy + chh - 4}, EDGE, 1);

        float sum[2][6] = {{0}}; int n[2] = {0, 0};
        for (const Episode& e : eps) {
            if (e.world != ws[i]) continue;
            const int g = e.reachedGoal ? 0 : 1;
            const float v[6] = {e.rProgress, e.rCoverage, e.rTime, e.rStop,
                                e.rClear, e.rTerminal};
            for (int k = 0; k < 6; ++k) sum[g][k] += v[k];
            ++n[g];
        }
        for (int k = 0; k < 6; ++k) {
            const int ry = cy + 16 + k * 28;
            txt(im, names[k], cx + 6, ry + 10, 0.36, DIM);
            for (int g = 0; g < 2; ++g) {
                if (!n[g]) continue;
                const float v = sum[g][k] / n[g];
                const int len = std::min(half, int(std::fabs(v) / peak * half));
                const int bx = v >= 0 ? zero : zero - len;
                // Arrived is the palette's green, failed its collision blue --
                // the same two colours those outcomes have everywhere else.
                const cv::Scalar col = g == 0 ? cv::Scalar(90, 180, 90)
                                              : cv::Scalar(150, 120, 210);
                cv::rectangle(im, cv::Rect(bx, ry + g * 9, std::max(1, len), 7),
                              col, cv::FILLED);
            }
        }
        if (i == 0) {
            cv::rectangle(im, cv::Rect(cx + lab + 4, cy + chh + 10, 10, 8),
                          cv::Scalar(90, 180, 90), cv::FILLED);
            txt(im, "arrived", cx + lab + 20, cy + chh + 18, 0.36, DIM);
            cv::rectangle(im, cv::Rect(cx + lab + 90, cy + chh + 10, 10, 8),
                          cv::Scalar(150, 120, 210), cv::FILLED);
            txt(im, "failed", cx + lab + 106, cy + chh + 18, 0.36, DIM);
        }
    }
    txt(im, "left of each line is a cost, right of it a payment. One scale "
            "across every world.", 28, H - 26, 0.40, DIM);
    return im;
}

// P5 -- is it getting better, on the scorecard rather than on reward.
cv::Mat drawProgress(const std::vector<Episode>& eps) {
    cv::Mat im = canvas("across checkpoints",
                        "goal rate and the fraction of the journey closed. The "
                        "second still moves while the first is pinned at zero.");
    std::set<long long> steps;
    for (const Episode& e : eps) steps.insert(e.checkpointSteps);
    const std::vector<long long> xs(steps.begin(), steps.end());
    const int x0 = 70, y0 = 110, pw = 900, ph = 190;

    for (int panel = 0; panel < 2; ++panel) {
        const int py = y0 + panel * (ph + 70);
        cv::rectangle(im, cv::Rect(x0, py, pw, ph), GRID, cv::FILLED);
        cv::rectangle(im, cv::Rect(x0, py, pw, ph), EDGE, 1);
        txt(im, panel ? "fraction of the journey closed" : "goal rate",
            x0, py - 8, 0.46, INK);
        txt(im, "1.0", x0 - 36, py + 10, 0.36, DIM);
        txt(im, "0", x0 - 16, py + ph, 0.36, DIM);
        if (xs.size() < 2) continue;

        const std::vector<std::string> ws = worldsOf(eps);
        for (size_t w = 0; w <= ws.size(); ++w) {
            const bool all = (w == ws.size());
            std::vector<cv::Point> pts;
            for (size_t k = 0; k < xs.size(); ++k) {
                float acc = 0; int n = 0;
                for (const Episode& e : eps) {
                    if (e.checkpointSteps != xs[k]) continue;
                    if (!all && e.world != ws[w]) continue;
                    acc += panel ? std::max(0.f, 1.f - e.minDistM /
                                            std::max(1e-3f, e.startDistM))
                                 : (e.reachedGoal ? 1.f : 0.f);
                    ++n;
                }
                if (!n) continue;
                const int px = x0 + int(float(k) / float(xs.size() - 1) * pw);
                pts.push_back({px, py + ph - int(acc / n * ph)});
            }
            if (pts.size() < 2) continue;
            const cv::Scalar col = all ? INK
                : cv::Scalar(80 + 30 * int(w % 6), 120 + 20 * int(w % 4),
                             200 - 25 * int(w % 5));
            cv::polylines(im, pts, false, col, all ? 2 : 1, cv::LINE_AA);
            // A LEGEND, NOT LABELS AT THE LINE ENDS. Curves that converge --
            // which is exactly what a set of worlds does once they are all
            // solved, the case worth looking at -- put every label in the same
            // few pixels on top of each other.
            if (panel == 0) {
                const int ly = py + 14 + int(w) * 18;
                cv::line(im, {x0 + pw - 90, ly}, {x0 + pw - 70, ly}, col,
                         all ? 2 : 1);
                txt(im, all ? "ALL" : ws[w], x0 + pw - 64, ly + 4, 0.36, col);
            }
        }
    }
    if (!xs.empty()) {
        txt(im, std::to_string(xs.front()) + " steps", x0, H - 26, 0.38, DIM);
        const std::string r = std::to_string(xs.back()) + " steps";
        txt(im, r, x0 + pw - textW(r, 0.38), H - 26, 0.38, DIM);
    }
    return im;
}

// P2 -- one choke point, or everywhere.
cv::Mat drawMap(const std::vector<Episode>& eps, const std::string& world,
                int seed) {
    cv::Mat im = canvas("where it goes wrong: " + world + " seed " +
                            std::to_string(seed),
                        "every episode flown in this exact world, coloured by "
                        "outcome. Triangle is the spawn, cross is the goal.");
    const cv::Rect box(28, 100, 700, 500);
    cv::rectangle(im, box, GRID, cv::FILLED);
    cv::rectangle(im, box, EDGE, 1);

    std::vector<const Episode*> mine;
    std::set<int> present;
    for (const Episode& e : eps)
        if (e.world == world && e.seed == seed && e.trail.size() > 1) {
            mine.push_back(&e);
            present.insert(classify(e));
        }
    if (mine.empty()) {
        txt(im, "no trace for this world and seed", box.x + 20, box.y + 40,
            0.46, DIM);
        return im;
    }

    // THE GOAL IS IN THE BOUNDS, not just the trails. A map framed on where the
    // aircraft went cannot show how far it was from where it was going, which
    // is the one thing the picture exists to say.
    float lo_x = 1e9f, hi_x = -1e9f, lo_y = 1e9f, hi_y = -1e9f;
    for (const Episode* e : mine) {
        for (const cv::Point2f& p : e->trail) {
            lo_x = std::min(lo_x, p.x); hi_x = std::max(hi_x, p.x);
            lo_y = std::min(lo_y, p.y); hi_y = std::max(hi_y, p.y);
        }
        lo_x = std::min(lo_x, e->goalX); hi_x = std::max(hi_x, e->goalX);
        lo_y = std::min(lo_y, e->goalY); hi_y = std::max(hi_y, e->goalY);
    }
    const float pad = 0.08f * std::max(hi_x - lo_x, hi_y - lo_y) + 2.f;
    lo_x -= pad; hi_x += pad; lo_y -= pad; hi_y += pad;
    const float sc = std::min(box.width / std::max(1e-3f, hi_x - lo_x),
                              box.height / std::max(1e-3f, hi_y - lo_y));
    auto to = [&](const cv::Point2f& p) {
        return cv::Point(box.x + int((p.x - lo_x) * sc),
                         // north up, so y is flipped
                         box.y + box.height - int((p.y - lo_y) * sc));
    };

    for (const Episode* e : mine) {
        std::vector<cv::Point> pts;
        pts.reserve(e->trail.size());
        for (const cv::Point2f& p : e->trail) pts.push_back(to(p));
        cv::polylines(im, pts, false, outcomeColour(classify(*e)), 1,
                      cv::LINE_AA);
        // spawn
        const cv::Point s = pts.front();
        const std::vector<cv::Point> tri{{s.x, s.y - 5}, {s.x - 5, s.y + 4},
                                         {s.x + 5, s.y + 4}};
        cv::polylines(im, tri, true, cv::Scalar(200, 200, 200), 1, cv::LINE_AA);
        if (e->minDistStep > 0) {
            size_t best = 0; int err = 1 << 30;
            for (size_t k = 0; k < pts.size(); ++k) {
                const int st = k < e->tstep.size() ? e->tstep[k] : int(k);
                if (std::abs(st - e->minDistStep) < err) {
                    err = std::abs(st - e->minDistStep); best = k;
                }
            }
            cv::circle(im, pts[best], 4, cv::Scalar(235, 235, 235), 1,
                       cv::LINE_AA);
        }
        if (e->collisions) {
            const cv::Point p = pts.back();
            cv::circle(im, p, 5, cv::Scalar(60, 60, 235), cv::FILLED);
            cv::line(im, {p.x - 7, p.y}, {p.x + 7, p.y},
                     cv::Scalar(255, 255, 255), 1);
            cv::line(im, {p.x, p.y - 7}, {p.x, p.y + 7},
                     cv::Scalar(255, 255, 255), 1);
        }
    }
    // The goal, at true scale: the ring is goalTolM, so "close" is a size on
    // the page rather than a number somewhere else.
    {
        const cv::Point g = to({mine.front()->goalX, mine.front()->goalY});
        const int r = std::max(3, int(mine.front()->goalTolM * sc));
        cv::circle(im, g, r, cv::Scalar(120, 220, 120), 1, cv::LINE_AA);
        cv::line(im, {g.x - 9, g.y}, {g.x + 9, g.y}, cv::Scalar(120, 220, 120), 1);
        cv::line(im, {g.x, g.y - 9}, {g.x, g.y + 9}, cv::Scalar(120, 220, 120), 1);
    }
    legend(im, 770, 130, present);
    txt(im, std::to_string(mine.size()) + " episodes   journey " +
            f1(mine.front()->startDistM) + " m", 770, 400, 0.42, DIM);
    txt(im, "o is each run's closest approach.", 770, 424, 0.38, DIM);
    txt(im, "A filled circle is where it hit.", 770, 446, 0.38, DIM);
    txt(im, "The ring at the cross is the goal", 770, 468, 0.38, DIM);
    txt(im, "tolerance, at true scale.", 770, 490, 0.38, DIM);
    return im;
}

int writeAll(const std::vector<Episode>& eps, const std::string& prefix) {
    int n = 0;
    auto put = [&](const std::string& name, const cv::Mat& im) {
        if (cv::imwrite(prefix + "_" + name + ".png", im)) ++n;
    };
    put("outcomes", drawOutcomes(eps));
    put("curves", drawCurves(eps));
    put("reward", drawReward(eps));
    std::set<long long> cps;
    for (const Episode& e : eps) cps.insert(e.checkpointSteps);
    if (cps.size() > 1) put("progress", drawProgress(eps));

    // One map per world, on the seed with the most traces -- the map is only
    // informative where several episodes flew the SAME world instance.
    for (const std::string& w : worldsOf(eps)) {
        std::map<int, int> bySeed;
        for (const Episode& e : eps)
            if (e.world == w && e.trail.size() > 1) ++bySeed[e.seed];
        if (bySeed.empty()) continue;
        int best = bySeed.begin()->first, bestN = 0;
        for (const auto& kv : bySeed)
            if (kv.second > bestN) { best = kv.first; bestN = kv.second; }
        put("map_" + w, drawMap(eps, w, best));
    }
    return n;
}

// ------------------------------------------------------------- synthetic
std::vector<Episode> synthetic() {
    std::vector<Episode> out;
    const char* ws[3] = {"forest", "maze", "city"};
    // Deterministic, no rng: a check that varies is a check nobody trusts.
    unsigned h = 12345u;
    auto rnd = [&h]() { h = h * 1664525u + 1013904223u;
                        return float(h >> 8) * (1.f / 16777216.f); };
    for (int w = 0; w < 3; ++w) {
        for (int o = 0; o < NOUTCOMES; ++o) {
            Episode e;
            e.world = ws[w];
            e.seed = 101 + o % 3;
            e.repeat = o;
            e.checkpointSteps = 100000LL * (1 + o % 4);
            e.maxSteps = 3000;
            e.steps = 3000;
            e.startDistM = 30.f + 60.f * float(w);
            e.goalTolM = 3.f; e.robotR = 0.6f;
            e.goalX = 56.f; e.goalY = 12.f + 30.f * float(w + 1);
            e.minClearM = 0.4f + rnd();
            e.travelM = 40.f + 120.f * rnd();
            e.netDispM = e.travelM * (0.3f + 0.6f * rnd());
            e.cellsVisited = 20 + int(200 * rnd());
            e.minDistM = e.startDistM * (0.2f + 0.7f * rnd());
            e.distM = e.minDistM + 5.f;
            e.minDistStep = 1500;
            e.rProgress = 40.f * rnd(); e.rCoverage = 8.f * rnd();
            e.rTime = -3.f * rnd();     e.rStop = -20.f * rnd();
            e.rClear = -4.f * rnd();    e.rTerminal = 0.f;
            // Force each outcome, so every colour and every legend row is
            // exercised whatever the random numbers do.
            switch (o) {
                case ARRIVED: e.reachedGoal = true; e.rTerminal = 100.f;
                              e.minDistM = 2.f; e.distM = 2.f; break;
                case COLLIDED_MAPPED: e.collisions = 1; e.hitUnknown = false;
                              e.rTerminal = -300.f; break;
                case COLLIDED_BLIND: e.collisions = 1; e.hitUnknown = true;
                              e.rTerminal = -300.f; break;
                case STALLED_HELD: e.travelM = 0.4f; e.netDispM = 0.4f;
                              e.stoppedSteps = 2900; break;
                case STALLED_STUCK: e.travelM = 0.4f; e.netDispM = 0.4f;
                              e.stoppedSteps = 10; break;
                case ORBITED: e.travelM = 200.f; e.netDispM = 8.f; break;
                case WRONG_WAY: e.travelM = 60.f; e.netDispM = 50.f;
                              e.minDistM = e.startDistM; e.distM = e.startDistM * 1.3f;
                              break;
                case DRIFTED_OFF: e.minDistM = e.startDistM * 0.2f;
                              e.distM = e.startDistM * 0.9f; e.minDistStep = 900;
                              break;
                case CLOSING_BUDGET: e.minDistM = e.startDistM * 0.05f;
                              e.distM = e.minDistM; e.minDistStep = 2990; break;
                default: e.minDistM = e.startDistM * 0.9f; e.distM = e.minDistM;
                         e.minDistStep = 2990; break;
            }
            // A trace, so the curve and map panels have something to draw.
            const int N = 240;
            for (int k = 0; k < N; ++k) {
                const float t = float(k) / float(N - 1);
                e.tstep.push_back(int(t * float(e.steps)));
                const float d = e.startDistM +
                    (e.minDistM - e.startDistM) * std::min(1.f, t * 1.3f);
                e.dist.push_back(d);
                e.trail.push_back({10.f + 40.f * t + 6.f * std::sin(9.f * t),
                                   10.f + 30.f * t * float(w + 1)});
            }

            out.push_back(e);
        }
    }
    return out;
}

int shot(const std::string& prefix) {
    const std::vector<Episode> eps = synthetic();
    const int n = writeAll(eps, prefix);
    std::printf("[report] wrote %d panel(s) to %s_*.png from synthetic records "
                "(nothing was flown)\n", n, prefix.c_str());
    return n ? 0 : 1;
}

int check() {
    const std::vector<Episode> eps = synthetic();
    int bad = 0;

    struct Panel { const char* name; cv::Mat im; };
    std::vector<Panel> panels;
    auto lay = [&](const char* name, cv::Mat (*fn)(const std::vector<Episode>&)) {
        std::vector<cv::Rect> boxes;
        g_boxes = &boxes;
        cv::Mat im = fn(eps);
        g_boxes = nullptr;
        // TEXT OFF THE CANVAS and TEXT OVER TEXT -- the two failures that pass
        // every other test, because nothing is there to collide with in the
        // first case and neither box is a button in the second. Learned from
        // gui --check, which was blind to both until a panel grew a column.
        for (const cv::Rect& t : boxes)
            if ((t & cv::Rect(0, 0, im.cols, im.rows)) != t) {
                std::printf("%s: text runs off the canvas at (%d,%d %dx%d)\n",
                            name, t.x, t.y, t.width, t.height);
                ++bad;
            }
        for (size_t i = 0; i < boxes.size(); ++i)
            for (size_t j = i + 1; j < boxes.size(); ++j)
                if ((boxes[i] & boxes[j]).area() > 0) {
                    std::printf("%s: text overlaps text at (%d,%d) / (%d,%d)\n",
                                name, boxes[i].x, boxes[i].y, boxes[j].x,
                                boxes[j].y);
                    ++bad;
                }
        // A BLANK PNG PASSES EVERYTHING ELSE. It is the failure mode of a
        // plotting routine that silently drew nothing, and the only one that
        // looks like success from the outside.
        cv::Mat g;
        cv::cvtColor(im, g, cv::COLOR_BGR2GRAY);
        double lo = 0, hi = 0;
        cv::minMaxLoc(g, &lo, &hi);
        if (hi - lo < 20) {
            std::printf("%s: panel is uniformly flat -- nothing was drawn\n",
                        name);
            ++bad;
        }
        panels.push_back({name, im});
    };

    lay("outcomes", drawOutcomes);
    lay("curves", drawCurves);
    lay("reward", drawReward);
    lay("progress", drawProgress);

    {
        std::vector<cv::Rect> boxes;
        g_boxes = &boxes;
        cv::Mat im = drawMap(eps, "forest", 101);
        g_boxes = nullptr;
        for (const cv::Rect& t : boxes)
            if ((t & cv::Rect(0, 0, im.cols, im.rows)) != t) {
                std::printf("map: text runs off the canvas at (%d,%d)\n",
                            t.x, t.y);
                ++bad;
            }
        for (size_t i = 0; i < boxes.size(); ++i)
            for (size_t j = i + 1; j < boxes.size(); ++j)
                if ((boxes[i] & boxes[j]).area() > 0) {
                    std::printf("map: text overlaps text at (%d,%d)/(%d,%d)\n",
                                boxes[i].x, boxes[i].y, boxes[j].x, boxes[j].y);
                    ++bad;
                }
    }

    // EVERY OUTCOME MUST BE REACHABLE. classify() and synthetic() are written
    // against each other, so a threshold edited on one side without the other
    // would quietly retire a class -- and the symptom would be a legend that
    // simply never mentions it again.
    std::set<int> seen;
    for (const Episode& e : eps) seen.insert(classify(e));
    for (int o = 0; o < NOUTCOMES; ++o)
        if (!seen.count(o)) {
            std::printf("taxonomy: nothing classifies as '%s' -- the class is "
                        "unreachable\n", outcomeName(o));
            ++bad;
        }
    // Every class needs a distinct colour, or the legend is a lie.
    for (int a = 0; a < NOUTCOMES; ++a)
        for (int b = a + 1; b < NOUTCOMES; ++b)
            if (outcomeColour(a) == outcomeColour(b)) {
                std::printf("taxonomy: '%s' and '%s' share a colour\n",
                            outcomeName(a), outcomeName(b));
                ++bad;
            }

    std::printf("[report check] %d layout violation(s)\n", bad);
    return bad;
}

}  // namespace krep
