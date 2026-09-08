// pybind11 bindings for VoxelEnv.
//
// IN-PROCESS ON PURPOSE. The observation is ~1900 floats and the step is a few
// milliseconds, so a socket or a pipe per step would cost a meaningful fraction
// of the step itself. Each SubprocVecEnv worker holds its own C++ env in its own
// process; nothing is shared and nothing is serialised.
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include <cmath>
#include <cstring>

#include "rl_env.hpp"

namespace py = pybind11;
using namespace sim;

PYBIND11_MODULE(voxelenv, m) {
    m.doc() = "Voxel navigation environment: rank admissible primitives.";

    py::class_<EnvConfig>(m, "EnvConfig")
        .def(py::init<>())
        .def_readwrite("world", &EnvConfig::world)
        .def_readwrite("seed", &EnvConfig::seed)
        .def_readwrite("max_steps", &EnvConfig::maxSteps)
        .def_readwrite("cam_w", &EnvConfig::camW)
        .def_readwrite("cam_h", &EnvConfig::camH)
        .def_readwrite("truth_depth", &EnvConfig::truthDepth)
        .def_readwrite("horizon_s", &EnvConfig::horizonS)
        .def_readwrite("robot_r", &EnvConfig::robotR)
        .def_readwrite("w_progress", &EnvConfig::wProgress)
        .def_readwrite("progress_scale_m", &EnvConfig::progressScaleM)
        .def_readwrite("w_coverage", &EnvConfig::wCoverage)
        .def_readwrite("w_time", &EnvConfig::wTime)
        .def_readwrite("w_stop", &EnvConfig::wStop)
        .def_readwrite("w_clear", &EnvConfig::wClear)
        .def_readwrite("r_goal", &EnvConfig::rGoal)
        .def_readwrite("r_collide", &EnvConfig::rCollide)
        // False lets the policy select primitives the geometry rejected, so it
        // can collide and must learn avoidance rather than being handed it.
        .def_readwrite("mask_unsafe", &EnvConfig::maskUnsafe)
        // Sample start and goal per episode instead of using one fixed
        // journey, so the goal channels have to be used.
        .def_readwrite("vary_goal", &EnvConfig::varyGoal);

    py::class_<EnvStep>(m, "EnvStep")
        .def_readonly("reward", &EnvStep::reward)
        .def_readonly("done", &EnvStep::done)
        .def_readonly("truncated", &EnvStep::truncated)
        // The scorecard, named to match sweep.sh's columns so evaluation can
        // emit the same table the classical planners are already reported in.
        .def_readonly("travel_m", &EnvStep::travelM)
        .def_readonly("dist_to_goal_m", &EnvStep::distToGoalM)
        .def_readonly("min_clear_m", &EnvStep::minClearM)
        // Closest approach and when: "never got there" and "got there and
        // drifted off" are different failures and the final distance hides it.
        .def_readonly("min_dist_to_goal_m", &EnvStep::minDistToGoalM)
        .def_readonly("min_dist_step", &EnvStep::minDistStep)
        .def_readonly("collisions", &EnvStep::collisions)
        .def_readonly("stopped_steps", &EnvStep::stoppedSteps)
        .def_readonly("steps", &EnvStep::steps)
        .def_readonly("reached_goal", &EnvStep::reachedGoal)
        // Episode totals per reward term -- which term a policy actually
        // improved, which a scalar return cannot say.
        .def_readonly("r_progress", &EnvStep::rProgress)
        .def_readonly("r_coverage", &EnvStep::rCoverage)
        .def_readonly("r_time", &EnvStep::rTime)
        .def_readonly("r_stop", &EnvStep::rStop)
        .def_readonly("r_clear", &EnvStep::rClear)
        .def_readonly("r_terminal", &EnvStep::rTerminal);

    py::enum_<BaselinePolicy>(m, "Baseline")
        .value("random", BaselinePolicy::Random)
        .value("freeM",  BaselinePolicy::FreeM)
        .value("goal",   BaselinePolicy::Goal)
        .value("score",  BaselinePolicy::Score);

    // The classical planners, from the SAME C++ that `kestrel bench` runs.
    // Reimplementing them in python would let the two drift, and then the
    // learned policy would be compared against something that is not the
    // baseline anyone else measured.
    m.def("choose_baseline", [](BaselinePolicy pol, py::array_t<float> obs,
                                py::array_t<bool> mask, int n_prims, unsigned rng) {
        std::vector<float> o(obs.data(), obs.data() + obs.size());
        std::vector<uint8_t> mk(mask.size());
        auto mv = mask.unchecked<1>();
        for (py::ssize_t i = 0; i < mask.size(); ++i) mk[size_t(i)] = mv(i) ? 1 : 0;
        const int a = chooseBaseline(pol, o, mk, n_prims, rng);
        return py::make_tuple(a, rng);          // rng advanced, so callers stay pure
    }, py::arg("policy"), py::arg("obs"), py::arg("mask"), py::arg("n_prims"),
       py::arg("rng"));

    py::class_<VoxelEnv>(m, "VoxelEnv")
        .def(py::init<const EnvConfig&>(), py::arg("config") = EnvConfig())
        .def("reset", &VoxelEnv::reset, py::arg("world"), py::arg("seed"))
        .def("step", &VoxelEnv::step, py::arg("primitive_index"))
        .def("observation", [](const VoxelEnv& e) {
            const auto& o = e.observation();
            return py::array_t<float>(py::ssize_t(o.size()), o.data());
        })
        // Boolean per primitive. A policy MUST mask its logits with this;
        // selecting a masked index is treated as a hold and penalised, never as
        // an error, so an untrained policy degrades rather than crashing.
        .def("action_mask", [](const VoxelEnv& e) {
            const auto& mk = e.actionMask();
            py::array_t<bool> a(py::ssize_t(mk.size()));
            auto v = a.mutable_unchecked<1>();
            for (py::ssize_t i = 0; i < (py::ssize_t)mk.size(); ++i) v(i) = mk[i] != 0;
            return a;
        })
        // One small BGR pane of the map the aircraft has built, for watching a
        // run. Shaped (px, px, 3) so it goes straight into cv2.imshow.
        .def("render_frame", [](const VoxelEnv& e, int w, int h, bool top_down) {
            std::vector<uint8_t> buf = e.renderFrame(w, h, top_down);
            // The renderer clamps its own bounds, so the shape is read back
            // from what it returned rather than from what was asked for --
            // otherwise a clamped request would reshape a short buffer.
            const py::ssize_t n = py::ssize_t(buf.size());
            py::ssize_t hh = h, ww = w;
            if (n != py::ssize_t(w) * h * 3) { hh = n / (3 * py::ssize_t(w)); }
            py::array_t<uint8_t> a({hh, ww, py::ssize_t(3)});
            std::memcpy(a.mutable_data(), buf.data(), size_t(n));
            return a;
        }, py::arg("w") = 320, py::arg("h") = 240, py::arg("top_down") = false)
        .def_property_readonly("n_prims", &VoxelEnv::nPrims)
        .def_property_readonly("obs_size", &VoxelEnv::obsSize)
        .def_static("features_per_prim", &VoxelEnv::obsFeaturesPerPrim)
        .def_static("global_features", &VoxelEnv::obsGlobalFeatures);
}
