// pybind11 bindings for VoxelEnv.
//
// IN-PROCESS ON PURPOSE. The observation is ~1900 floats and the step is a few
// milliseconds, so a socket or a pipe per step would cost a meaningful fraction
// of the step itself. Each SubprocVecEnv worker holds its own C++ env in its own
// process; nothing is shared and nothing is serialised.
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

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
        .def_readwrite("w_coverage", &EnvConfig::wCoverage)
        .def_readwrite("w_time", &EnvConfig::wTime)
        .def_readwrite("w_stop", &EnvConfig::wStop)
        .def_readwrite("w_clear", &EnvConfig::wClear)
        .def_readwrite("r_goal", &EnvConfig::rGoal)
        .def_readwrite("r_collide", &EnvConfig::rCollide);

    py::class_<EnvStep>(m, "EnvStep")
        .def_readonly("reward", &EnvStep::reward)
        .def_readonly("done", &EnvStep::done)
        .def_readonly("truncated", &EnvStep::truncated)
        // The scorecard, named to match sweep.sh's columns so evaluation can
        // emit the same table the classical planners are already reported in.
        .def_readonly("travel_m", &EnvStep::travelM)
        .def_readonly("dist_to_goal_m", &EnvStep::distToGoalM)
        .def_readonly("min_clear_m", &EnvStep::minClearM)
        .def_readonly("collisions", &EnvStep::collisions)
        .def_readonly("stopped_steps", &EnvStep::stoppedSteps)
        .def_readonly("steps", &EnvStep::steps)
        .def_readonly("reached_goal", &EnvStep::reachedGoal);

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
        .def_property_readonly("n_prims", &VoxelEnv::nPrims)
        .def_property_readonly("obs_size", &VoxelEnv::obsSize)
        .def_static("features_per_prim", &VoxelEnv::obsFeaturesPerPrim)
        .def_static("global_features", &VoxelEnv::obsGlobalFeatures);
}
