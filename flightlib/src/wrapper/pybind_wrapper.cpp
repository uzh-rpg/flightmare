
// pybind11
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

// flightlib
#include "flightlib/envs/env_base.hpp"
#include "flightlib/envs/quadrotor_env/quadrotor_env.hpp"
#include "flightlib/envs/test_env.hpp"
#include "flightlib/envs/vec_env.hpp"

namespace py = pybind11;
using namespace flightlib;

PYBIND11_MODULE(flightgym, m) {
  py::class_<VecEnv<QuadrotorEnv>>(m, "QuadrotorEnv_v1")
    .def(py::init<>())
    .def(py::init<const std::string&>())
    .def("reset", &VecEnv<QuadrotorEnv>::reset)
    .def("step", &VecEnv<QuadrotorEnv>::step)
    .def("testStep", &VecEnv<QuadrotorEnv>::testStep)
    .def("setSeed", &VecEnv<QuadrotorEnv>::setSeed)
    .def("close", &VecEnv<QuadrotorEnv>::close)
    .def("isTerminalState", &VecEnv<QuadrotorEnv>::isTerminalState)
    .def("curriculumUpdate", &VecEnv<QuadrotorEnv>::curriculumUpdate)
    .def("connectFlightmare", &VecEnv<QuadrotorEnv>::connectFlightmare)
    .def("disconnectFlightmare", &VecEnv<QuadrotorEnv>::disconnectFlightmare)
    .def("getNumOfEnvs", &VecEnv<QuadrotorEnv>::getNumOfEnvs)
    .def("getObsDim", &VecEnv<QuadrotorEnv>::getObsDim)
    .def("getActDim", &VecEnv<QuadrotorEnv>::getActDim)
    .def("getExtraInfoNames", &VecEnv<QuadrotorEnv>::getExtraInfoNames)
    .def("__repr__", [](const VecEnv<QuadrotorEnv>& a) {
      return "RPG Drone Racing Environment";
    });

  py::class_<QuadrotorEnv>(m, "QuadrotorEnv_v0")
    .def(py::init<>())
    .def("reset", &QuadrotorEnv::reset)
    .def("step", &QuadrotorEnv::step)
    .def("setSeed", &QuadrotorEnv::setSeed)
    .def("close", &QuadrotorEnv::close)
    .def("isTerminalState", &QuadrotorEnv::isTerminalState)
    .def("connectFlightmare", &QuadrotorEnv::connectFlightmare)
    .def("disconnectFlightmare", &QuadrotorEnv::disconnectFlightmare)
    .def("getObsDim", &QuadrotorEnv::getObsDim)
    .def("getActDim", &QuadrotorEnv::getActDim)
    .def("__repr__",
         [](const QuadrotorEnv& a) { return "RPG Drone Racing Environment"; });

  py::class_<TestEnv<QuadrotorEnv>>(m, "TestEnv_v0")
    .def(py::init<>())
    .def("reset", &TestEnv<QuadrotorEnv>::reset)
    .def("__repr__", [](const TestEnv<QuadrotorEnv>& a) { return "Test Env"; });
}