
// pybind11
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

// flightlib
#include "flightlib/envs/env_base.hpp"
#include "flightlib/envs/quadrotor_env/quadrotor_env.hpp"
#include "flightlib/envs/quadrotor_env/quadrotor_vec_env.hpp"
#include "flightlib/envs/vec_env_base.hpp"

namespace py = pybind11;
using namespace flightlib;

PYBIND11_MODULE(flightgym, m) {
  py::class_<QuadrotorVecEnv<QuadrotorEnv>>(m, "QuadrotorEnv_v1")
    .def(py::init<>())
    .def(py::init<const std::string&>())
    .def(py::init<const std::string&, const bool>())
    .def("reset",
         static_cast<bool (QuadrotorVecEnv<QuadrotorEnv>::*)(
           Ref<MatrixRowMajor<>>)>(&QuadrotorVecEnv<QuadrotorEnv>::reset),
         "reset")
    .def("reset",
         static_cast<bool (QuadrotorVecEnv<QuadrotorEnv>::*)(
           Ref<MatrixRowMajor<>>, bool)>(&QuadrotorVecEnv<QuadrotorEnv>::reset),
         "reset with random option")
    .def("step", &QuadrotorVecEnv<QuadrotorEnv>::step)
    .def("setSeed", &QuadrotorVecEnv<QuadrotorEnv>::setSeed)
    .def("close", &QuadrotorVecEnv<QuadrotorEnv>::close)
    .def("isTerminalState", &QuadrotorVecEnv<QuadrotorEnv>::isTerminalState)
    .def("curriculumUpdate", &QuadrotorVecEnv<QuadrotorEnv>::curriculumUpdate)
    .def("connectUnity", &QuadrotorVecEnv<QuadrotorEnv>::connectUnity)
    .def("disconnectUnity", &QuadrotorVecEnv<QuadrotorEnv>::disconnectUnity)
    .def("updateUnity", &QuadrotorVecEnv<QuadrotorEnv>::updateUnity)
    .def("getObs", &QuadrotorVecEnv<QuadrotorEnv>::getObs)
    .def("getQuadAct", &QuadrotorVecEnv<QuadrotorEnv>::getQuadAct)
    .def("getQuadState", &QuadrotorVecEnv<QuadrotorEnv>::getQuadState)
    .def("getImage", &QuadrotorVecEnv<QuadrotorEnv>::getImage)
    .def("getDepthImage", &QuadrotorVecEnv<QuadrotorEnv>::getDepthImage)
    .def("getNumOfEnvs", &QuadrotorVecEnv<QuadrotorEnv>::getNumOfEnvs)
    .def("getObsDim", &QuadrotorVecEnv<QuadrotorEnv>::getObsDim)
    .def("getActDim", &QuadrotorVecEnv<QuadrotorEnv>::getActDim)
    .def("getRewDim", &QuadrotorVecEnv<QuadrotorEnv>::getRewDim)
    .def("getImgHeight", &QuadrotorVecEnv<QuadrotorEnv>::getImgHeight)
    .def("getImgWidth", &QuadrotorVecEnv<QuadrotorEnv>::getImgWidth)
    .def("getRewardNames", &QuadrotorVecEnv<QuadrotorEnv>::getRewardNames)
    .def("getExtraInfoNames", &QuadrotorVecEnv<QuadrotorEnv>::getExtraInfoNames)
    .def("__repr__", [](const QuadrotorVecEnv<QuadrotorEnv>& a) {
      return "RPG Drone Control Environment";
    });
}