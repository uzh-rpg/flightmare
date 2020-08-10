#include "flightlib/common/logger.hpp"

#include <gtest/gtest.h>

#include <cmath>

#include "flightlib/common/quad_state.hpp"
#include "flightlib/common/timer.hpp"
#include "flightlib/common/types.hpp"

using namespace flightlib;

TEST(Logger, SimpleLogging) {
  Logger logger("Test");
  Timer timer("Timer", "Printing");
  timer.tic();

  logger << "This is a text stream log." << std::endl;
  logger.info("This is an info log.");
  logger.warn("This could be a warning, but just for demo.");
  logger.error("This could be an error, but just for demo.");

  logger.info(
    "You can print strings like \"%s\", and formatted numbers like %1.3f.",
    "text", M_PI);
  logger
    << "You can use the stream operator \'<<\' just like with \'std::cout\'."
    << std::endl;
  logger << "This can be helpul for printing complex objects like Eigen vector "
            "and matrices:"
         << std::endl
         << "A vector:" << std::endl
         << Vector<4>(0, 1, 2, 3).transpose() << std::endl
         << "A Matrix:" << std::endl
         << Matrix<3, 3>::Identity() << std::endl;

  timer.toc();
  QuadState state;
  state.setZero();
  logger << "And also our own defined objects, like so:" << std::endl
         << "A timer:" << std::endl
         << timer << std::endl
         << "A QuadState" << std::endl
         << state << std::endl;
}

TEST(Logger, NoColorLogging) {
  Logger logger("Test", false);

  logger << "This is a text stream log." << std::endl;
  logger.info("This is an info log.");
  logger.warn("This could be a warning, but just for demo.");
  logger.error("This could be an error, but just for demo.");
}