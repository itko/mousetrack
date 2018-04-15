/// \file
/// Maintainer: Felice Serena
///
///

#include "cli_options.h"

namespace MouseTrack {

op::options_description cli_options() {
  op::options_description desc{"Options"};
  auto ad = desc.add_options();
  // clang-format off
  ad("help,h", "Help screen");
  ad("src-dir,s", op::value<std::string>(), "Source directory to process");
  ad("out-dir,o", op::value<std::string>(), "Target directory for computed results. No results will be written, if not provided.");
  ad("cli,c", "Don't start the Graphical User Interface, run on command line.");
  ad("pipeline-timer", "Measures the execution time of each pipeline step end sends it's output to the log with debug priority.");
  ad("log,l", op::value<std::string>()->default_value("info"), "Set lowest log level to show. Possible options: trace, debug, info, warning, error, fatal, none. Default: info");
  ad("first-frame", op::value<int>(), "Desired lowest start frame (inclusive).");
  ad("last-frame", op::value<int>(), "Desired highest end frame (inclusive).");
  ad("subsample-to", op::value<int>()->default_value(100*000), "Subsample the points cloud such that there are only <n> points.");
  ad("mean-shift-sigma", op::value<double>()->default_value(0.01), "Sigma used for the mean-shift clustering.");
  ad("mean-shift-max-iterations", op::value<int>()->default_value(1000), "Maximum number of iterations for a point before it should converge.");
  ad("mean-shift-merge-threshold", op::value<double>()->default_value(0.001), "Maximum distance of two clusters such that they can still merge");
  ad("mean-shift-convergence-threshold", op::value<double>()->default_value(0.0001), "Maximum distance a point is allowed to travel in an iteration and still being classified as converged.");
  // clang-format on
  return desc;
}

} // MouseTrack
