/// \file
/// Maintainer: Felice Serena
///
///

#include "cli_options.h"

namespace MouseTrack {

op::options_description cli_options() {
  op::options_description desc{"Options"};
  desc.add_options()("help,h", "Help screen")(
      "src-dir,s", op::value<std::string>(), "Source directory to process")(
      "out-dir,o", op::value<std::string>(),
      "Target directory for computed results. No results will be written, if "
      "not provided.")(
      "cli,c",
      "Don't start the Graphical User Interface, run on command line.")(
      "pipeline-timer", "Measures the execution time of each pipeline step end "
                        "sends it's output to the log with debug priority.")(
      "log,l", op::value<std::string>()->default_value("info"),
      "Set lowest log level to show. Possible options: trace, debug, info, "
      "warning, error, fatal, none. Default: info")(
      "first-frame", op::value<int>(),
      "Desired lowest start frame (inclusive).")(
      "last-frame", op::value<int>(), "Desired highest end frame (inclusive).")(
      "subsample-to", op::value<int>()->default_value(100 * 000),
      "Subsample the points cloud such that there are only <n> points.")(
      "mean-shift-sigma", op::value<double>()->default_value(0.01),
      "Sigma used for the mean-shift clustering.")(
      "mean-shift-max-iterations", op::value<int>()->default_value(1000),
      "Maximum number of iterations for a point before it should converge.")(
      "mean-shift-merge-threshold", op::value<double>()->default_value(0.001),
      "Maximum distance of two clusters such that they can still merge")(
      "mean-shift-convergence-threshold",
      op::value<double>()->default_value(0.0001),
      "Maximum distance a point is allowed to travel in an iteration and still "
      "being classified as converged.");
  return desc;
}

} // namespace MouseTrack
