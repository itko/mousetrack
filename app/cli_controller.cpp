/// \file
/// Maintainer: Felice Serena
///
///

#include "cli_controller.h"
#include "cli_options.h"

namespace MouseTrack {

int CliController::main(int argc, char *argv[],
                        op::variables_map &cli_options) {
  // start pipeline
  pipeline().start();

  // wait for pipeline to terminate
  pipeline().join();

  return 0;
}

} // namespace MouseTrack
