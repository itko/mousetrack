/// \file
/// Maintainer: Felice Serena
///
///

#include "cli_controller.h"
#include "cli_options.h"
#include "pipeline_timer.h"

namespace MouseTrack {

int CliController::main(int argc, char *argv[]){
    PipelineTimer timer;

    pipeline().addObserver(&timer);

    // start pipeline
    pipeline().start();

    // wait for pipeline to terminate
    pipeline().join();

    return 0;
}

} // MouseTrack
