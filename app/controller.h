/// \file
/// Maintainer: Felice Serena
///
///

#pragma once

#include "pipeline.h"
#include "types.h"

namespace MouseTrack {

class Controller {
public:
  /// Called by main after creation.
  /// It passes the command line arguments in raw and in parsed form.
  /// The parsed form is more convenient, but a GUI module you use, might need
  /// argc/argv or you decide to add additional controller specific options.
  ///
  /// args, argv: the arguments passed to main
  ///
  /// cli_options: the command line arguments in parsed form
  virtual int main(int argc, char *argv[], op::variables_map &cli_options) = 0;

  /// Read-write accessor for pipeline controll
  Pipeline &pipeline();

  /// Read-only accessor
  const Pipeline &pipeline() const;

private:
  Pipeline _pipeline;
};

} // namespace MouseTrack
