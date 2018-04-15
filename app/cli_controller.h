/// \file
/// Maintainer: Felice Serena
///
///

#pragma once

#include "controller.h"

namespace MouseTrack {

class CliController : public Controller {

public:
  virtual int main(int argc, char *argv[], op::variables_map &cli_options);
};

} // namespace MouseTrack
