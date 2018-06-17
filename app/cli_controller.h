/// \file
/// Maintainer: Felice Serena
///
///

#pragma once

#include "controller.h"


/// Application controller for command line mode.
///
///
namespace MouseTrack {

class CliController : public Controller {
private:
  bool _isTerminating = false;

public:
  virtual int main(int argc, char *argv[], op::variables_map &cli_options);
  void handleSignal(int signal);
};

} // namespace MouseTrack
