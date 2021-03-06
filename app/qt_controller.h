/// \file
/// Maintainer: Luzian Hug
///
///

#pragma once

#include "controller.h"

namespace MouseTrack {

class QtController : public Controller {

public:
  virtual int main(int argc, char *argv[], op::variables_map &cli_options);
};

} // namespace MouseTrack
