/// \file
/// Maintainer: Felice Serena
///
///

#pragma once

#include "matlab_reader.h"

namespace MouseTrack {

class MatlabReaderConcurrent : public MatlabReader {
public:
  MatlabReaderConcurrent(fs::path root_directory);

  virtual FrameWindow frameWindow(FrameNumber f) const;
};

} // namespace MouseTrack
