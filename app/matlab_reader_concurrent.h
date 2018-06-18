/// \file
/// Maintainer: Felice Serena
///
///

#pragma once

#include "matlab_reader.h"

namespace MouseTrack {

/// A variation of the MatlabReader to check if concurrent reads are worth it.
class MatlabReaderConcurrent : public MatlabReader {
public:
  /// Root directory to search (file content as extracted by the matlab script)
  MatlabReaderConcurrent(fs::path root_directory);

  virtual FrameWindow frameWindow(FrameNumber f) const;
};

} // namespace MouseTrack
