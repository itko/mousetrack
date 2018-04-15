/// \file
/// Maintainer: Felice Serena
///
///

#pragma once

#include "pipeline.h"
#include "types.h"

namespace MouseTrack {

class PipelineFactory {
public:
  /// temporary solution until we have some internal representation of the
  /// pipeline configuration
  Pipeline fromCliOptions(const op::variables_map &options) const;

private:
  std::unique_ptr<Clustering>
  getClustering(const op::variables_map &options) const;
};

} // namespace MouseTrack
