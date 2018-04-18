/// \file
/// Maintainer: Felice Serena
///
///

#pragma once

#include "pipeline.h"
#include "types.h"

namespace MouseTrack {

/// This class knows about all pipeline modules and how to connect them to
/// create a valid Pipeline.
class PipelineFactory {
public:
  /// temporary solution until we have some internal representation of the
  /// pipeline configuration
  Pipeline fromCliOptions(const op::variables_map &options) const;

private:
  /// Depending on the given options, choose, create and return a reader
  std::unique_ptr<Reader> getReader(const op::variables_map &options) const;

  /// Depending on the given options, choose, create and return a frame window
  /// filter
  std::unique_ptr<FrameWindowFiltering>
  getWindowFiltering(const op::variables_map &options) const;

  /// Depending on the given options, choose, create and return a registration
  /// module
  std::unique_ptr<Registration>
  getRegistration(const op::variables_map &options) const;

  /// Depending on the given options, choose, create and return a point cloud
  /// filtering module
  std::unique_ptr<PointCloudFiltering>
  getCloudFiltering(const op::variables_map &options) const;

  /// Depending on the given options, choose, create and return a clustering
  /// module
  std::unique_ptr<Clustering>
  getClustering(const op::variables_map &options) const;

  /// Depending on the given options, choose, create and return a descripting
  /// module
  std::unique_ptr<Descripting>
  getDescripting(const op::variables_map &options) const;

  /// Depending on the given options, choose, create and return a matching
  /// module
  std::unique_ptr<Matching> getMatching(const op::variables_map &options) const;

  /// Depending on the given options, choose, create and return a trajectory
  /// builder module
  std::unique_ptr<TrajectoryBuilder>
  getTrajectoryBuilder(const op::variables_map &options) const;
};

} // namespace MouseTrack
