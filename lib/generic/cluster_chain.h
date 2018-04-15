/// \file
/// Maintainer: Felice Serena
///
///

#pragma once

#include "cluster.h"
#include "cluster_descriptor.h"

#include <map>
#include <memory>

namespace MouseTrack {

/// Holds two maps of pointers to clusters and cluster descriptors
/// The indexes corresond to the according frame number
/// The key does not exist, if no cluster was found in a frame
class ClusterChain {
public:
  /// Write access to all clusters.
  std::map<FrameNumber, const Cluster *> &clusters();

  /// Read access to all clusters.
  const std::map<FrameNumber, const Cluster *> &clusters() const;

  /// Write access to all cluster descriptors.
  std::map<FrameNumber, std::shared_ptr<const ClusterDescriptor>> &
  descriptors();

  /// Read access to all cluster descriptors
  const std::map<FrameNumber, std::shared_ptr<const ClusterDescriptor>> &
  descriptors() const;

private:
  std::map<FrameNumber, std::shared_ptr<const ClusterDescriptor>> _descriptors;
  std::map<FrameNumber, const Cluster *> _clusters;
};

} // namespace MouseTrack
