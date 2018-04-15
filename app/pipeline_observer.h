/// \file
/// Maintainer: Felice Serena
///

#pragma once

#include "generic/frame_window.h"
#include "generic/types.h"
// Mocks: remove after merge
typedef int FrameIndex;

#include "generic/cluster.h"
#include "generic/cluster_chain.h"
#include "generic/cluster_descriptor.h"
#include "generic/point_cloud.h"

#include <Eigen/Core>

#include <map>
#include <memory>
#include <vector>

namespace MouseTrack {

/// Most methods are passed a shared pointer with the output of a pipeline step
/// for a frame.
/// Any thread can potentially call a method, it is your responsibility to
/// enqueue any data in your run loop.
///
/// Expectations:
///
/// - You should perform only minimal work in the method:
///   Usually you only want to copy the shared pointer and enqueue a job in your
///   run loop
///   to process it later. Copying a shared pointer is thread safe.
/// - You are not allowed to modify the data held by the shared pointer,
///   other threads might also be accessing it.
///
///
/// Guarantees:
///
/// - No concurrent calls: Only one method is called at a time by only one
/// thread.
/// - Sequential FrameIndexes: It is guaranteed, that calls with the same frame
/// index
///   are called in orderd. It is also guaranteed, that for a processing step,
///   the passed frame indices increase monotonically.
///   Example: frameStart(2) always is called before frameStart(3).
///   startFrameWindow(2) is always called before newFrameWindow(2).
///
/// Notes:
/// - You don't need to impement all methods. By default, each method is a noop.
class PipelineObserver {
public:
  /// Pipeline starts to process data
  virtual void pipelineStarted();

  /// Pipeline finished all work.
  virtual void pipelineTerminated();

  /// When the pipeline terminates, it delivers the complete cluster chain used
  /// internally.
  virtual void
  newClusterChains(std::shared_ptr<const std::vector<ClusterChain>> chains);

  /// Work on frame `frame` started, it is inside the pipeline.
  virtual void frameStart(FrameIndex frame);

  /// Work on frame `frame` finished, it is no longer processed by the pipeline.
  virtual void frameEnd(FrameIndex frame);

  // see descriptions of the corresponding module interfaces

  // clang-format off
    virtual void startFrameWindow     (FrameIndex f);
    virtual void newFrameWindow       (FrameIndex f, std::shared_ptr<const FrameWindow> window);

    virtual void startRegistration    (FrameIndex f);
    virtual void newRawPointCloud     (FrameIndex f, std::shared_ptr<const PointCloud> cloud);

    virtual void startPointCloudFiltering(FrameIndex f);
    virtual void newFilteredPointCloud(FrameIndex f, std::shared_ptr<const PointCloud> cloud);

    virtual void startClustering      (FrameIndex f);
    virtual void newClusters          (FrameIndex f, std::shared_ptr<const std::vector<Cluster>> clusters);

    virtual void startDescripting     (FrameIndex f);
    virtual void newDescriptors       (FrameIndex f, std::shared_ptr<const std::vector<std::shared_ptr<const ClusterDescriptor>>> descriptors);

    virtual void startMatching        (FrameIndex f);
    virtual void newMatches           (FrameIndex f, std::shared_ptr<const std::vector<long>> matches);

    virtual void startControlPoints   (FrameIndex f);
    virtual void newControlPoints     (FrameIndex f, std::shared_ptr<const std::vector<Eigen::Vector3d>> controlPoints);
  // clang-format on
};
} // namespace MouseTrack
