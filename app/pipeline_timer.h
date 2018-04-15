/// \file
/// Maintainer: Felice Serena
///

#pragma once

#include "pipeline_observer.h"
#include <chrono>

namespace MouseTrack {

class PipelineTimer : public PipelineObserver {
public:
  virtual void pipelineStarted();

  virtual void pipelineTerminated();

  virtual void frameStart(FrameIndex frame);

  virtual void frameEnd(FrameIndex frame);

  virtual void startFrameWindow(FrameIndex f);
  virtual void newFrameWindow(FrameIndex f,
                              std::shared_ptr<const FrameWindow> window);

  virtual void startRegistration(FrameIndex f);
  virtual void newRawPointCloud(FrameIndex f,
                                std::shared_ptr<const PointCloud> cloud);

  virtual void startPointCloudFiltering(FrameIndex f);
  virtual void newFilteredPointCloud(FrameIndex f,
                                     std::shared_ptr<const PointCloud> cloud);

  virtual void startClustering(FrameIndex f);
  virtual void
  newClusters(FrameIndex f,
              std::shared_ptr<const std::vector<Cluster>> clusters);

  virtual void startDescripting(FrameIndex f);
  virtual void newDescriptors(
      FrameIndex f,
      std::shared_ptr<
          const std::vector<std::shared_ptr<const ClusterDescriptor>>>
          descriptors);

  virtual void startMatching(FrameIndex f);
  virtual void newMatches(FrameIndex f,
                          std::shared_ptr<const std::vector<long>> matches);

  virtual void startControlPoints(FrameIndex f);
  virtual void newControlPoints(
      FrameIndex f,
      std::shared_ptr<const std::vector<Eigen::Vector3d>> controlPoints);

private:
  typedef std::pair<FrameIndex, std::string> Key;
  void startTimer(FrameIndex f, const std::string &key);
  void stopTimer(FrameIndex f, const std::string &key);
  std::map<Key, std::chrono::system_clock::time_point> _starts;
};

} // namespace MouseTrack
