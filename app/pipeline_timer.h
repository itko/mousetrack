/// \file
/// Maintainer: Felice Serena
///

#pragma once

#include "pipeline_observer.h"

#include <chrono>
#include <fstream>

namespace MouseTrack {

/// The pipeline timer measures the time for each module of the pipeline but also for the entire pipeline execution.
/// It writes the measured times to the console output but also allows to write it to a file.
class PipelineTimer : public PipelineObserver {
public:
  virtual void pipelineStarted();

  virtual void pipelineTerminated();

  virtual void frameStart(FrameNumber frame);

  virtual void frameEnd(FrameNumber frame);

  virtual void startFrameWindow(FrameNumber f);
  virtual void newFrameWindow(FrameNumber f,
                              std::shared_ptr<const FrameWindow> window);

  virtual void startFrameWindowFiltering(FrameNumber f);
  virtual void
  newFilteredFrameWindow(FrameNumber f,
                         std::shared_ptr<const FrameWindow> window);

  virtual void startRegistration(FrameNumber f);
  virtual void newRawPointCloud(FrameNumber f,
                                std::shared_ptr<const PointCloud> cloud);

  virtual void startPointCloudFiltering(FrameNumber f);
  virtual void newFilteredPointCloud(FrameNumber f,
                                     std::shared_ptr<const PointCloud> cloud);

  virtual void startClustering(FrameNumber f);
  virtual void
  newClusters(FrameNumber f,
              std::shared_ptr<const std::vector<Cluster>> clusters);

  virtual void startDescripting(FrameNumber f);
  virtual void newDescriptors(
      FrameNumber f,
      std::shared_ptr<
          const std::vector<std::shared_ptr<const ClusterDescriptor>>>
          descriptors);

  virtual void startMatching(FrameNumber f);
  virtual void newMatches(FrameNumber f,
                          std::shared_ptr<const std::vector<long>> matches);

  virtual void startControlPoints(FrameNumber f);
  virtual void newControlPoints(
      FrameNumber f,
      std::shared_ptr<const std::vector<Eigen::Vector3d>> controlPoints);

  /// Output file path to write times in csv format, set to empty string if not
  /// desired
  void logPath(const std::string &logPath);
  const std::string &logPath() const;

private:
  typedef std::pair<FrameNumber, std::string> Key;
  void startTimer(FrameNumber f, const std::string &key);
  void stopTimer(FrameNumber f, const std::string &key);
  std::map<Key, std::chrono::system_clock::time_point> _starts;
  std::map<FrameNumber, std::map<std::string, int>> _durations;
  std::string _logPath;
  std::ofstream _logFileHandle;
};

} // namespace MouseTrack
