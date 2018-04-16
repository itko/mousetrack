/// \file
/// Maintainer: Felice Serena
///
///

#include "pipeline_timer.h"
#include <boost/log/trivial.hpp>

namespace MouseTrack {

void PipelineTimer::pipelineStarted() { startTimer(-1, "pipeline"); }

void PipelineTimer::pipelineTerminated() { stopTimer(-1, "pipeline"); }

void PipelineTimer::frameStart(FrameIndex f) { startTimer(f, "total"); }

void PipelineTimer::frameEnd(FrameIndex f) { stopTimer(f, "total"); }

void PipelineTimer::startFrameWindow(FrameIndex f) {
  startTimer(f, "readFrameWindow");
}

void PipelineTimer::newFrameWindow(FrameIndex f,
                                   std::shared_ptr<const FrameWindow> window) {
  stopTimer(f, "readFrameWindow");
}

void PipelineTimer::startFrameWindowFiltering(FrameIndex f) {
  startTimer(f, "frameWindowFiltering");
}

void PipelineTimer::newFilteredFrameWindow(
    FrameIndex f, std::shared_ptr<const FrameWindow> window) {
  stopTimer(f, "frameWindowFiltering");
}

void PipelineTimer::startRegistration(FrameIndex f) {
  startTimer(f, "registration");
}

void PipelineTimer::newRawPointCloud(FrameIndex f,
                                     std::shared_ptr<const PointCloud> cloud) {
  stopTimer(f, "registration");
}

void PipelineTimer::startPointCloudFiltering(FrameIndex f) {
  startTimer(f, "point_cloud_filtering");
}

void PipelineTimer::newFilteredPointCloud(
    FrameIndex f, std::shared_ptr<const PointCloud> cloud) {
  stopTimer(f, "point_cloud_filtering");
}

void PipelineTimer::startClustering(FrameIndex f) {
  startTimer(f, "clustering");
}

void PipelineTimer::newClusters(
    FrameIndex f, std::shared_ptr<const std::vector<Cluster>> clusters) {
  stopTimer(f, "clustering");
}

void PipelineTimer::startDescripting(FrameIndex f) {
  startTimer(f, "descripting");
}

void PipelineTimer::newDescriptors(
    FrameIndex f,
    std::shared_ptr<const std::vector<std::shared_ptr<const ClusterDescriptor>>>
        descriptors) {
  stopTimer(f, "descripting");
}

void PipelineTimer::startMatching(FrameIndex f) { startTimer(f, "matching"); }

void PipelineTimer::newMatches(
    FrameIndex f, std::shared_ptr<const std::vector<long>> matches) {
  stopTimer(f, "matching");
}
void PipelineTimer::startControlPoints(FrameIndex f) {
  startTimer(f, "controlPoints");
}
void PipelineTimer::newControlPoints(
    FrameIndex f,
    std::shared_ptr<const std::vector<Eigen::Vector3d>> controlPoints) {
  stopTimer(f, "controlPoints");
}

void PipelineTimer::startTimer(FrameIndex f, const std::string &key) {
  _starts[Key(f, key)] = std::chrono::system_clock::now();
}

void PipelineTimer::stopTimer(FrameIndex f, const std::string &key) {
  auto start = _starts.find(Key(f, key));
  if (start == _starts.end()) {
    BOOST_LOG_TRIVIAL(warning)
        << "Couldn't find timer start for key (" << f << ", " << key << ")";
    return;
  }
  auto duration = std::chrono::system_clock::now() - start->second;
  _starts.erase(start);
  int d =
      std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
  BOOST_LOG_TRIVIAL(info) << "(" << f << ", " << key << "): " << d / 1000.0
                          << " milliseconds";
}

} // namespace MouseTrack
