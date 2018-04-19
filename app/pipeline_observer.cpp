/// \file
/// Maintainer: Felice Serena
///
///

#include "pipeline_observer.h"

namespace MouseTrack {

void PipelineObserver::pipelineStarted() {
  // empty
}

void PipelineObserver::pipelineTerminated() {
  // empty
}

void PipelineObserver::newClusterChains(
    std::shared_ptr<const std::vector<ClusterChain>> chains) {
  // empty
}

void PipelineObserver::frameStart(FrameNumber frame) {
  // empty
}

void PipelineObserver::frameEnd(FrameNumber frame) {
  // empty
}

void PipelineObserver::startFrameWindow(FrameNumber f) {
  // empty
}

void PipelineObserver::newFrameWindow(
    FrameNumber f, std::shared_ptr<const FrameWindow> window) {
  // empty
}

void PipelineObserver::startFrameWindowFiltering(FrameIndex f) {
  // empty
}

void PipelineObserver::newFilteredFrameWindow(
    FrameIndex f, std::shared_ptr<const FrameWindow> window) {
  // empty
}

void PipelineObserver::startRegistration(FrameIndex f) {
  // empty
}

void PipelineObserver::newRawPointCloud(
    FrameNumber f, std::shared_ptr<const PointCloud> cloud) {
  // empty
}

void PipelineObserver::startPointCloudFiltering(FrameNumber f) {
  // empty
}

void PipelineObserver::newFilteredPointCloud(
    FrameNumber f, std::shared_ptr<const PointCloud> cloud) {
  // empty
}

void PipelineObserver::startClustering(FrameNumber f) {
  // empty
}

void PipelineObserver::newClusters(
    FrameNumber f, std::shared_ptr<const std::vector<Cluster>> cluster) {
  // empty
}

void PipelineObserver::startDescripting(FrameNumber f) {
  // empty
}

void PipelineObserver::newDescriptors(
    FrameNumber f,
    std::shared_ptr<const std::vector<std::shared_ptr<const ClusterDescriptor>>>
        descriptors) {
  // empty
}

void PipelineObserver::startMatching(FrameNumber f) {
  // empty
}

void PipelineObserver::newMatches(
    FrameNumber f, std::shared_ptr<const std::vector<long>> matches) {
  // empty
}

void PipelineObserver::startControlPoints(FrameNumber f) {
  // empty
}

void PipelineObserver::newControlPoints(
    FrameNumber f,
    std::shared_ptr<const std::vector<Eigen::Vector3d>> controlPoints) {
  // empty
}

} // namespace MouseTrack
