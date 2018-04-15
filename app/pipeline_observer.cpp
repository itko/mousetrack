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

void PipelineObserver::newClusterChains(std::shared_ptr<const std::vector<ClusterChain>> chains) {

}

void PipelineObserver::frameStart(FrameIndex frame) {
    // empty
}

void PipelineObserver::frameEnd(FrameIndex frame) {
    // empty
}

void PipelineObserver::startFrameWindow     (FrameIndex f) {
    // empty
}

void PipelineObserver::newFrameWindow       (FrameIndex f, std::shared_ptr<const FrameWindow> window) {
    // empty
}

void PipelineObserver::startRegistration    (FrameIndex f) {
    // empty
}

void PipelineObserver::newRawPointCloud     (FrameIndex f, std::shared_ptr<const PointCloud> cloud) {
    // empty
}

void PipelineObserver::startPointCloudFiltering(FrameIndex f) {
    // empty
}

void PipelineObserver::newFilteredPointCloud(FrameIndex f, std::shared_ptr<const PointCloud> cloud) {
    // empty
}


void PipelineObserver::startClustering      (FrameIndex f) {
    // empty
}

void PipelineObserver::newClusters          (FrameIndex f, std::shared_ptr<const std::vector<Cluster>> cluster) {
    // empty
}

void PipelineObserver::startDescripting     (FrameIndex f) {
    // empty
}

void PipelineObserver::newDescriptors       (FrameIndex f, std::shared_ptr<const std::vector<std::shared_ptr<const ClusterDescriptor>>> descriptors) {
    // empty
}

void PipelineObserver::startMatching        (FrameIndex f) {
    // empty
}

void PipelineObserver::newMatches           (FrameIndex f, std::shared_ptr<const std::vector<long>> matches) {
    // empty
}

void PipelineObserver::startControlPoints   (FrameIndex f) {
    // empty
}

void PipelineObserver::newControlPoints     (FrameIndex f, std::shared_ptr<const std::vector<Eigen::Vector3d>> controlPoints) {
    // empty
}

} // MouseTrack
