/// \file
/// Maintainer: Luzian Hug
///

#include "gui_observer.h"


namespace MouseTrack {



    GUIObserver::GUIObserver(CameraViewWidget* widget) : _widget(widget) {
      //empty
    }

    void GUIObserver::pipelineStarted() {
      //empty
    }

    void GUIObserver::pipelineTerminated() {
      //empty
    }

    void GUIObserver::frameStart(FrameIndex frame) {
      //empty
    }

    void GUIObserver::frameEnd(FrameIndex frame) {
      _widget->update();
    }

    void GUIObserver::startFrameWindow     (FrameIndex f) {
      //empty
    }
    void GUIObserver::newFrameWindow       (FrameIndex f, std::shared_ptr<const FrameWindow> window) {
      _widget->setFrameWindow(window);
    }
    void GUIObserver::startRegistration    (FrameIndex f) {
      //empty
    }
    void GUIObserver::newRawPointCloud     (FrameIndex f, std::shared_ptr<const PointCloud> cloud) {
      _widget->setPointCloud(cloud);
    }
    void GUIObserver::startClustering      (FrameIndex f) {
      //empty
    }
    void GUIObserver::newClusters          (FrameIndex f, std::shared_ptr<const std::vector<Cluster>> clusters) {
      _widget->setClusters(clusters);
    }
    void GUIObserver::startDescripting     (FrameIndex f) {
      //empty
    }
    void GUIObserver::newDescriptors       (FrameIndex f, std::shared_ptr<const std::vector<std::shared_ptr<const ClusterDescriptor>>> descriptors) {
      _widget->setDescriptors(descriptors);
    }
    void GUIObserver::startMatching        (FrameIndex f) {
      //empty
    }
    void GUIObserver::newMatches           (FrameIndex f, std::shared_ptr<const std::vector<long>> matches) {
      _widget->setMatches(matches);
    }
    void GUIObserver::startControlPoints   (FrameIndex f) {
      //empty
    }
    void GUIObserver::newControlPoints     (FrameIndex f, std::shared_ptr<const std::vector<Eigen::Vector3d>> controlPoints) {
      _widget->setControlPoints(controlPoints);
    }


} // MouseTrack
