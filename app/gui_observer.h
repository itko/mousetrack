/// \file
/// Maintainer: Luzian Hug
///


#pragma once

#include "pipeline_observer.h"
#include "camera_view_widget.h"

namespace MouseTrack {

class GUIObserver: public PipelineObserver {
public:
    GUIObserver(CameraViewWidget* widget);

    void pipelineStarted();

    void pipelineTerminated();

    void frameStart(FrameIndex frame);

    void frameEnd(FrameIndex frame);

    void startFrameWindow     (FrameIndex f);
    void newFrameWindow       (FrameIndex f, std::shared_ptr<const FrameWindow> window);
    void startRegistration    (FrameIndex f);
    void newRawPointCloud     (FrameIndex f, std::shared_ptr<const PointCloud> cloud);
    void startClustering      (FrameIndex f);
    void newClusters          (FrameIndex f, std::shared_ptr<const std::vector<Cluster>> clusters);
    void startDescripting     (FrameIndex f);
    void newDescriptors       (FrameIndex f, std::shared_ptr<const std::vector<std::shared_ptr<const ClusterDescriptor>>> descriptors);
    void startMatching        (FrameIndex f);
    void newMatches           (FrameIndex f, std::shared_ptr<const std::vector<long>> matches);
    void startControlPoints   (FrameIndex f);
    void newControlPoints     (FrameIndex f, std::shared_ptr<const std::vector<Eigen::Vector3d>> controlPoints);
private:
    CameraViewWidget* _widget;
};

} // MouseTrack
