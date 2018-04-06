/// \file
/// Maintainer: Luzian Hug
///
///

#include "camera_view_widget.h"

namespace MouseTrack {


 CameraViewWidget::CameraViewWidget() {
     //empty
 }
  CameraViewWidget::~CameraViewWidget() {
      //empty
  }
  void  CameraViewWidget::update() {
      //TODO
  }

  void CameraViewWidget::setFrameWindow(std::shared_ptr<const FrameWindow> window) {
      _window = window;
  }
  void CameraViewWidget::setPointCloud(std::shared_ptr<const PointCloud> cloud) {
      _cloud = cloud;
  }
  void CameraViewWidget::setClusters(std::shared_ptr<const std::vector<Cluster>> clusters) {
      _clusters = clusters;
  }
  void CameraViewWidget::setDescriptors(std::shared_ptr<const std::vector<std::shared_ptr<const ClusterDescriptor>>> descriptors) {
      _descriptors = descriptors;
  }
  void CameraViewWidget::setMatches(std::shared_ptr<const std::vector<long>> matches) {
      _matches = matches;
  }
  void CameraViewWidget::setControlPoints(std::shared_ptr<const std::vector<Eigen::Vector3d>> controlPoints) {
      _controlPoints = controlPoints;
  }


} //MouseTrack
