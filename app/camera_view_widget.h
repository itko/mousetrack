/// \file
/// Maintainer: Luzian Hug
///
///

#pragma once

#include <QLabel>
#include <memory>
#include <vector>
#include "Eigen/Core"

#include "generic/frame_window.h"
#include "generic/point_cloud.h"
#include "generic/cluster.h"
#include "generic/cluster_descriptor.h"
#include <boost/log/trivial.hpp>

namespace MouseTrack {

class CameraViewWidget : public QLabel {
public:
  CameraViewWidget();
  ~CameraViewWidget();
  void update();
  void setFrameWindow(std::shared_ptr<const FrameWindow> window);
  void setPointCloud(std::shared_ptr<const PointCloud> cloud);
  void setClusters(std::shared_ptr<const std::vector<Cluster>> clusters);
  void setDescriptors(std::shared_ptr<const std::vector<std::shared_ptr<const ClusterDescriptor>>> descriptors);
  void setMatches(std::shared_ptr<const std::vector<long>> matches);
  void setControlPoints(std::shared_ptr<const std::vector<Eigen::Vector3d>> controlPoints);

private:
std::shared_ptr<const FrameWindow> _window;
std::shared_ptr<const PointCloud> _cloud;
std::shared_ptr<const std::vector<Cluster>> _clusters;
std::shared_ptr<const std::vector<std::shared_ptr<const ClusterDescriptor>>> _descriptors;
std::shared_ptr<const std::vector<long>> _matches;
std::shared_ptr<const std::vector<Eigen::Vector3d>> _controlPoints;
};
} //MouseTrack
