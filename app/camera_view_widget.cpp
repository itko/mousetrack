/// \file
/// Maintainer: Luzian Hug
///
///

#include "camera_view_widget.h"
#include <QPixmap>
#include <QImage>

namespace MouseTrack {


 CameraViewWidget::CameraViewWidget() {
     //empty
 }
  CameraViewWidget::~CameraViewWidget() {
      //empty
  }
  void  CameraViewWidget::update() {
      Frame frame0 = _window->frames()[0];
      Picture pic = frame0.referencePicture;
      QImage* image = new QImage(pic.cols(),pic.rows(),QImage::Format_Grayscale8);
      for(int x=0; x < pic.cols(); x++ ) {
          for(int y=0; y < pic.rows(); y++) {
              double value = pic(y,x);
              BOOST_LOG_TRIVIAL(debug) << value;
              image->setPixel(x,y,(int) value);
          }
      }
      QPixmap pixmap = QPixmap::fromImage(*image);
      this->setPixmap(pixmap);


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
