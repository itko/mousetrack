/// \file
/// Maintainer: Luzian Hug
///

#include "background_subtraction.h"

#include "Eigen/Core"

#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <boost/log/trivial.hpp>

namespace MouseTrack {
  BackgroundSubtraction::BackgroundSubtraction(const FrameWindow* const cage_frame) {
    //empty
  }

  FrameWindow BackgroundSubtraction::operator()(const FrameWindow &window) {
      // Initializing return object
      std::vector<Frame> frames (window.frames().size());
      
      // Cycle through all streams
      for (size_t i = 0; i < _cage_frame->frames().size(); i++) {
        // Copy the stuff we need from the FrameWindow objects
        Picture cage_image = _cage_frame->frames()[i].referencePicture;
        Picture mouse_image = window.frames()[i].referencePicture;

        // ALL the dimensions must match
        if (!(cage_image.rows() == mouse_image.rows() && cage_image.cols() == mouse_image.cols())) {
                  BOOST_LOG_TRIVIAL(warning) << "Image dimensions don't match - background subtraction could not be performed";
                  // If it fails, return the input (do nothing)
                  return window;
                }

                // Perform the subtraction
                Picture sub = (mouse_image - cage_image).array().abs();

                // Convert to opencv format and from [0,1] to [0,255] format
                cv::Mat subcv;
                sub = Eigen::floor(sub.array() * 255);
                cv::eigen2cv(sub, subcv);

                // Apply Otsu's Method for thresholding
                cv::Mat maskcv;
                double thresh_otsu = cv::threshold(subcv, maskcv, 0, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);

                // Convert back to Eigen
                Eigen::MatrixXd mask;
                cv::cv2eigen(maskcv, mask);
                mask = mask / 255;

                // A very low threshold means there's no significant bright
                // spots, i.e. no mouse => set mask to zeros
                if (thresh_otsu < _threshold)
                  mask *= 0;

                // Build Frame object...6
                Frame frame = window.frames()[i];
                frame.normalizedDisparityMap.zMap() *= mask;
                frame.rawDisparityMap.zMap() *= mask;
                frame.referencePicture *= mask;

                //... and add it to the output frame window
                frames[i] = frame;
      }
    return FrameWindow(frames);
  }
  

  const FrameWindow* BackgroundSubtraction::cage_frame() const {
    return _cage_frame;
  }

  void BackgroundSubtraction::cage_frame(FrameWindow* cage_frame) {
    _cage_frame = cage_frame;
  }

  const double BackgroundSubtraction::threshold() const {
    return _threshold;
  }
  void BackgroundSubtraction::threshold(double threshold) {
    _threshold = threshold;
  }


} // namespace MouseTrack
