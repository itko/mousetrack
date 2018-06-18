/// \file
/// Maintainer: Felice Serena
///

#pragma once

#include "frame_window_filtering.h"

#include "classifier/classifier.h"

#include <memory>

namespace MouseTrack {

///
/// Feed the object hog-feature vectors and the corresponding labels via the `train` method.
///
/// The data is passed on to an internal classifier.
///
/// The operator() method, runs a sliding window over each reference frame to collect hog feature vectors.
///
/// Those feature vectors are then classified according to the training data.
/// The window labels are the passed to each pixel and stored in the `labels` member of each frame.
///
class HogLabeling : public FrameWindowFiltering {
public:
  typedef Classifier::Mat Mat;
  typedef Classifier::Vec Vec;
  void train(const Mat &X_train, const Vec &y_train);
  virtual FrameWindow operator()(const FrameWindow &window) const;

  int slidingWindowWidth() const;
  void slidingWindowWidth(int _new);

  int slidingWindowHeight() const;
  void slidingWindowHeight(int _new);

  /// Get the number of pixels between two neighboring sliding windows (used along both axis)
  int slidingWindowStride() const;

  /// Set the number of pixels between two neighboring sliding windows (used along both axis)
  void slidingWindowStride(int _new);

  std::unique_ptr<Classifier>& classifier();

private:
  std::unique_ptr<Classifier> _classifier;
  /// highest label + 1: we need to know, how many labels there are. Set by `train()`
  int _numLabels;

  /// width of the sliding window
  int _windowWidth = 64;

  /// height of the sliding window
  int _windowHeight = 64;

  /// Distance between two neighboring sliding windows
  int _stepSize = 64 / 8;

  /// number of histogram bins for HOG??
  int _nbins = 9;

  /// ?? from opencv HOG
  int _blockWidth = 16;
  int _blockHeight = 16;
  int _blockStrideWidth = 8;
  int _blockStrideHeight = 8;
  int _cellWidth = 8;
  int _cellHeight = 8;

  /// make sure the highest value is 1 and the lowest value is 0
  bool _normalizeRange = true;

  /// normalize accross labels such that all labels for one pixel sum to 1
  bool _normalizeAccross = false;
};

} // namespace MouseTrack
