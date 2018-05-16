/// \file
/// Maintainer: Felice Serena
///

#pragma once

#include "frame_window_filtering.h"

#include "classifier/classifier.h"

#include <memory>

namespace MouseTrack {

class HogLabeling : public FrameWindowFiltering {
public:
  typedef Classifier::Mat Mat;
  typedef Classifier::Vec Vec;
  void train(const Mat &X_train, const Vec &y_train);
  virtual FrameWindow operator()(const FrameWindow &window) const;

private:
  std::unique_ptr<Classifier> _classifier;
  // highest label + 1: we need to know, how many labels there are
  int _numLabels;
};

} // namespace MouseTrack
