/// \file
/// Maintainer: Felice Serena
///
///

#pragma once

#include "classifier.h"
#include "spatial/spatial_oracle.h"

#include <memory>

namespace MouseTrack {

class KnnClassifier : public Classifier {
public:
  typedef SpatialOracle<Mat, Eigen::VectorXd, double> Oracle;

  virtual void fit(const Mat &X_train, const Vec &y_train);
  virtual Eigen::MatrixXd predict(const Mat &X_test) const;

  int k() const;
  void k(int newK);

private:
  std::unique_ptr<Oracle> _oracle;
  Mat _X_train;
  Vec _y_train;

  // number of nearest neighbors to consider
  int _k = 3;
  int _highestLabel;
};

} // namespace MouseTrack
