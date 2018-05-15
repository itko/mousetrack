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
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
      Mat;
  typedef SpatialOracle<Mat, Eigen::VectorXd, double> Oracle;

  void fit(const Mat &X_train, const Eigen::RowVectorXi &y_train);
  Eigen::MatrixXd predict(const Mat &X_test) const;

private:
  std::unique_ptr<Oracle> _oracle;
  Mat _X_train;
  Eigen::RowVectorXi _y_train;

  // number of nearest neighbors to consider
  int _k;
};

} // namespace MouseTrack
