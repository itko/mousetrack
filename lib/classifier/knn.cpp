/// \file
/// Maintainer: Felice Serena
///
///

#include "knn.h"

#include "spatial/uniform_grid.h"

namespace MouseTrack {

void KnnClassifier::fit(const Mat &X_train, const Eigen::RowVectorXi &y_train) {
  _X_train = X_train;
  _y_train = y_train;
  auto min = _X_train.colwise().minCoeff();
  auto max = _X_train.colwise().maxCoeff();
  Eigen::VectorXd size = max - min;
  double maxR = size.norm();
  // heuristic
  double cellWidth = size.minCoeff() / 5;
  int dims = X_train.cols();
  _oracle = std::make_unique<UniformGrid<double, -1>>(maxR, cellWidth, dims);
  _oracle->compute(_X_train);
}
Eigen::MatrixXd KnnClassifier::predict(const Mat &X_test) const {
  //_oracle.find_closest(X_test, _k);
  return Eigen::MatrixXd{};
}

} // namespace MouseTrack
