/// \file
/// Maintainer: Felice Serena
///
///

#include "knn.h"

#include "spatial/brute_force.h"
#include "spatial/flann.h"

namespace MouseTrack {

void KnnClassifier::fit(const Mat &X_train, const Vec &y_train) {
  _X_train = X_train;
  _y_train = y_train;
  auto min = _X_train.colwise().minCoeff();
  auto max = _X_train.colwise().maxCoeff();
  Eigen::VectorXd size = max - min;
  double maxR = size.norm();
  // heuristic
  double cellWidth = size.minCoeff() / 5;
  int dims = X_train.cols();
  // don't use Uniform grid, it uses memory proportional to 2^dimensions
  //_oracle = std::make_unique<UniformGrid<double, -1>>(maxR, cellWidth, dims);
  //_oracle = std::make_unique<BruteForce<double, -1>>();
  _oracle = std::make_unique<Flann<double, -1>>();
  _oracle->compute(_X_train);
  _highestLabel = _y_train.maxCoeff();
}

Eigen::MatrixXd KnnClassifier::predict(const Mat &X_test) const {
  Eigen::MatrixXd result;
  result.setZero(_highestLabel + 1, X_test.cols());
  for (int i = 0; i < X_test.cols(); ++i) {
    auto closest = _oracle->find_closest(X_test.col(i), _k);
    for (auto c : closest) {
      int l = _y_train[c];
      result(l, i) += 1;
    }
    result.col(i) /= (result.col(i).sum() + 0.000001);
  }
  return result;
}

int KnnClassifier::k() const { return _k; }

void KnnClassifier::k(int newK) {
  assert(newK > 0);
  _k = newK;
}

} // namespace MouseTrack
