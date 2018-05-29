/// \file
/// Maintainer: Felice Serena
///
///

#include "knn.h"

#include "spatial/brute_force.h"
#include "spatial/flann.h"

namespace MouseTrack {

KnnClassifier::KnnClassifier() {
  _oracleFactory.desiredOracle(OFactory::FLANN);
}

KnnClassifier::OFactory &KnnClassifier::oracleFactory() {
  return _oracleFactory;
}

const KnnClassifier::OFactory &KnnClassifier::oracleFactory() const {
  return _oracleFactory;
}

void KnnClassifier::fit(const Mat &X_train, const Vec &y_train) {
  _X_train = X_train;
  _y_train = y_train;
  OFactory::Query query;
  query.example_data = &_X_train;
  _oracle = _oracleFactory.forQuery(query);
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
