/// \file
/// Maintainer: Felice Serena
///
///

#pragma once

#include <Eigen/Core>

namespace MouseTrack {

/// Interface for classification algorithms.
class Classifier {
public:
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
      Mat;
  /// Takes a matrix (cols are samples, rows are dimensions) representing
  /// the input values and a row vector of labels.
  /// It then learns from the data to fit the model to it.
  void fit(const Mat &X_train, const Eigen::RowVectorXi &y_train);

  /// Takes a matrix of input values (cols are samples, rows are dimensions),
  /// and returns a matrix of confidence values.
  /// The returned matrix has `X_test.cols()` columns and k rows,
  /// where `k` is the number of labels.
  /// It holds a confidence value for each (sample, label) pair where 0 means
  /// "very unlikely" and 1 "very likely".
  /// The confidence values are not normalized, i.e. they don't sum to 1 and are
  /// hence not probabilities.
  Eigen::MatrixXd predict(const Mat &X_test) const;
};

} // namespace MouseTrack
