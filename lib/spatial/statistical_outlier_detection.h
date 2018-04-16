/// \file
/// Maintainer: Felice Serena
///

#pragma once

#include "spatial_oracle.h"

namespace MouseTrack {

namespace impl {

/// true, if at least one component of v1 is larger than v2
template <typename Vec1, typename Vec2>
bool isLarger(const Vec1 &v1, const Vec2 &v2) {
  for (int i = 0; i < v1.size(); ++i) {
    if (v1[i] > v2[i]) {
      return true;
    }
  }
  return false;
}

} // namespace impl
using namespace impl;

/// Based on: Towards 3D point cloud based object maps for household
/// environments, Bogdan Rusu, section 4.1
///
/// Idea: We characterize outliers based on their local neighborhood. For this
/// we calculate the mean and standard deviation of nearest neighbor distances.
///
/// Then we trim points outside mu +- alpha * sigma
///
/// k: number of nearest neighbors to take into account
///
/// mu: mean of k nearest neighbors
///
/// sigma: standard deviation of nearest neighbors
///
/// alpha: decides how much variance we want to allow
///
/// pts: #D x #P matrix
///
/// oracle: A spatial oracle ready for queries on pts.
template <typename PointList, typename Point, typename Precision>
std::vector<size_t> statisticalOutlierDetection(
    const PointList &pts,
    const SpatialOracle<PointList, Point, Precision> *oracle, unsigned int k,
    Precision alpha) {

  std::vector<size_t> outliers;
  for (int i = 0; i < pts.cols(); ++i) {
    auto neighbors = oracle->find_closest(pts.col(i), k);
    if (neighbors.empty()) {
      // classify lonely points as outliers?
      // should this happen?
      outliers.push_back(i);
      continue;
    }
    PointList locals(pts.rows(), neighbors.size());
    for (size_t n = 0; n < neighbors.size(); ++n) {
      // include i??
      locals.col(n) = pts.col(neighbors[n]);
    }
    Point mean = locals.rowwise().sum() / locals.cols();
    auto diffs = locals.colwise() - mean;
    auto variance =
        ((diffs.array() * diffs.array()).rowwise().sum()) / locals.cols();
    Point stddev = alpha * variance.array().sqrt();
    // p is inlier iff p in [mean - stddev, mean + stddev]
    auto centered = (pts.col(i) - mean).array().abs();
    if (isLarger(centered, stddev)) {
      // outlier
      outliers.push_back(i);
      break;
    }
  }
  return outliers;
}

} // namespace MouseTrack
