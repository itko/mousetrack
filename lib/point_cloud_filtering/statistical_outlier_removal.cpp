/// \file
/// Maintainer: Felice Serena
///

#include "statistical_outlier_removal.h"
#include "spatial/statistical_outlier_detection.h"
#include "spatial/uniform_grid.h"
#include <boost/log/trivial.hpp>

namespace MouseTrack {

StatisticalOutlierRemoval::StatisticalOutlierRemoval(int k, double alpha)
    : _k(k), _alpha(alpha) {
  // empty
}

PointCloud StatisticalOutlierRemoval::
operator()(const PointCloud &inCloud) const {
  auto bb_min = inCloud.min();
  auto bb_max = inCloud.max();
  auto bb_size = bb_max - bb_min;

  // division arbitrary, heuristic for better choice?
  UniformGrid3d ug(bb_size.maxCoeff(), bb_size.minCoeff() / 40.0);

  typedef UniformGrid3d::PointList PointList;
  typedef UniformGrid3d::Point Point;
  PointList pts(3, inCloud.size());
  for (size_t i = 0; i < inCloud.size(); ++i) {
    auto p = inCloud[i];
    pts(0, i) = p.x();
    pts(1, i) = p.x();
    pts(2, i) = p.x();
  }
  ug.compute(pts);
  auto outliers = statisticalOutlierDetection<PointList, Point, Precision>(
      pts, &ug, k(), alpha());

  PointCloud outCloud;
  outCloud.resize(inCloud.size() - outliers.size());
  size_t next_insert = 0;
  size_t o = 0;
  for (size_t i = 0; inCloud.size(); ++i) {
    if (outliers[o] == i) {
      // skip
      ++o;
      continue;
    }
    outCloud[next_insert] = inCloud[i];
    ++next_insert;
  }
  return outCloud;
}

// setter/getter

void StatisticalOutlierRemoval::k(int _new) { _k = _new; }

int StatisticalOutlierRemoval::k() const { return _k; }

void StatisticalOutlierRemoval::alpha(double _new) { _alpha = _new; }

double StatisticalOutlierRemoval::alpha() const { return _alpha; }

} // namespace MouseTrack
