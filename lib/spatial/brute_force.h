/// \file
/// Maintainer: Felice Serena
///

#pragma once

#include "spatial_oracle.h"
#include <Eigen/Core>
#include <limits>
#include <queue>

namespace MouseTrack {

/// Reference implementation,
/// performs queries in a naive, inefficient way
/// Expects points to be Eigen col vectors of size Dim x 1
/// The point list is a Dim x #Points eigen matrix

template <typename _Precision, int _Dim>
class BruteForce
    : public SpatialOracle<Eigen::Matrix<_Precision, _Dim, Eigen::Dynamic,
                                         Eigen::ColMajor + Eigen::AutoAlign>,
                           _Precision> {
public:
  typedef Eigen::Matrix<_Precision, _Dim, Eigen::Dynamic,
                        Eigen::ColMajor + Eigen::AutoAlign>
      PointList;

private:
  const PointList *_points = nullptr;

  template <typename Point>
  std::vector<PointIndex> find_closest_for_point(const Point &p,
                                                 unsigned int k) const {
    typedef std::pair<Precision, PointIndex> P;
    std::priority_queue<P> candidates;
    auto dists = (_points->colwise() - p).colwise().squaredNorm();
    for (int i = 0; i < dists.cols(); i += 1) {
      Precision d = dists(0, i);
      candidates.push(P(d, i));
      if (candidates.size() > k) {
        candidates.pop();
      }
    }
    std::vector<PointIndex> result;
    while (candidates.size() > 0) {
      result.push_back(candidates.top().second);
      candidates.pop();
    }
    return result;
  }

  template <typename Point>
  std::vector<PointIndex> find_in_range_for_point(const Point &p,
                                                  const Precision r) const {
    std::vector<PointIndex> in_range;
    Precision r2 = r * r;
    auto diffs = _points->colwise() - p;
    auto dists = diffs.colwise().squaredNorm();
    for (int i = 0; i < dists.cols(); i += 1) {
      Precision d = dists(0, i);
      if (d < r2) {
        in_range.push_back(i);
      }
    }
    return in_range;
  }

public:
  BruteForce() {
    // empty
  }

  void compute(const PointList &points) { _points = &points; }

  virtual std::vector<std::vector<PointIndex>>
  find_closest(const PointList &ps, unsigned int k) const {
    assert(_points != nullptr);
    assert(k >= 1);
    std::vector<std::vector<PointIndex>> result;
    result.reserve(ps.size());
    for (int c = 0; c < ps.cols(); ++c) {
      result.push_back(find_closest_for_point(ps.col(c), k));
    }
    return result;
  }

  virtual std::vector<std::vector<PointIndex>>
  find_in_range(const PointList &ps, const Precision r) const {
    assert(_points != nullptr);
    std::vector<std::vector<PointIndex>> in_range;
    in_range.reserve(ps.cols());
    for (int c = 0; c < ps.cols(); ++c) {
      in_range.push_back(find_in_range_for_point(ps.col(c).eval(), r));
    }
    return in_range;
  }
};

// some convenient typedefs

typedef BruteForce<double, -1> BruteForceXd;
typedef BruteForce<double, 1> BruteForce1d;
typedef BruteForce<double, 2> BruteForce2d;
typedef BruteForce<double, 3> BruteForce3d;
typedef BruteForce<double, 4> BruteForce4d;
typedef BruteForce<double, 5> BruteForce5d;

typedef BruteForce<float, -1> BruteForceXf;
typedef BruteForce<float, 1> BruteForce1f;
typedef BruteForce<float, 2> BruteForce2f;
typedef BruteForce<float, 3> BruteForce3f;
typedef BruteForce<float, 4> BruteForce4f;
typedef BruteForce<float, 5> BruteForce5f;

} // namespace MouseTrack
