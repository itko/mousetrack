/// \file
/// Maintainer: Felice Serena
///

#pragma once

#include "spatial_oracle.h"
#include <Eigen/Core>
#include <boost/log/trivial.hpp>
#include <flann/flann.hpp>

namespace MouseTrack {

/// Class to wrap Flann.
template <typename _Precision, int _Dim>
class Flann
    : public SpatialOracle<Eigen::Matrix<_Precision, _Dim, Eigen::Dynamic,
                                         Eigen::ColMajor + Eigen::AutoAlign>,
                           _Precision> {
public:
  // columns are points
  typedef Eigen::Matrix<_Precision, _Dim, Eigen::Dynamic,
                        Eigen::ColMajor + Eigen::AutoAlign>
      PointList;
  typedef _Precision Precision;

private:
  // make sure to store one point per row for flann
  const PointList *_points = nullptr;
  std::unique_ptr<flann::Index<flann::L2<Precision>>> _index;
  flann::Matrix<Precision> dataset;

  const flann::Matrix<Precision> flannFrom(const PointList &ps) const {
    // flann matrices expect row major data, where rows are samples and cols
    // dimensions, we are responsible for the lifetime
    const flann::Matrix<Precision> converted =
        flann::Matrix<Precision>((Precision *)ps.data(), ps.cols(), ps.rows());
    return converted;
  }

public:
  virtual void compute(const PointList &srcData) {
    _points = &srcData;
    dataset = flannFrom(srcData);
    // choose exact implementation: this is a list of tried values
    flann::IndexParams params = flann::KDTreeIndexParams(); // inaccurate
    params = flann::LinearIndexParams();                    // slow
    params = flann::KDTreeSingleIndexParams(); // for low dimensional data
    params = flann::AutotunedIndexParams();    // auto tuning: target precision,
                                            // build weight,... cool but crashes
    params = flann::KMeansIndexParams(); // fast

    _index =
        std::make_unique<flann::Index<flann::L2<Precision>>>(dataset, params);
    _index->buildIndex();
  }

  virtual std::vector<std::vector<PointIndex>>
  find_closest(const PointList &ps, unsigned int k) const {
    assert(k >= 1);
    // d: dimensions
    // k: desired nearest neighbors

    std::vector<std::vector<PointIndex>> indices;
    std::vector<std::vector<Precision>> dists;
    // query
    const flann::Matrix<Precision> query = flannFrom(ps);

    // querying
    auto params = flann::SearchParams(128);
    // automatically parallelize
    params.cores = 0;
    _index->knnSearch(query, indices, dists, k, params);

    return std::move(indices);
  }

  virtual std::vector<std::vector<PointIndex>>
  find_in_range(const PointList &ps, const Precision r) const {
    // d: dimensions
    std::vector<std::vector<PointIndex>> indices;
    std::vector<std::vector<Precision>> dists;

    // query
    const flann::Matrix<Precision> query = flannFrom(ps);

    // querying
    auto params = flann::SearchParams(128);
    // automatically parallelize
    params.cores = 0;
    _index->radiusSearch(query, indices, dists, r, params);

    return std::move(indices);
  }
};

typedef Flann<double, -1> FlannXd;
typedef Flann<double, 1> Flann1d;
typedef Flann<double, 2> Flann2d;
typedef Flann<double, 3> Flann3d;
typedef Flann<double, 4> Flann4d;
typedef Flann<double, 5> Flann5d;

typedef Flann<float, -1> FlannXf;
typedef Flann<float, 1> Flann1f;
typedef Flann<float, 2> Flann2f;
typedef Flann<float, 3> Flann3f;
typedef Flann<float, 4> Flann4f;
typedef Flann<float, 5> Flann5f;

} // namespace MouseTrack
