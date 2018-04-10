/// \file
/// Maintainer: Felice Serena
///

#pragma once

#include "generic/types.h"
#include <Eigen/Core>

namespace MouseTrack {

/// A spatial oracle takes a list of points via the `compute()` method,
/// proecesses and stores it internally, and allows the client
/// to perform spatial queries.
///
/// `PointList`: data type to communicate a batch of points.
///
/// `Point`: representation of points
///
/// `Precision`: desired numerical precision (double, float)
template <typename _PointList, typename _Point, typename _Precision>
class SpatialOracle {
public:
    /// Make available to instance
    typedef _PointList PointList;
    /// Make available to instance
    typedef _Point Point;
    /// Make available to instance
    typedef _Precision Precision;

    /// Set points you want to perform queries on.
    virtual void compute(const PointList& points) = 0;

    /// Set points you want to perform queries on.
    /// Move setter to consume points.
    virtual void compute(PointList&& points) = 0;

    /// Give index of nearest neighbor to `p`
    virtual PointIndex find_closest(const Point& p) const = 0;

    /// Give an index-list of all points within distance `r` around `p`
    virtual std::vector<PointIndex> find_in_range(const Point& p, const Precision r) const = 0;
};

} // MouseTrack


