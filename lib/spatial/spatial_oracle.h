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
/// Needs to provide:
/// - `operator[]` to access points
/// - `size()` returns number of available points
/// - `resize(size_t n)` fills range `[0,n)` with points for valid access.
///
/// `Point` needs to provide an `operator[size_t]` accessor to it's underlying values,
/// valid for the range `[0, Dim)`
///
/// `Dim`: number of dimensions of `Point`
///
/// `Precision`: desired numerical precision (double, float)
template <typename PointList, typename Point, typename Precision>
class SpatialOracle {
public:
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


