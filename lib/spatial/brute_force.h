/// \file
/// Maintainer: Felice Serena
///

#pragma once

#include "spatial_oracle.h"
#include <limits>

namespace MouseTrack {
/*
Precision squaredNorm(const Point& p1, const Point& p2) {
    Precision agg = 0;
    for(int i = 0; i < Dim; i += 1){
        Precision tmp = p1[i] * p2[i];
        agg += tmp*tmp;
    }
    return agg;
}
Precision norm(const Point& p1, const Point& p2) {
    return std::sqrt(squaredNorm(p1, p2));
}
*/

/// Reference implementation,
/// performs queries in a naive, inefficient way
template <typename PointList, typename Point, int Dim, typename Precision>
class BruteForce : public SpatialOracle<std::vector<Eigen::VectorXd>, Eigen::VectorXd, Dim, Precision> {
private:
    const PointList _points;
public:
    virtual void compute(const PointList& points) {
        _points.resize(points.size());
        for(int i = 0; i < points.size(); i += 1){
            _points[i] = points[i];
        }
    }

    virtual void compute(PointList&& points) {
        _points = std::move(points);
    }

    virtual PointIndex find_closest(const Point& p) const  {
        size_t nearestP;
        double nearestDist = std::numeric_limits<double>::max();
        for(PointIndex i = 0; i < _points.size(); i += 1){
            double dist = norm(p, _points[i]);
            if(dist < nearestDist){
                nearestP = i;
                nearestDist = dist;
            }
        }
        return nearestP;
    }

    virtual std::vector<PointIndex> find_in_range(const Point& p, const Precision h) const {
        std::vector<PointIndex> in_range;
        Precision r2 = r*r;
        for(PointIndex i = 0; i < _points.size(); i += 1){
            Precision dist2 = squaredNorm(p, _points[i]);
            if(dist2 <= r2){
                in_range.push_back(i);
            }
        }
        auto dists = (Ps.rowwise() - p).rowwise().squaredNorm();
        for(size_t i = 0; i < dists.size(); i += 1){
            if(dists(i) < h2){
                in_range.push_back(i);
            }
        }
        return in_range;
    }
};

} // MouseTrack
