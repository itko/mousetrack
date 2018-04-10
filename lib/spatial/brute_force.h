/// \file
/// Maintainer: Felice Serena
///

#pragma once

#include "spatial_oracle.h"
#include <limits>
#include <Eigen/Core>

namespace MouseTrack {

/// Reference implementation,
/// performs queries in a naive, inefficient way
/// Expects points to be Eigen col vectors of size Dim x 1
/// The point list is a Dim x #Points eigen matrix

template <int _Dim, typename _Precision>
class BruteForce : public SpatialOracle<Eigen::Matrix<_Precision, _Dim, Eigen::Dynamic, Eigen::RowMajor + Eigen::AutoAlign>, Eigen::Matrix<_Precision, _Dim, 1>, _Precision> {
public:
    typedef Eigen::Matrix<_Precision, _Dim, Eigen::Dynamic, Eigen::RowMajor + Eigen::AutoAlign> PointList;
    typedef Eigen::Matrix<_Precision, _Dim, 1> Point;
private:
    PointList _points;
public:
    BruteForce(){
        // empty
    }

    virtual void compute(const PointList& points) {
        _points = points;
    }

    virtual void compute(PointList&& points) {
        _points = std::move(points);
    }

    virtual PointIndex find_closest(const Point& p) const  {
        PointIndex nearestP;
        (_points.colwise() - p).colwise().squaredNorm().minCoeff(&nearestP);
        return nearestP;
    }

    virtual std::vector<PointIndex> find_in_range(const Point& p, const Precision r) const {
        std::vector<PointIndex> in_range;
        Precision r2 = r*r;
        auto dists = (_points.colwise() - p).colwise().squaredNorm();
        for(int i = 0; i < dists.cols(); i += 1){
            double d = dists(0,i);
            if(d < r2){
                in_range.push_back(i);
            }
        }
        return in_range;
    }
};

// some convenient typedefs

typedef BruteForce<1, double> BruteForce1d;
typedef BruteForce<2, double> BruteForce2d;
typedef BruteForce<3, double> BruteForce3d;
typedef BruteForce<4, double> BruteForce4d;
typedef BruteForce<5, double> BruteForce5d;

typedef BruteForce<1, float> BruteForce1f;
typedef BruteForce<2, float> BruteForce2f;
typedef BruteForce<3, float> BruteForce3f;
typedef BruteForce<4, float> BruteForce4f;
typedef BruteForce<5, float> BruteForce5f;

} // MouseTrack
