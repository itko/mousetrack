/// \file
/// Maintainer: Felice Serena
///
///

#include "brute_force.h"
#include <Eigen/Core>
#include <set>

#include <boost/test/unit_test.hpp>

namespace utf = boost::unit_test;

typedef std::vector<int> CellCoordinate;

using namespace MouseTrack;
using namespace Eigen;

BOOST_AUTO_TEST_CASE( brute_force_4d ) {
    constexpr int Dim = 4;
    typedef BruteForce4d BF;
    BF::PointList all(Dim, 5);
    all.col(0) = Vector4d(0.0,0.0,0.0,0.0);
    all.col(1) = Vector4d(2.0,2.0,2.0,2.0);
    all.col(2) = Vector4d(10.0,0.0,0.0,0.0);
    all.col(3) = Vector4d(0.0,-1.0,-1.0,-1.0);
    all.col(4) = Vector4d(10.0,1.0,1.0,1.0);

    std::set<PointIndex> expected;
    expected.insert(2);
    expected.insert(4);
    std::unique_ptr<SpatialOracle<BF::PointList, BF::Point, double>> oracle(new BF());
    oracle->compute(all);

    auto result = oracle->find_in_range(Vector4d(9.5,0,0,0), 2);

    std::set<PointIndex> received(result.begin(), result.end());

    BOOST_CHECK_MESSAGE(expected.size() == received.size(), "Expected and received sets have different cardinalities.");
    BOOST_CHECK_MESSAGE(expected == received, "Expected and received set do not contain same elements." );
}


BOOST_AUTO_TEST_CASE( brute_force_Xd ) {
    constexpr int Dim = 4;
    typedef BruteForceXd BF;
    BF::PointList all(Dim, 5);
    all.col(0) = Vector4d(0.0,0.0,0.0,0.0);
    all.col(1) = Vector4d(2.0,2.0,2.0,2.0);
    all.col(2) = Vector4d(10.0,0.0,0.0,0.0);
    all.col(3) = Vector4d(0.0,-1.0,-1.0,-1.0);
    all.col(4) = Vector4d(10.0,1.0,1.0,1.0);

    std::set<PointIndex> expected;
    expected.insert(2);
    expected.insert(4);
    std::unique_ptr<SpatialOracle<BF::PointList, BF::Point, double>> oracle(new BF());
    oracle->compute(all);

    auto result = oracle->find_in_range(Vector4d(9.5,0,0,0), 2);

    std::set<PointIndex> received(result.begin(), result.end());

    BOOST_CHECK_MESSAGE(expected.size() == received.size(), "Expected and received sets have different cardinalities.");
    BOOST_CHECK_MESSAGE(expected == received, "Expected and received set do not contain same elements." );
}

