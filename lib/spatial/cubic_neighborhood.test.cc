/// \file
/// Maintainer: Felice Serena
///
///

#include "cubic_neighborhood.h"

#include <boost/test/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>

namespace utf = boost::unit_test;

using namespace Eigen;
using namespace MouseTrack::SpatialImpl;

BOOST_AUTO_TEST_CASE( cubic_neighborhood_min_dists ) {
    CubicNeighborhood<3> neigh(2); // create first three layers

    BOOST_CHECK_EQUAL(neigh[0].size(), 1);
    BOOST_CHECK_EQUAL(neigh[0][0], Vector3i(0,0,0));

    BOOST_CHECK_CLOSE(neigh[0].min(), 0.0, 0.00001);
    BOOST_CHECK_CLOSE(neigh[0].max(), std::sqrt(3*0.5*0.5), 0.00001);

    BOOST_CHECK_EQUAL(neigh[1].size(), 27-1);

    BOOST_CHECK_CLOSE(neigh[1].min(), 0.5, 0.00001);
    BOOST_CHECK_CLOSE(neigh[1].max(), std::sqrt(3*1.5*1.5), 0.00001);

    BOOST_CHECK_EQUAL(neigh[2].size(), 125-27);

    BOOST_CHECK_CLOSE(neigh[2].min(), 1.5, 0.00001);
    BOOST_CHECK_CLOSE(neigh[2].max(), std::sqrt(3*2.5*2.5), 0.00001);
}


