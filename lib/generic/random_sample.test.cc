/// \file
/// Maintainer: Luzian Hug
///
///

#include "random_sample"

#include <boost/test/unit_test.hpp>

namespace utf = boost::unit_test;


BOOST_AUTO_TEST_CASE( random_sample ) {
    MouseTrack::PointCloud pc;
    pc.resize(2);
    pc[0].x() = 1.0;
    pc[0].y() = 2.0;
    pc[0].z() = 3.0;
    pc[0].intensity() = 4.0;

    pc[1].x() = 1.0;
    pc[1].y() = 2.0;
    pc[1].z() = 3.0;
    pc[1].intensity() = 4.0;

    PointCloud sampled = random_sample(pc,1);
    BOOST_CHECK_EQUAL(sampled.size(),1);

}
