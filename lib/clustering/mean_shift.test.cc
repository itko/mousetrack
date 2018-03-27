/// \file
/// Maintainer: Felice Serena
///
///

#include "mean_shift.h"

#include <boost/test/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>



namespace utf = boost::unit_test;


BOOST_AUTO_TEST_CASE( mean_shift_read_write ) {
    MouseTrack::PointCloud pc;
    const int pcsize = 8;
    pc.resize(pcsize);

    for (int i=0; i<pcsize; i++) {
      pc[i].x() = 1.0;
      pc[i].y() = 2.0;
      pc[i].z() = 3.0;
      pc[i].intensity() = 4.0;
    }

    const int k = 3;

    MouseTrack::MeanShift ms = MouseTrack::MeanShift(k);

    std::vector<MouseTrack::Cluster> clusters = ms(pc);

    //count all the points in clustering
    int npoints = 0;

    for(int i=0; i < clusters.size(); i++) {
      //Clusters must not be empty
      BOOST_CHECK(!clusters[i].points().empty());
      npoints += clusters[i].points().size();
    }
    BOOST_CHECK_EQUAL(npoints,pcsize);

}
