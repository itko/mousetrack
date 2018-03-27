/// \file
/// Maintainer: Felice Serena
///
///

#include "mean_shift.h"

#include <boost/test/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <stdlib.h>


namespace utf = boost::unit_test;


BOOST_AUTO_TEST_CASE( mean_shift_read_write ) {
MouseTrack::PointCloud pc;
MouseTrack::MeanShift ms = MouseTrack::MeanShift(1);
ms(pc);
}

BOOST_AUTO_TEST_CASE( mean_shift_single_point ) {
  MouseTrack::MeanShift ms = MouseTrack::MeanShift(1);

  MouseTrack::PointCloud pc;
  pc.resize(1);
  pc[0].x() = 0;
  pc[0].y() = 3;
  pc[0].z() = 5;
  pc[0].intensity() = 1;

  std::vector<MouseTrack::Cluster> clusters = ms(pc);
  BOOST_CHECK_EQUAL(clusters.size(),1);
}

BOOST_AUTO_TEST_CASE( two_faraway_points ) {
  MouseTrack::MeanShift ms = MouseTrack::MeanShift(1);
  MouseTrack::PointCloud pc;
  pc.resize(2);
  pc[0].x() = 0;
  pc[0].y() = 0;
  pc[0].z() = 0;
  pc[0].intensity() = 0;
  pc[1].x() = 100;
  pc[1].y() = 100;
  pc[1].z() = 100;
  pc[1].intensity() = 1;

  std::vector<MouseTrack::Cluster> clusters = ms(pc);
  BOOST_CHECK_EQUAL(clusters.size(),2);
}

BOOST_AUTO_TEST_CASE( two_closetogether_points ) {
  MouseTrack::MeanShift ms = MouseTrack::MeanShift(1);
  MouseTrack::PointCloud pc;
  pc.resize(2);
  pc[0].x() = 0;
  pc[0].y() = 0;
  pc[0].z() = 0;
  pc[0].intensity() = 0;
  pc[1].x() = 0.1;
  pc[1].y() = 0.1;
  pc[1].z() = 0.1;
  pc[1].intensity() = 0;

  std::vector<MouseTrack::Cluster> clusters = ms(pc);
  BOOST_CHECK_EQUAL(clusters.size(),1);
}
