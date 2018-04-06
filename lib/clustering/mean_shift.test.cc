/// \file
/// Maintainer: Luzian Hug
///
///

#include "mean_shift.h"

#include <boost/test/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <stdlib.h>
#include <random>


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

BOOST_AUTO_TEST_CASE( three_gaussian_clusters ) {
  std::default_random_engine gen;

  std::normal_distribution<double> gauss0(0.0,1.0);
  std::normal_distribution<double> gauss10(10.0,1.0);

  MouseTrack::PointCloud pc;
  pc.resize(300);

  MouseTrack::MeanShift ms = MouseTrack::MeanShift(1);

  for (int i = 0; i<300; i+=3) {
    pc[i].x() = gauss10(gen);
    pc[i].y() = gauss0(gen);
    pc[i].z() = gauss0(gen);
    pc[i].intensity() = gauss0(gen);

    pc[i+1].x() = gauss0(gen);
    pc[i+1].y() = gauss10(gen);
    pc[i+1].z() = gauss0(gen);
    pc[i+1].intensity() = gauss0(gen);

    pc[i+2].x() = gauss0(gen);
    pc[i+2].y() = gauss0(gen);
    pc[i+2].z() = gauss10(gen);
    pc[i+2].intensity() = gauss0(gen);
  }

  std::vector<MouseTrack::Cluster> clusters = ms(pc);

  BOOST_CHECK_EQUAL(clusters.size(),3);
  BOOST_CHECK_EQUAL(clusters[0].points().size(),100);
  BOOST_CHECK_EQUAL(clusters[1].points().size(),100);
  BOOST_CHECK_EQUAL(clusters[2].points().size(),100);


}
