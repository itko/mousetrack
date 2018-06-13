/// \file
/// Maintainer: Felice Serena
///
///

#include "flann.h"
#include <Eigen/Core>
#include <set>

#include <boost/test/unit_test.hpp>

namespace utf = boost::unit_test;

typedef std::vector<int> CellCoordinate;

using namespace MouseTrack;
using namespace Eigen;

BOOST_AUTO_TEST_CASE(flann_4d_two_close_points_find_in_range) {
  constexpr int Dim = 4;
  typedef Flann4d Oracle;
  Oracle::PointList all(Dim, 2);
  all.col(0) = Vector4d(0.0, 0.0, 0.0, 0.0);
  all.col(1) = Vector4d(0.1, 0.1, 0.1, 0);

  Oracle oracle;
  oracle.compute(all);

  std::multiset<PointIndex> expected0;
  expected0.insert(0);
  expected0.insert(1);

  std::multiset<PointIndex> expected1;
  expected1.insert(0);
  expected1.insert(1);

  Eigen::Matrix<double, 4, 2> query;
  query.col(0) = Vector4d(0.0, 0, 0, 0);
  query.col(1) = Vector4d(0.1, 0.2, 0.3, 0);
  auto result = oracle.find_in_range(query, 1.0);
  std::multiset<PointIndex> received0(result[0].begin(), result[0].end());
  std::multiset<PointIndex> received1(result[1].begin(), result[1].end());

  BOOST_CHECK_EQUAL(expected0.size(), received0.size());
  BOOST_CHECK_MESSAGE(
      expected0 == received0,
      "Expected and received set do not contain same elements.");

  BOOST_CHECK_EQUAL(expected1.size(), received1.size());
  BOOST_CHECK_MESSAGE(
      expected1 == received1,
      "Expected and received set do not contain same elements.");
}

BOOST_AUTO_TEST_CASE(flann_Xd_find_closest) {
  constexpr int Dim = 4;
  typedef Flann4d Oracle;
  Oracle::PointList all(Dim, 5);
  all.col(0) = Vector4d(0.0, 0.0, 0.0, 0.0);
  all.col(1) = Vector4d(2.0, 2.0, 2.0, 2.0);
  all.col(2) = Vector4d(10.0, 0.0, 0.0, 0.0);
  all.col(3) = Vector4d(0.0, -1.0, -1.0, -1.0);
  all.col(4) = Vector4d(10.0, 1.0, 1.0, 1.0);

  Oracle oracle;
  oracle.compute(all);

  std::vector<size_t> expected0;
  expected0.push_back(4);
  std::vector<size_t> expected1;
  expected1.push_back(0);

  Matrix<double, 4, 2> query;
  query.col(0) = Vector4d(10.1, 0.9, 0.9, 0.9);
  query.col(1) = Vector4d(0.1, 0.1, 0.1, 0.1);

  auto result = oracle.find_closest(query, 1);

  BOOST_CHECK_EQUAL(expected0.size(), result[0].size());
  BOOST_CHECK_EQUAL(expected0[0], result[0][0]);

  BOOST_CHECK_EQUAL(expected1.size(), result[1].size());
  BOOST_CHECK_EQUAL(expected1[0], result[1][0]);
}

BOOST_AUTO_TEST_CASE(flann_Xd_find_closestK) {
  constexpr int Dim = 4;
  typedef Flann4d Oracle;
  Oracle::PointList all(Dim, 5);
  all.col(0) = Vector4d(0.0, 0.0, 0.0, 0.0);
  all.col(1) = Vector4d(2.0, 2.0, 2.0, 2.0);
  all.col(2) = Vector4d(10.0, 0.0, 0.0, 0.0);
  all.col(3) = Vector4d(0.0, -1.0, -1.0, -1.0);
  all.col(4) = Vector4d(10.0, 1.0, 1.0, 1.0);

  std::multiset<PointIndex> expected;
  expected.insert(2);
  expected.insert(4);

  Oracle oracle;
  oracle.compute(all);

  auto result = oracle.find_closest(Vector4d(10.1, 0.9, 0.9, 0.9), 2)[0];

  std::multiset<PointIndex> received(result.begin(), result.end());

  BOOST_CHECK_MESSAGE(
      expected.size() == received.size(),
      "Expected and received sets have different cardinalities.");
  BOOST_CHECK_MESSAGE(
      expected == received,
      "Expected and received set do not contain same elements.");
}
