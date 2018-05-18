/// \file
/// Maintainer: Felice Serena
///
///

#include "read_png.h"

#include <boost/test/unit_test.hpp>
#include <sstream>

using namespace MouseTrack;

void check_matrix(PictureI expected, PictureI received) {
  BOOST_CHECK(expected.rows() == received.rows());
  BOOST_CHECK(expected.cols() == received.cols());

  for (int i = 0; i < expected.rows(); i += 1) {
    for (int j = 0; j < expected.cols(); j += 1) {
      std::stringstream ss;
      ss << "i: " << i << " j: " << j << " expected: " << expected(i, j)
         << " pic: " << received(i, j) << "\n";
      BOOST_CHECK_MESSAGE(expected(i, j) == received(i, j), ss.str());
    }
  }
}

BOOST_AUTO_TEST_CASE(read_png_grey1x1) {
  // Note: make sure to run ./lib/tests to pass this test (from the build
  // directory which in turn is located in the project root)
  const PictureI pic = read_png("../lib/generic/read_png.test/grey1x1.png");
  PictureI expected(1, 1);
  expected(0, 0) = 128;

  check_matrix(expected, pic);
}

BOOST_AUTO_TEST_CASE(read_png_grey2x2) {
  // Note: make sure to run ./lib/tests to pass this test (from the build
  // directory which in turn is located in the project root)
  const PictureI pic = read_png("../lib/generic/read_png.test/grey2x2.png");
  PictureI expected(2, 2);
  expected(0, 0) = 26;
  expected(1, 0) = 77;
  expected(0, 1) = 51;
  expected(1, 1) = 102;

  check_matrix(expected, pic);
}

BOOST_AUTO_TEST_CASE(read_png_grey2x3) {
  // Note: make sure to run ./lib/tests to pass this test (from the build
  // directory which in turn is located in the project root)
  const PictureI pic = read_png("../lib/generic/read_png.test/grey2x3.png");
  PictureI expected(2, 3);
  expected(0, 0) = 26;
  expected(1, 0) = 102;
  expected(0, 1) = 51;
  expected(1, 1) = 128;
  expected(0, 2) = 77;
  expected(1, 2) = 153;

  check_matrix(expected, pic);
}
