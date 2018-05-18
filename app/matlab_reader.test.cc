/// \file
/// Maintainer: Felice Serena
///
///

#include "matlab_reader.h"

#include <boost/test/unit_test.hpp>
//#include <boost/test/floating_point_comparison.hpp>

BOOST_AUTO_TEST_CASE(matlab_reader_average_success_case_file_methods) {
  // Note: make sure to run ./app/appTests to pass this test (from the build
  // directory which in turn is located in the project root)
  const auto reader =
      MouseTrack::MatlabReader("../app/matlab_reader.test/mock_data");

  BOOST_CHECK(reader.valid());

  BOOST_CHECK_EQUAL(1, reader.beginStream());
  BOOST_CHECK_EQUAL(5, reader.endStream());

  BOOST_CHECK_EQUAL(17, reader.beginFrame());
  BOOST_CHECK_EQUAL(19, reader.endFrame());

  BOOST_CHECK(reader.ignoredPaths().empty());
  BOOST_CHECK(reader.ignoredChannels().empty());

  for (int s = reader.beginStream(); s < reader.endStream(); s += 1) {
    for (int f = reader.beginFrame(); f < reader.endFrame(); f += 1) {
      auto normMap = reader.normalizedDisparityMap(s, f);
      BOOST_CHECK(normMap.size() == 1);

      auto rawMap = reader.rawDisparityMap(s, f);
      BOOST_CHECK(rawMap.size() == 1);

      auto pic = reader.picture(s, f);
      BOOST_CHECK(pic.size() == 1);
    }
  }

  for (int f = reader.beginFrame(); f < reader.endFrame(); f += 1) {
    auto channelParam = reader.channelParameters(f);
    BOOST_CHECK_EQUAL(4, channelParam.cols()); // each parameter
    BOOST_CHECK_EQUAL(4, channelParam.rows()); // for each stream
  }

  auto rots = reader.rotationCorrections();
  BOOST_CHECK_EQUAL(4, rots.size());

  auto chain = reader.camchain();
  BOOST_CHECK_EQUAL(8, chain.size());
}

BOOST_AUTO_TEST_CASE(matlab_reader_average_success_case_frames) {
  // Note: make sure to run ./app/appTests to pass this test (from the build
  // directory which in turn is located in the project root)
  const auto reader =
      MouseTrack::MatlabReader("../app/matlab_reader.test/mock_data");

  BOOST_CHECK(reader.valid());

  BOOST_CHECK_EQUAL(1, reader.beginStream());
  BOOST_CHECK_EQUAL(5, reader.endStream());

  BOOST_CHECK_EQUAL(17, reader.beginFrame());
  BOOST_CHECK_EQUAL(19, reader.endFrame());

  BOOST_CHECK(reader.ignoredPaths().empty());
  BOOST_CHECK(reader.ignoredChannels().empty());

  for (int f = reader.beginFrame() + 1; f < reader.endFrame(); f += 1) {
    const auto window = reader.frameWindow(f);
    const auto &frames = window.frames();
    for (int s = 0; s < reader.endStream() - reader.beginStream(); s += 1) {
      auto &frame = frames[s];

      auto normMap = frame.normalizedDisparityMap;
      BOOST_CHECK(normMap.size() == 1);

      auto rawMap = frame.rawDisparityMap;
      BOOST_CHECK(rawMap.size() == 1);

      auto pic = frame.referencePicture;
      BOOST_CHECK(pic.size() == 1);
    }

    // test some values

    // Frame parameters
    BOOST_CHECK_CLOSE_FRACTION(frames[0].focallength, 757, 0.1);
    BOOST_CHECK_CLOSE_FRACTION(frames[0].baseline, 0.023138, 0.000001);
    BOOST_CHECK_CLOSE_FRACTION(frames[0].ccx, 406.38, 0.01);
    BOOST_CHECK_CLOSE_FRACTION(frames[0].ccy, 237.18, 0.01);

    BOOST_CHECK_CLOSE_FRACTION(frames[1].focallength, 745, 0.1);

    BOOST_CHECK_CLOSE_FRACTION(frames[3].baseline, 0.023182, 0.000001);
    BOOST_CHECK_CLOSE_FRACTION(frames[3].ccx, 354.79, 0.01);
    BOOST_CHECK_CLOSE_FRACTION(frames[3].ccy, 234.41, 0.01);

    // constant parameters
    BOOST_CHECK_CLOSE_FRACTION(frames[0].rotationCorrection(1, 0), 0.00047705,
                               0.00000001);
    BOOST_CHECK_CLOSE_FRACTION(frames[3].rotationCorrection(3, 3), 1.0, 0.001);

    BOOST_CHECK_CLOSE_FRACTION(frames[0].camChainPicture(0, 0), 1, 0.000001);
    BOOST_CHECK_CLOSE_FRACTION(frames[3].camChainDisparity(0, 0), 0.99996,
                               0.00001);
  }
}
