/// \file
/// Maintainer: Felice Serena
///
///

#include "matlab_reader.h"

#include <boost/test/unit_test.hpp>



BOOST_AUTO_TEST_CASE( matlab_reader_average_success_case ) {
    // Note: make sure to run ./app/appTests to pass this test (from the build directory which in turn is located in the project root)
    const auto reader = MouseTrack::MatlabReader("../app/matlab_reader.test/mock_data");

    BOOST_CHECK(reader.valid());

    BOOST_CHECK_EQUAL(1, reader.beginStream());
    BOOST_CHECK_EQUAL(5, reader.endStream());

    BOOST_CHECK_EQUAL(17, reader.beginFrame());
    BOOST_CHECK_EQUAL(19, reader.endFrame());

    BOOST_CHECK(reader.ignoredPaths().empty());
    BOOST_CHECK(reader.ignoredChannels().empty());

    for(int s = reader.beginStream()+3; s < reader.endStream(); s += 1){
        for(int f = reader.beginFrame()+1; f < reader.endFrame(); f += 1) {
            auto normMap = reader.normalizedDisparityMap(s, f);
            BOOST_CHECK(normMap.zMap().size() == 1);

            auto rawMap = reader.rawDisparityMap(s, f);
            BOOST_CHECK(rawMap.zMap().size() == 1);

            auto pic = reader.picture(s, f);
            BOOST_CHECK(pic.size() == 1);
        }
    }

    for(int f = reader.beginFrame(); f < reader.endFrame(); f += 1) {
        auto channelParam = reader.channelParameters(f);
        BOOST_CHECK_EQUAL(4, channelParam.cols()); // each parameter
        BOOST_CHECK_EQUAL(4, channelParam.rows()); // for each stream
    }

    auto rots = reader.rotationCorrections();
    BOOST_CHECK_EQUAL(4, rots.size());

    auto chain = reader.camchain();
    BOOST_CHECK_EQUAL(8, chain.size());
}



