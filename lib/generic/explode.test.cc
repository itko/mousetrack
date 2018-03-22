/// \file
/// Maintainer: Felice Serena
///
///

#include "explode.h"

#include <boost/test/unit_test.hpp>


BOOST_AUTO_TEST_CASE( explode_common ) {
    std::string in{"a,bc,def,hello world!,you are great!"};
    std::vector<std::string> expected{"a", "bc", "def", "hello world!", "you are great!"};

    auto result = MouseTrack::explode(in, ',');

    BOOST_CHECK_MESSAGE( expected.size() == result.size(), "Result does not have correct number of elements." );

    for(size_t i = 0; i < expected.size(); i += 1){
        BOOST_CHECK_MESSAGE( expected[i] == result[i] , "Element mismatch." );
    }
}




BOOST_AUTO_TEST_CASE( explode_empy_item ) {
    std::string in{"a,,,,you are great!"};
    std::vector<std::string> expected{"a", "", "", "", "you are great!"};

    auto result = MouseTrack::explode(in, ',');

    BOOST_CHECK_MESSAGE( expected.size() == result.size(), "Result does not have correct number of elements." );

    for(size_t i = 0; i < expected.size(); i += 1){
        BOOST_CHECK_MESSAGE( expected[i] == result[i] , "Element mismatch." );
    }
}
