/// \file
/// Maintainer: Felice Serena
///
///

#include "read_csv.h"

#include <boost/test/unit_test.hpp>



BOOST_AUTO_TEST_CASE( read_csv_all ) {
    // Note: make sure to run ./lib/tests to pass this test (from the build directory which in turn is located in the project root)
    const std::vector<std::vector<std::string>> content = MouseTrack::read_csv("../lib/generic/read_csv.test.example1.csv");
    std::vector<std::vector<std::string>> expected;
    expected.push_back(std::vector<std::string>{"a" ,"", "b", "c"});
    expected.push_back(std::vector<std::string>{"e" ,"h", "", "j"});
    expected.push_back(std::vector<std::string>{"f" ,"g", "d", "k"});

    BOOST_CHECK( expected.size() == content.size() );
    for(size_t i = 0; i < content.size(); i += 1) {
        BOOST_CHECK(expected[i].size() == content[i].size());
        for(size_t j = 0; j < content[i].size(); j += 1) {
            BOOST_CHECK_MESSAGE(expected[i][j] == content[i][j], "Found unexpected element.");
        }
    }
}


