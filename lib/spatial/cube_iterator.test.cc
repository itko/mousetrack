/// \file
/// Maintainer: Felice Serena
///
///

#include "cube_iterator.h"
#include <set>

#include <boost/test/unit_test.hpp>

namespace utf = boost::unit_test;

typedef std::vector<int> CellCoordinate;

using namespace MouseTrack::SpatialImpl;

BOOST_AUTO_TEST_CASE( cube_iterator_4d ) {
    /// Create all expected grid coordinates for four dimensions
    /// Let the iterator create all grid coordinates
    /// compare the two set
    const int cubeWidth = 3;
    std::multiset<CellCoordinate> expected;
    CellCoordinate tmp;
    tmp.resize(4);
    for(int a = 0; a < cubeWidth; ++a){
        tmp[0] = a;
        for(int b = 0; b < cubeWidth; ++b){
            tmp[1] = b;
            for(int c = 0; c < cubeWidth; ++c){
                tmp[2] = c;
                for(int d = 0; d < cubeWidth; ++d){
                    tmp[3] = d;
                    expected.insert(tmp);
                }
            }
        }
    }
    std::multiset<CellCoordinate> received;
    CubeIterator<4, CellCoordinate> it(cubeWidth);
    CubeIterator<4, CellCoordinate> end;
    for(; it != end; ++it){
        received.insert(*it);
    }

    BOOST_CHECK_MESSAGE(expected.size() == received.size(), "Expected and received sets have different cardinalities.");
    BOOST_CHECK_MESSAGE(expected == received, "Expected and received set do not contain same elements." );
}



BOOST_AUTO_TEST_CASE( cube_iterator_Xd ) {
    /// Create all expected grid coordinates for four dimensions
    /// Let the iterator create all grid coordinates
    /// compare the two set
    const int cubeWidth = 3;
    std::multiset<CellCoordinate> expected;
    CellCoordinate tmp;
    tmp.resize(4);
    for(int a = 0; a < cubeWidth; ++a){
        tmp[0] = a;
        for(int b = 0; b < cubeWidth; ++b){
            tmp[1] = b;
            for(int c = 0; c < cubeWidth; ++c){
                tmp[2] = c;
                for(int d = 0; d < cubeWidth; ++d){
                    tmp[3] = d;
                    expected.insert(tmp);
                }
            }
        }
    }
    std::multiset<CellCoordinate> received;
    CubeIterator<-1, CellCoordinate> it(cubeWidth, 4);
    CubeIterator<-1, CellCoordinate> end;
    for(; it != end; ++it){
        received.insert(*it);
    }

    BOOST_CHECK_MESSAGE(expected.size() == received.size(), "Expected and received sets have different cardinalities.");
    BOOST_CHECK_MESSAGE(expected == received, "Expected and received set do not contain same elements." );
}


