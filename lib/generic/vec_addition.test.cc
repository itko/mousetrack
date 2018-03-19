#include "vec_addition.h"
#define BOOST_TEST_MODULE VecAddTest
#include <boost/test/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>

namespace utf = boost::unit_test;


BOOST_AUTO_TEST_CASE( vector_add_equal_length ) {
	std::vector<double> a, b, sum;
	a.push_back(0.2);
	b.push_back(0.3);
    sum = MouseTrack::add(a, b);

	BOOST_CHECK_MESSAGE( 1 == sum.size(), "Output size should be max(a.size(), b.size())." );
	BOOST_CHECK_CLOSE( 0.5, sum[0], .00001);
}


