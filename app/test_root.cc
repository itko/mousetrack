#define BOOST_TEST_MAIN
#if !defined( WIN32 )
    #define BOOST_TEST_DYN_LINK
#endif
#define BOOST_TEST_MODULE MouseTrackTests
#include <boost/test/unit_test.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>

struct SetUp {
    SetUp(){
        boost::log::core::get()->set_filter
        (
            // By default, we don't care about trace
            boost::log::trivial::severity >= boost::log::trivial::debug
        );
    }
    ~SetUp(){
        // empty
    }
};

BOOST_GLOBAL_FIXTURE(SetUp);
