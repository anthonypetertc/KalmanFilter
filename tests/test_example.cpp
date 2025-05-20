#define BOOST_TEST_MODULE ExampleTest
#include <boost/test/included/unit_test.hpp>

#include "kalman_filter/example.h"

BOOST_AUTO_TEST_CASE(AddFunction) {
  BOOST_CHECK_EQUAL(add(1, 2), 3);
  BOOST_CHECK_EQUAL(add(-1, 1), 0);
}
