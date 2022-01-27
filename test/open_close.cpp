#include <gtest/gtest.h>

#include "ICM20649.hpp"

TEST(ICM20649, OpenClose)
{
  ICM20649::Config config;
  ICM20649::Device device(config);
}
