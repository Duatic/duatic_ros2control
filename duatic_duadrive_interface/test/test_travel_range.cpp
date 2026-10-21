/*
 * Copyright 2026 Duatic AG
 */

#include <gtest/gtest.h>

#include "duatic_duadrive_interface/travel_range.hpp"

using duatic::duadrive_interface::TravelRange;

TEST(TravelRangeTest, NegativeBeforeFirstSample)
{
  TravelRange range;
  EXPECT_LT(range.span(), 0.0);
}

TEST(TravelRangeTest, SpansSmallestToLargestSample)
{
  TravelRange range;
  range.update(10.0);
  range.update(9.6);
  range.update(10.4);
  EXPECT_NEAR(range.span(), 0.8, 1e-9);
}

TEST(TravelRangeTest, KeepsSpanWhenPositionReturns)
{
  TravelRange range;
  range.update(10.0);
  range.update(12.0);
  range.update(10.5);
  EXPECT_NEAR(range.span(), 2.0, 1e-9);
}
