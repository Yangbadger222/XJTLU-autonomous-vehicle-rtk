#include <gtest/gtest.h>

#include <stdexcept>

#include "um982_raw_driver/epoch_deduplicator.hpp"

TEST(EpochDeduplicator, RejectsDuplicateReceiverWeekTow)
{
  um982_raw_driver::EpochDeduplicator deduplicator;
  const um982_raw_driver::ObservationEpochKey key{
    um982_raw_driver::ObservationReceiver::Master, 0, 2427, 345678000};

  EXPECT_TRUE(deduplicator.accept(key));
  EXPECT_FALSE(deduplicator.accept(key));
  EXPECT_EQ(deduplicator.size(), 1U);
}

TEST(EpochDeduplicator, KeepsReceiversAndTimeReferencesIndependent)
{
  um982_raw_driver::EpochDeduplicator deduplicator;
  const um982_raw_driver::ObservationEpochKey master{
    um982_raw_driver::ObservationReceiver::Master, 0, 2427, 345678000};
  const um982_raw_driver::ObservationEpochKey base{
    um982_raw_driver::ObservationReceiver::Base, 0, 2427, 345678000};
  const um982_raw_driver::ObservationEpochKey bdt_master{
    um982_raw_driver::ObservationReceiver::Master, 1, 2427, 345678000};

  EXPECT_TRUE(deduplicator.accept(master));
  EXPECT_TRUE(deduplicator.accept(base));
  EXPECT_TRUE(deduplicator.accept(bdt_master));
}

TEST(EpochDeduplicator, EvictsOldestKeyAtCapacity)
{
  um982_raw_driver::EpochDeduplicator deduplicator(2);
  const um982_raw_driver::ObservationEpochKey first{
    um982_raw_driver::ObservationReceiver::Master, 0, 2427, 1};
  const um982_raw_driver::ObservationEpochKey second{
    um982_raw_driver::ObservationReceiver::Master, 0, 2427, 2};
  const um982_raw_driver::ObservationEpochKey third{
    um982_raw_driver::ObservationReceiver::Master, 0, 2427, 3};

  EXPECT_TRUE(deduplicator.accept(first));
  EXPECT_TRUE(deduplicator.accept(second));
  EXPECT_TRUE(deduplicator.accept(third));
  EXPECT_TRUE(deduplicator.accept(first));
  EXPECT_EQ(deduplicator.size(), 2U);
}

TEST(EpochDeduplicator, ResetStartsANewStreamSession)
{
  um982_raw_driver::EpochDeduplicator deduplicator;
  const um982_raw_driver::ObservationEpochKey key{
    um982_raw_driver::ObservationReceiver::Secondary, 0, 2427, 100};
  ASSERT_TRUE(deduplicator.accept(key));

  deduplicator.reset();

  EXPECT_TRUE(deduplicator.accept(key));
  EXPECT_EQ(deduplicator.size(), 1U);
}

TEST(EpochDeduplicator, RejectsZeroCapacity)
{
  EXPECT_THROW(um982_raw_driver::EpochDeduplicator(0), std::invalid_argument);
}
