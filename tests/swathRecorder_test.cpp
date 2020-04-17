#include "lib_sonarcov/SwathRecorder.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include <ros/console.h>
#include <log4cxx/logger.h>

using ::testing::_;
using namespace scov;

TEST(SwathRecorder, CanAddRecord)
{
  SwathRecorder rec;
  rec.SetOutputSide(BoatSide::Port);
  EXPECT_TRUE(rec.SwathOuterPts(BoatSide::Port).empty());

  SwathRecord r{50., 100., 90., 35., 33., 16};
  EXPECT_TRUE(rec.AddRecord(r));
  // Reject same record
  EXPECT_FALSE(rec.AddRecord(r));
  // First record is not directly added, only a start ref point
  EXPECT_TRUE(rec.SwathOuterPts(BoatSide::Port).empty());

  // Duplicates are ignored
  rec.AddRecord(r);
  EXPECT_TRUE(rec.SwathOuterPts(BoatSide::Port).empty());

  // Expect 10meters of interval to add record
  SwathRecord r2{54., 104., 90., 35., 33., 16};
  rec.AddRecord(r2);
  EXPECT_TRUE(rec.SwathOuterPts(BoatSide::Port).empty());

  // Now we have a new record far enough to be added
  SwathRecord r3{60., 110., 90., 35., 33., 16};
  rec.AddRecord(r3);
  EXPECT_FALSE(rec.SwathOuterPts(BoatSide::Port).empty());

  // Add a record not that far but with a different angle (turn)
  SwathRecord r4{65., 112., 111., 35., 33., 16};
  rec.AddRecord(r4);
  EXPECT_EQ(rec.recordsNumber(), 2);
}

TEST(SwathRecorder, CanReset)
{
  SwathRecorder rec;
  rec.SetOutputSide(BoatSide::Port);
  EXPECT_TRUE(rec.SwathOuterPts(BoatSide::Port).empty());

  SwathRecord r{50., 100., 90., 35., 33., 16};
  EXPECT_TRUE(rec.AddRecord(r));

  // Now we have a new record far enough to be added
  SwathRecord r3{60., 110., 90., 35., 33., 16};
  rec.AddRecord(r3);
  EXPECT_FALSE(rec.SwathOuterPts(BoatSide::Port).empty());

  rec.ResetLine();
  EXPECT_TRUE(rec.SwathOuterPts(BoatSide::Port).empty());
}

TEST(SwathRecorder, CanSaveLast)
{
  SwathRecorder rec;
  rec.SetOutputSide(BoatSide::Port);
  EXPECT_TRUE(rec.SwathOuterPts(BoatSide::Port).empty());

  SwathRecord r{50., 100., 90., 35., 33., 16};
  EXPECT_TRUE(rec.AddRecord(r));

  // Now we have a new record far enough to be added
  SwathRecord r3{60., 110., 90., 35., 33., 16};
  rec.AddRecord(r3);
  EXPECT_EQ(rec.recordsNumber(), 1);

  SwathRecord r4{61., 111., 90., 35., 33., 16};
  rec.AddRecord(r4);
  // Too close to be recorded (in the interval)
  EXPECT_EQ(rec.recordsNumber(), 1);

  rec.SaveLast();
  // Will be saved even in the interval
  EXPECT_EQ(rec.recordsNumber(), 2);
}

TEST(SwathRecorder, Getters)
{
  SwathRecorder rec;
  rec.SetOutputSide(BoatSide::Port);
  EXPECT_TRUE(rec.SwathOuterPts(BoatSide::Port).empty());

  SwathRecord r{50., 100., 90., 35., 33., 16};
  EXPECT_TRUE(rec.AddRecord(r));
  SwathRecord r3{60., 110., 90., 32., 31., 16};
  rec.AddRecord(r3);
  SwathRecord r4{80., 130., 90., 38., 39., 16};
  rec.AddRecord(r4);
  EXPECT_EQ(rec.recordsNumber(), 3);

  // Port point, absolute pos, from record r3
  EXPECT_EQ(rec.SwathOuterPts(BoatSide::Port)[0].y(), 133);
  // Stbd point, absolute pos, from record r4
  EXPECT_EQ(rec.LastOuterPoints().stbd_pt.y(), 92);

  EXPECT_EQ(rec.SwathWidth(BoatSide::Port,2),39);
  EXPECT_EQ(rec.SwathLocation(2).x(),80);
}

int main(int argc, char **argv)
{
  // ROS logs in unit tests
  ROSCONSOLE_AUTOINIT;
  log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
  my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);
  // The following line must be executed to initialize Google Mock
  // (and Google Test) before running the tests.

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
