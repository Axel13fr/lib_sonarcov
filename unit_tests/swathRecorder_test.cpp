#include "lib_sonarcov/SwathRecorder.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

using ::testing::_;
using namespace scov;

TEST(SwathRecorder, CanAddRecord)
{
  SwathRecorder rec;
  rec.SetOutputSide(BoatSide::Port);
  EXPECT_TRUE(rec.SwathOuterPts(BoatSide::Port).empty());

  SwathRecord r{50., 100., 90., 35., 33., 16};
  rec.AddRecord(r);
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
  EXPECT_EQ(rec.recordsNumber(),2);
}

int main(int argc, char** argv)
{
    // The following line must be executed to initialize Google Mock
    // (and Google Test) before running the tests.
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

