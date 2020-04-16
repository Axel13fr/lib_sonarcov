#include "lib_sonarcov/SwathRecorder.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

using ::testing::_;
using namespace scov;

TEST(SwathRecorder, CanAddRecord)
{
  SwathRecorder rec;
  EXPECT_TRUE(rec.SwathOuterPts(BoatSide::Stbd).empty());

  SwathRecord r{50., 100., 90., 35., 33., 16};
  rec.AddRecord(r);
  EXPECT_FALSE(rec.SwathOuterPts(BoatSide::Port).empty());
}

int main(int argc, char** argv)
{
    // The following line must be executed to initialize Google Mock
    // (and Google Test) before running the tests.
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

