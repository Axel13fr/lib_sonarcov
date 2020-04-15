#include "lib_sonarcov/SwathRecord.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

using ::testing::_;
using namespace scov;

TEST(SwathRecord, CanComputeOuterPoints)
{
    SwathRecord r{50., 100., 90., 35., 33., 16};
    auto p = r.outerPoint(BoatSide::Port);
    EXPECT_EQ(p.x(), 50);
    EXPECT_EQ(p.y(), 100 + 33);

    SwathRecord r2{50., 100., 0, 65., 48., 33};
    p = r2.outerPoint(BoatSide::Stbd);
    EXPECT_EQ(p.x(), 50+65);
    EXPECT_EQ(p.y(), 100);
}

int main(int argc, char** argv)
{
    // The following line must be executed to initialize Google Mock
    // (and Google Test) before running the tests.
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

