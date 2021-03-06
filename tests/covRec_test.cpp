#include "lib_sonarcov/CoverageRecorder.h"
#include <boost/geometry.hpp>

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include <ros/console.h>
#include <log4cxx/logger.h>

using ::testing::_;
using namespace scov;

BPolygon getSquareOpRegion(const double square_size)
{
  BPolygon opreg;
  const auto HALF_SQUARE_SIZE = square_size / 2.;
  boost::geometry::append(opreg.outer(), BPoint(-HALF_SQUARE_SIZE, HALF_SQUARE_SIZE));
  boost::geometry::append(opreg.outer(), BPoint(HALF_SQUARE_SIZE, HALF_SQUARE_SIZE));
  boost::geometry::append(opreg.outer(), BPoint(HALF_SQUARE_SIZE, -HALF_SQUARE_SIZE));
  boost::geometry::append(opreg.outer(), BPoint(-HALF_SQUARE_SIZE, -HALF_SQUARE_SIZE));
  boost::geometry::append(opreg.outer(), BPoint(-HALF_SQUARE_SIZE, HALF_SQUARE_SIZE));
  boost::geometry::validity_failure_type failure;
  EXPECT_TRUE(boost::geometry::is_valid(opreg, failure));

  return opreg;
}

/**
 50% Coverage of the op region test case
+-------------------------------------------------------------+
|*******************************                    GRID MAP  |
|*******************************                              |
|*******************************                              |
|*******************************                              |
|*******************************                              |
|*****************+---------------------------+               |
|*****************|*************     OP region|               |
|*****************|*************              |               |
|*****************|*************              |               |
|*****************|*************              |               |
|*****************|*************              |               |
|*****************|*************              |               |
|****SCANNED* ****|*************              |               |
|*****************|*************              |               |
|*****************|*************              |               |
|*****************|*************              |               |
|*****************|*************              |               |
|*****************|*************              |               |
|*****************|*************              |               |
|*****************+---------------------------+               |
|***************** *************                              |
|***************** *************                              |
|***************** *************                              |
|***************** *************                              |
|***************** *************                              |
|***************** *************                              |
+-------------------------------------------------------------+


**/

TEST(CoverageRecorder, CanDoRayBasedCoverage)
{
  CoverageParams param{"world",100, 100, 1};
  CoverageRecorder rec(param);

  SwathRecord r{-50, 0, 90, 100, 99, 50};
  // Let's paint half of the map: from x =[-50->0] and
  // the complete Y column from -50 to 50.
  for (int i = 0; i < 50; i++)
  {
    r.loc_x += 1;
    rec.addRecordToCoverage(r);
  }

  // Let's check if the operation region is half covered
  BPolygon opreg = getSquareOpRegion(50);

  EXPECT_EQ(rec.getCoveragePercent(opreg).ray_based_percent, 50.);

  // Test clear function
  rec.clear();
  EXPECT_EQ(rec.getCoveragePercent(opreg).ray_based_percent, 0.);
}

TEST(CoverageRecorder, CanDoPointBasedCoverage)
{
  CoverageParams param{"world",50, 50, 1};
  CoverageRecorder rec(param);

  sensor_msgs::PointCloud m;

  // Let's paint one diagonal line: 20 cells covered
  for (int i = -10; i <= 10; i++)
  {
    geometry_msgs::Point32 pt;
    pt.x = i;
    pt.y = i;
    m.points.push_back(pt);
  }

  // frame id must be world !
  EXPECT_FALSE(rec.addPointCloudToCoverage(m));
  m.header.frame_id = "world";
  EXPECT_TRUE(rec.addPointCloudToCoverage(m));

  // Let's check if the operation region is half covered
  const auto SQUARE_SIZE = 20;
  BPolygon opreg = getSquareOpRegion(SQUARE_SIZE);

  auto op_regcells = SQUARE_SIZE * SQUARE_SIZE;
  EXPECT_EQ(rec.getCoveragePercent(opreg).point_based_percent, 20. * 100 / op_regcells);
}

int main(int argc, char **argv)
{
  // ROS logs in unit tests
  ROSCONSOLE_AUTOINIT;
  /* To enable DEBUG logging:
  log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
  my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);
  */

  // The following line must be executed to initialize Google Mock
  // (and Google Test) before running the tests.
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
