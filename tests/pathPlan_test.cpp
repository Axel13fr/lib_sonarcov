#include "lib_sonarcov/SwathRecorder.h"
#include "lib_sonarcov/PathPlan.h"

#include "testlogger.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include <random>
#include <ros/console.h>
#include <log4cxx/logger.h>

using ::testing::_;
using namespace scov;

class SurveyHelper
{
 public:
  TestLogger &m_l;
  double std_noise;
  SwathRecorder &m_rec;
  std::mt19937 gen = std::mt19937(1773);
  std::normal_distribution<> d{0, std_noise};

  SurveyHelper(TestLogger &l, SwathRecorder &rec, double std_noise = 0) : m_l(l), std_noise(std_noise), m_rec(rec)
  {
  }

  void translPosAdd(double dx, double dy, SwathRecord &r)
  {

    r.loc_x += dx;
    r.loc_y += dy;
    // Add noise to swath record
    auto noisePort = d(gen);
    auto noiseStbd = d(gen);
    r.swath_port += noisePort;
    r.swath_stbd += noiseStbd;

    EXPECT_TRUE(m_rec.AddRecord(r));
    m_l.logSwathRecord(r);
  }

  void translSwath(double dswath, BoatSide s, SwathRecord &rec)
  {
    if (s == BoatSide::Port)
    {
      rec.swath_port += dswath;
    }
    else
    {
      rec.swath_stbd += dswath;
    }
  }
};

/**
 Op Area
    +----------------------------------------------------+
    |     Boat Trajectory                                |
    | +------------------------------------------------------+
    |                                                    |
    |                                                    |
    |                                                    |
    |   Sonar Swath on Starboard Side                    |
    |   +--------------------+                           |
    |                         +                          |
    |                          +----------------------+  |
    |                                                    |
    |                                                    |
    |                                                    |
    |                                                    |
    |                                                    |
    +----------------------------------------------------+
**/
TEST(PathPlan, CanGenerateSimplePlan)
{
  TestLogger l;

  const auto STBD_SWATH = 33.;
  const auto PORT_SWATH = 32.;
  const auto SPEED_MS = 2.;
  SwathRecorder rec;
  rec.SetOutputSide(BoatSide::Stbd);

  SurveyHelper h(l, rec);

  SwathRecord r{0, 0, 90., STBD_SWATH, PORT_SWATH, 16};
  // Initial pos
  EXPECT_TRUE(rec.AddRecord(r));

  // For 60meters, constant measurements
  for (uint i_secs = 1; i_secs <= 30; i_secs++)
  {
    // 3 sonar measurements per sec
    h.translPosAdd(SPEED_MS * 1 / 3., 0, r);
    h.translPosAdd(SPEED_MS * 1 / 3., 0, r);
    h.translPosAdd(SPEED_MS * 1 / 3., 0, r);
  }

  // We are at x = 60m
  EXPECT_NEAR(r.loc_x, 60., 0.001);

  // Next 20meters, swath Stbd increasing
  for (uint i_secs = 1; i_secs <= 10; i_secs++)
  {
    // 3 sonar measurements per sec
    h.translSwath(1 / 3., BoatSide::Stbd, r);
    h.translPosAdd(SPEED_MS * 1 / 3., 0, r);
    h.translSwath(1 / 3., BoatSide::Stbd, r);
    h.translPosAdd(SPEED_MS * 1 / 3., 0, r);
    h.translSwath(1 / 3., BoatSide::Stbd, r);
    h.translPosAdd(SPEED_MS * 1 / 3., 0, r);
  }

  EXPECT_NEAR(r.swath_stbd, STBD_SWATH + 10, 0.001);

  // Next 60meters, constant measurements
  for (uint i_secs = 1; i_secs <= 30; i_secs++)
  {
    // 3 sonar measurements per sec
    h.translPosAdd(SPEED_MS * 1 / 3., 0, r);
    h.translPosAdd(SPEED_MS * 1 / 3., 0, r);
    h.translPosAdd(SPEED_MS * 1 / 3., 0, r);
  }

  EXPECT_NEAR(r.loc_x, 140, 0.001);
  EXPECT_NEAR(r.swath_stbd, 43, 0.001);

  BPolygon opreg;
  boost::geometry::append(opreg.outer(), BPoint(1, 50));
  boost::geometry::append(opreg.outer(), BPoint(130, 50));
  boost::geometry::append(opreg.outer(), BPoint(130, -100));
  boost::geometry::append(opreg.outer(), BPoint(1, -100));
  boost::geometry::append(opreg.outer(), BPoint(1, 50));
  boost::geometry::validity_failure_type failure;
  EXPECT_TRUE(boost::geometry::is_valid(opreg, failure));
  l.logSurveyRegion(opreg);

  PathPlan plan(rec, BoatSide::Stbd, opreg);
  l.registerPathPlanLogs(plan);
  auto nextPath = plan.GenerateNextPath();
}


/**
 Op Area
    +----------------------------------------------------+
    |     Boat Trajectory                                |
    | +------------------------------------------------------+
    |                                                    |
    |                                                    |
    |                                                    |
    |   Sonar Swath on Starboard Side                    |
    |             with NOISE(not shown here)             |
    |   +--------------------+                           |
    |                         +                          |
    |                          +----------------------+  |
    |                                                    |
    |                                                    |
    |                                                    |
    |                                                    |
    |                                                    |
    +----------------------------------------------------+
**/
TEST(PathPlan, CanGenerateNoisyPlan)
{
  TestLogger l;

  const auto STBD_SWATH = 33.;
  const auto PORT_SWATH = 32.;
  const auto SPEED_MS = 2.;
  SwathRecorder rec;
  rec.SetOutputSide(BoatSide::Stbd);

  SurveyHelper h(l, rec, 3);

  SwathRecord r{0, 0, 90., STBD_SWATH, PORT_SWATH, 16};
  // Initial pos
  EXPECT_TRUE(rec.AddRecord(r));

  // For 60meters, constant measurements
  for (uint i_secs = 1; i_secs <= 30; i_secs++)
  {
    // 3 sonar measurements per sec
    h.translPosAdd(SPEED_MS * 1 / 3., 0, r);
    h.translPosAdd(SPEED_MS * 1 / 3., 0, r);
    h.translPosAdd(SPEED_MS * 1 / 3., 0, r);
  }

  // We are at x = 60m
  EXPECT_NEAR(r.loc_x, 60., 0.001);

  // Next 10meters, swath Stbd increasing
  for (uint i_secs = 1; i_secs <= 10; i_secs++)
  {
    // 3 sonar measurements per sec
    h.translSwath(1 / 3., BoatSide::Stbd, r);
    h.translPosAdd(SPEED_MS * 1 / 3., 0, r);
    h.translSwath(1 / 3., BoatSide::Stbd, r);
    h.translPosAdd(SPEED_MS * 1 / 3., 0, r);
    h.translSwath(1 / 3., BoatSide::Stbd, r);
    h.translPosAdd(SPEED_MS * 1 / 3., 0, r);
  }

  // Next 60meters, constant measurements
  for (uint i_secs = 1; i_secs <= 30; i_secs++)
  {
    // 3 sonar measurements per sec
    h.translPosAdd(SPEED_MS * 1 / 3., 0, r);
    h.translPosAdd(SPEED_MS * 1 / 3., 0, r);
    h.translPosAdd(SPEED_MS * 1 / 3., 0, r);
  }

  EXPECT_NEAR(r.loc_x, 140, 0.001);

  BPolygon opreg;
  boost::geometry::append(opreg.outer(), BPoint(1, 50));
  boost::geometry::append(opreg.outer(), BPoint(130, 50));
  boost::geometry::append(opreg.outer(), BPoint(130, -100));
  boost::geometry::append(opreg.outer(), BPoint(1, -100));
  boost::geometry::append(opreg.outer(), BPoint(1, 50));
  boost::geometry::validity_failure_type failure;
  EXPECT_TRUE(boost::geometry::is_valid(opreg, failure));
  l.logSurveyRegion(opreg);

  PathPlan plan(rec, BoatSide::Stbd, opreg);
  l.registerPathPlanLogs(plan);
  auto nextPath = plan.GenerateNextPath();
}

int main(int argc, char **argv)
{
  // ROS logs in unit tests
  ROSCONSOLE_AUTOINIT;
  log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
  //my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);

  // The following line must be executed to initialize Google Mock
  // (and Google Test) before running the tests.
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
