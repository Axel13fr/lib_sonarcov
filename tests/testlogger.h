#pragma once

#include <string>
#include <fstream>
#include <list>
#include "gtest/gtest.h"
#include "lib_sonarcov/SwathRecord.h"
#include "lib_sonarcov/PathPlan.h"


using namespace scov;

/**
 * @brief The TestLogger class generates CSV files from path planner tests to later display that
 * in python
 */
class TestLogger
{
public:
    TestLogger()
        : m_fileStream(get_output_file_name(), m_fileStream.out)
    {
        //tracker.registerKalmanUpdateCb([&](KalmanFilter &kalman, bool new_track) { this->logCb(kalman, new_track); });
    }

    void registerPathPlanLogs(PathPlan &planner)
    {
      planner.registerPathCb(
          [&](const std::string &name, const std::list<Eigen::Vector2d> &path) { this->logPath(name, path); });
    }

    static constexpr auto OUTPUT_PATH = "tests/PathPlannerOutput";
    static std::string get_output_file_name()
    {
        // Use the current test name to generate a file name for dbg files which contain Kalman outputs
        // These .dbg files can then be read by a python script to generate graphs
        return std::string(OUTPUT_PATH) + "/" + ::testing::UnitTest::GetInstance()->current_test_info()->name() + ".dbg";
    }

    void logSwathRecord(const SwathRecord &r)
    {
      auto stbd = r.outerPoint(BoatSide::Stbd);
      auto port = r.outerPoint(BoatSide::Port);
      r.outerPoints();
      m_fileStream << "SWATH," << r.loc_x << ','
                   << r.loc_y  << ','
                   << port.x() << ','
                   << port.y() << ','
                   << stbd.x() << ','
                   << stbd.y()
                   << std::endl;
    }

    void logPath(const std::string& name,const std::list<Eigen::Vector2d>& path){
      for(const auto& pt : path){
        m_fileStream << name << ','
                     << pt.x()  << ','
                     << pt.y()
                     << std::endl;
      }
    }

    void logSurveyRegion(const BPolygon& opreg){
      for (const auto& pt: opreg.outer()) {
        m_fileStream << "SURVEY_REGION"<< ','
                     << pt.x()  << ','
                     << pt.y()
                     << std::endl;
      }
    }

    void logNextLine(const std::list<EPoint> line)
    {
      for (auto &p : line)
      {
        m_fileStream << "NEXT_PATH," << p.x() << ',' << p.y() << std::endl;
      }
    }

private:
    std::fstream m_fileStream;
};
