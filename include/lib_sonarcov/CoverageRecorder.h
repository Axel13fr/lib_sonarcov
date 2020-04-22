#pragma once

#include "lib_sonarcov/SwathRecord.h"

#include <sensor_msgs/PointCloud.h>
#include <grid_map_core/GridMap.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/geometries.hpp>

namespace scov
{

using BPoint = boost::geometry::model::d2::point_xy<double>;
using BPolygon = boost::geometry::model::polygon<BPoint>;

struct coverageParams
{
  // Gridmap side lengths in x, and y-direction of the grid map [m].
  double width_m = 10000;
  double height_m = 10000;
  // the cell size in [m/cell]
  double resolution = 1.;
};

struct CoverageResult
{
  // The full swath projection on the ground will cover the cells, even if some cells
  // will be have measurement points
  double ray_based_percent;
  // Only each point measurements falling into cells will generate coverage for the given cell
  double point_based_percent;
};


class CoverageRecorder
{
 public:
  CoverageRecorder(coverageParams param, const std::string &world_frame_id = "world");

  /**
   * @brief addRecordToCoverage
   * @param rec A swath record containing the sonar position in world coordinates
   * and swath distances from the sensor position
   */
  void addRecordToCoverage(const SwathRecord &rec);

  /**
   * @brief addPointCloudToCoverage
   * @param pcld sonar point cloud in world coordinates
   * @return false if point cloud is not in world coordinates
   */
  bool addPointCloudToCoverage(sensor_msgs::PointCloudConstPtr pcld);

  /**
   * @brief getCoveragePercent
   * @param op_region in world coordinates
   * @param min_ping_per_cell Minimum number of measurements per cell to consider it covered
   * @return the coverage of the operation region in percents [0;100]
   */
  CoverageResult getCoveragePercent(const BPolygon &op_region, uint min_ping_per_cell =0) const;

  /**
   * @brief getCoverageGrid
   * @return the grid map containing coverage information
   * The different layers can be accessed via the const defined below
   */
  const grid_map::GridMap &getCoverageGrid() const
  {
    return m_gridMap;
  }

  static constexpr auto RAY_BASED_COVERAGE = "ray_coverage";
  static constexpr auto POINT_CLOUD_BASED_COVERAGE = "point_cloud_coverage";

 private:
  grid_map::GridMap m_gridMap;
  std::string m_worldFrameId = "world";
};

}  // namespace scov
