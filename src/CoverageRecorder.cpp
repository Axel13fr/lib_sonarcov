#include <lib_sonarcov/CoverageRecorder.h>
#include <grid_map_core/iterators/LineIterator.hpp>
#include <grid_map_core/iterators/PolygonIterator.hpp>
#include <ros/console.h>

using namespace scov;

CoverageRecorder::CoverageRecorder(coverageParams param, const std::string &world_frame_id)
{
  m_gridMap.setFrameId(world_frame_id);
  m_gridMap.setGeometry(grid_map::Length(param.width_m, param.height_m), param.resolution);
  m_gridMap.add(RAY_BASED_COVERAGE);
  m_gridMap.add(POINT_CLOUD_BASED_COVERAGE);

  auto p = m_gridMap.getPosition();

  ROS_INFO("Created map with size %f x %f m (%i x %i cells) located at ( %f , %f ).", m_gridMap.getLength().x(),
           m_gridMap.getLength().y(), m_gridMap.getSize()(0), m_gridMap.getSize()(1), p.x(), p.y());
}

void CoverageRecorder::addRecordToCoverage(const SwathRecord &rec)
{
  auto out = rec.outerPoints();
  // Increment the whole sonar swath over the gridmap
  // This is simplified because it supposes that all points along the swath are measured for the coverage
  // but this isn't true. For a better computation, this should be done with the point cloud instead
  auto cell_cnt = 0;
  for (grid_map::LineIterator iterator(m_gridMap, out.port_pt, out.stbd_pt); !iterator.isPastEnd(); ++iterator)
  {
    cell_cnt++;

    auto &elm = m_gridMap.at(RAY_BASED_COVERAGE, *iterator);
    if (std::isnan(elm))
    {
      elm = 1.f;
    }
    else
    {
      elm += 1.f;
    }
  }

  ROS_DEBUG_STREAM("New swath " << "(" << out.port_pt.x() << "," << out.port_pt.y() << ")"
                   << "(" << out.stbd_pt.x() << "," << out.stbd_pt.y()<< ")" << " added to " << cell_cnt << " cells.");
}

bool CoverageRecorder::addPointCloudToCoverage(sensor_msgs::PointCloudConstPtr pcld)
{
  if (pcld->header.frame_id != m_worldFrameId)
  {
    return false;
  }
  auto cell_cnt = 0;
  // Increment the cell where each point of the cloud falls
  for (const auto &pt : pcld->points)
  {
    cell_cnt++;
    auto &elm = m_gridMap.atPosition(POINT_CLOUD_BASED_COVERAGE, grid_map::Position(pt.x, pt.y));
    if (std::isnan(elm))
    {
      elm = 1.f;
    }
    else
    {
      elm += 1.f;
    }
  }

  ROS_DEBUG_STREAM("New pt Cloud covered " << cell_cnt << " cells.");
  return true;
}

CoverageResult CoverageRecorder::getCoveragePercent(const BPolygon &op_region, uint min_ping_per_cell)
{
  // Convert to gridmap polygon
  grid_map::Polygon poly;
  for (const auto &p : op_region.outer())
  {
    poly.addVertex(grid_map::Position(p.x(), p.y()));
  }

  auto ray_cells = 0;
  auto point_cloud_cells = 0;
  auto cells_in_area = 0;
  // Iterate over cells in the op region
  for (grid_map::PolygonIterator it(m_gridMap, poly); !it.isPastEnd(); ++it)
  {
    // For each cell, check if we had at least one scan
    if (m_gridMap.at(RAY_BASED_COVERAGE, *it) >= min_ping_per_cell)
    {
      ray_cells++;
    }
    if (m_gridMap.at(POINT_CLOUD_BASED_COVERAGE, *it) >= min_ping_per_cell)
    {
      point_cloud_cells++;
    }

    cells_in_area++;
  }
  ROS_DEBUG_STREAM("\nMeasured cells:" << point_cloud_cells << "Ray Cells" << ray_cells << "\nCells in area "
                                       << cells_in_area);
  return {(ray_cells * 100. / cells_in_area), (point_cloud_cells*100./cells_in_area)};
}
