#include <lib_sonarcov/CoverageRecorder.h>
#include <grid_map_core/iterators/LineIterator.hpp>
#include <grid_map_core/iterators/PolygonIterator.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/console.h>

using namespace scov;

void CoverageRecorder::initGridMapFromParams()
{
  m_gridMap.setFrameId(m_params.frame_id);
  m_gridMap.setGeometry(grid_map::Length(m_params.width_m, m_params.height_m), m_params.resolution);
  m_gridMap.add(RAY_BASED_COVERAGE);
  m_gridMap.add(POINT_CLOUD_BASED_COVERAGE);
  m_gridMap.add(DEPTH_MAP);
}

CoverageRecorder::CoverageRecorder(const CoverageParams &param) : m_params(param)
{
  initGridMapFromParams();

  auto p = m_gridMap.getPosition();

  ROS_INFO("Created map with size %f x %f m (%i x %i cells) located at ( %f , %f ).", m_gridMap.getLength().x(),
           m_gridMap.getLength().y(), m_gridMap.getSize()(0), m_gridMap.getSize()(1), p.x(), p.y());
}

void CoverageRecorder::clear()
{
  m_gridMap.clearAll();
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

  ROS_DEBUG_STREAM("New swath "
                   << "(" << out.port_pt.x() << "," << out.port_pt.y() << ")"
                   << "(" << out.stbd_pt.x() << "," << out.stbd_pt.y() << ")"
                   << " added to " << cell_cnt << " cells.");
}

void CoverageRecorder::setCellDepth(const grid_map::Position &pt, const float depth)
{
  auto &elm = m_gridMap.atPosition(DEPTH_MAP, pt);
  // For now, simply override with the latest depth measured
  // TODO: find something better like averaging and outlier rejection ?
  elm = depth;
}

bool CoverageRecorder::addPointCloudToCoverage(const sensor_msgs::PointCloud &pcld)
{
  if (pcld.header.frame_id != m_worldFrameId)
  {
    return false;
  }
  auto cell_cnt = 0;
  // Increment the cell where each point of the cloud falls
  for (const auto &pt : pcld.points)
  {
    cell_cnt++;
    addPointToGrid(POINT_CLOUD_BASED_COVERAGE, {pt.x, pt.y});
  }

  ROS_DEBUG_STREAM("New pt Cloud covered " << cell_cnt << " cells.");
  return true;
}

void CoverageRecorder::addPointToGrid(const std::string layer, const grid_map::Position &pt)
{
  auto &elm = m_gridMap.atPosition(layer, pt);
  if (std::isnan(elm))
  {
    elm = 1.f;
  }
  else
  {
    elm += 1.f;
  }
}

bool CoverageRecorder::addPointCloudToCoverage(const sensor_msgs::PointCloud2 &pcld)
{
  if (pcld.header.frame_id != m_worldFrameId)
  {
    return false;
  }

  pcl::PointCloud<pcl::PointXYZ> worldCld;
  pcl::fromROSMsg(pcld, worldCld);

  auto cell_cnt = 0;
  // Increment the cell where each point of the cloud falls
  for (const auto &pt : worldCld.points)
  {
    cell_cnt++;
    addPointToGrid(POINT_CLOUD_BASED_COVERAGE, {pt.x, pt.y});
    setCellDepth({pt.x, pt.y}, pt.z);
  }

  ROS_DEBUG_STREAM("New pt Cloud covered " << cell_cnt << " cells.");
  return true;
}

CoverageResult CoverageRecorder::getCoveragePercent(const BPolygon &op_region, uint min_ping_per_cell) const
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
    if (m_gridMap.at(RAY_BASED_COVERAGE, *it) > min_ping_per_cell)
    {
      ray_cells++;
    }
    if (m_gridMap.at(POINT_CLOUD_BASED_COVERAGE, *it) > min_ping_per_cell)
    {
      point_cloud_cells++;
    }

    cells_in_area++;
  }
  ROS_DEBUG_STREAM("\nMeasured cells:" << point_cloud_cells << "Ray Cells" << ray_cells << "\nCells in area "
                                       << cells_in_area);
  return {(ray_cells * 100. / cells_in_area), (point_cloud_cells * 100. / cells_in_area)};
}

void CoverageRecorder::setGridMapCenter(double lat, double lon)
{
  if (not m_localCartReceived)
  {
    m_localCartReceived = true;
    m_localCart = GeographicLib::LocalCartesian(lat, lon);
  }
  else
  {
    // Update gridmap position to new world coordinate system:
    // let O be the old world coordinates, N the new world coordinates and T the map position
    GeographicLib::LocalCartesian new_local(lat, lon);
    double lat_oldRef = m_localCart.LatitudeOrigin();
    double long_oldRef = m_localCart.LongitudeOrigin();
    ROS_DEBUG_STREAM("Updating ref from geo(" << lat_oldRef << " ; " << long_oldRef << ") to geo("
                                              << lat << " ; "
                                              << lon << ")");
    double x_oldLocal_inNewRef;
    double y_oldLocal_inNewRef;
    double z;
    new_local.Forward(lat_oldRef, long_oldRef, 0, x_oldLocal_inNewRef, y_oldLocal_inNewRef, z);
    auto map_pos_inOldRef = m_gridMap.getPosition();

    // Then NT = NO + OT = coord of old local coord expressed in new local coord + coord of map in old local coord
    auto x_new_map_pos = x_oldLocal_inNewRef + map_pos_inOldRef.x();
    auto y_new_map_pos = y_oldLocal_inNewRef + map_pos_inOldRef.y();

    grid_map::Position pos(x_new_map_pos, y_new_map_pos);
    m_gridMap.setPosition(pos);
    m_localCart = new_local;
  }
}

CoverageParams CoverageRecorder::getParams() const
{
  return m_params;
}

void CoverageRecorder::setParams(const CoverageParams &params)
{
  m_params = params;
  initGridMapFromParams();
}
