/**
 * @file SwathRecord.h
 * @brief Represends a record of a sonar measurement on a moving vehicle.
 * @author Damian Manda
 * @date 23 Feb 2016
 * @copyright MIT License
 */

#pragma once

#include <eigen3/Eigen/Core>

namespace scov
{
using EPoint = Eigen::Vector2d;
/**
 * @enum BoatSide
 * @brief Indicates the side of a boat for the swath
 */
enum class BoatSide
{
  Stbd,
  Port,
  Unknown
};

struct OuterPoints
{
  EPoint port_pt;
  EPoint stbd_pt;
};

/**
 * @struct SwathRecord
 * @brief Stores the location and width of one measured sonar swath.
 */
class SwathRecord
{
 public:
  // Sonar position in world coordinates
  double loc_x;    // X Sonar position in meters
  double loc_y;    // Y Sonar position in meters
  double heading;  // Sonar heading, true heading (clockwise from N)
  // Swath distances relative the sonar position
  double swath_stbd;
  double swath_port;
  // This will either be the nadir depth (if this is all we have) or outer
  // swath depth
  double depth;

  /**
   * Gets the x,y position of the edge of a swath from a record
   * @param  record The swath record to use for location and width
   * @param  side   The side of the boat on which to project the swath
   * @return        Location of the swath outer points
   */
  EPoint outerPoint(BoatSide side) const;
  /**
   * @brief outerPoints
   * @return both outher points into a struct
   */
  OuterPoints outerPoints() const
  {
    return {outerPoint(BoatSide::Port), outerPoint(BoatSide::Stbd)};
  }
};
}  // namespace scov
