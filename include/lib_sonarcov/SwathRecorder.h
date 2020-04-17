/**
 * @file SwathRecord.h
 * @brief Records a swath history from a sonar on a moving vehicle.
 * @details Also gives the outer points at the edge of the swath along a track.
 * @author Damian Manda
 * @date 23 Feb 2016
 * @copyright MIT License
 */

#pragma once

#include "SwathRecord.h"

#include <eigen3/Eigen/Core>
#include <list>
#include <vector>
#include <map>

namespace scov{

using EPoint = Eigen::Vector2d;
using EPointVec = std::vector<EPoint>;

/**
 * @class RecordSwath
 * @brief Records points of a sonar swath for analysis of the coverage and
 * subsequent tracks.
 */
class SwathRecorder
{

public:
  explicit SwathRecorder(double interval = 10);
  /**
  * Adds a recorded swath to the path.
  * @param  swath_stbd Swath width to starboard
  * @param  swath_port Swath width to port
  * @param  loc_x      X coordinate of position where record takes place
  * @param  loc_y      Y coordinate of position
  * @param  heading    Heading of the vessel at the time of recording
  * @return            True if the record coverage was successfully added
  */
  bool AddRecord(const SwathRecord& r);
  /**
  * Resets the storage for a new line.
  */
  void ResetLine();

  /**
  * Saves the last point to a record.
  * This makes sure that the last swath (after crossing the boundary) is
  * recorded so that it is included in planning.
  * @return If the min_record is valid
  */
  bool SaveLast();

  /**
  * Get all of the points on one side of the swath limits
  * @param side   The side of the boat on which to return the swath
  * @return       An ordered list of the points on the outside of the swath
  */
  EPointVec SwathOuterPts(BoatSide side);

  std::pair<EPoint, EPoint> LastOuterPoints();

  /**
  * Gets a specific width along a recorded decimated swath
  * @param  side  Side of the boat on which the swath was recorded
  * @param  index Position of the desired swath
  * @return       The swath width in meters
  */
  double SwathWidth(BoatSide side, size_t index);

  /**
   * Gets all the minimum swath widths on a side (recorded at the set side)
   * @param side The side from which to get swaths
   * @return     A vector of swath widths.
   */
  std::vector<double> AllSwathWidths(BoatSide side);

  /**
   * Gets the x,y location of where a specific min swath was recorded
   * @param  index The index along the swath record
   * @return       EPoint of recording location
   */
  EPoint SwathLocation(unsigned int index);

  /**
  * Sets the side that will be used for outer point determination
  * @param side Side of the boat on which to generate outer swath points
  */
  void SetOutputSide(BoatSide side) { m_output_side = side; }

  /**
   * Gets the side on which minimum interval points are being processed
   */
  BoatSide GetOutputSide() {return m_output_side; }

  /**
   * The distance between subsequent analysis intervals for swath minimums.
   */
  double IntervalDist() { return m_interval; }

  /**
   * Determines if the record has valid points for building a path.
   */
  bool ValidRecord();

  /**
   * @brief recordsNumber
   * @return number of records accumulated after decimation by the recorder
   */
  size_t recordsNumber()
  {
    return m_min_record.size();
  }

 private:
  /**
  * Determines the minimum swath over the recorded interval and places it into
  * a list of minimums.
  */
  void MinInterval();

  /**
  * Adds a record to the coverage model.
  * @param  record The record to add
  * @return        Whether the record was able to be added sucessfully (no
  * geometry errors).
  */
  bool AddToCoverage(SwathRecord record);

  // Configuration Variables
  double m_min_allowable_swath;
  double m_interval;

  // All records
  std::vector<SwathRecord> m_interval_record;
  // Minimum record in terms of swath on a given side, over a distance interval
  std::vector<SwathRecord> m_min_record;
  // Swath records over an interval
  std::map<BoatSide, std::vector<double>> m_interval_swath;

  double m_last_x;
  double m_last_y;
  bool m_has_records;
  double m_acc_dist;
  SwathRecord m_previous_record;
  // side on which minimum interval points are being processed
  BoatSide m_output_side;

};
}
