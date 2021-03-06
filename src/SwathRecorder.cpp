/************************************************************/
/*    NAME: Damian Manda                                    */
/*    ORGN: UNH                                             */
/*    FILE: RecordSwath.cpp                                 */
/*    DATE: 23 Feb 2016                                     */
/************************************************************/

#include "lib_sonarcov/SwathRecorder.h"
#include "lib_sonarcov/AngleUtils.h"
#include <ros/console.h>
#include <cmath>
#include <algorithm>

using namespace scov;
using namespace ang;

static constexpr auto DEBUG = true;
static constexpr auto TURN_THRESHOLD = 20;

//---------------------------------------------------------
// Constructor

SwathRecorder::SwathRecorder(double interval) : m_min_allowable_swath(0),m_interval(interval),
                         m_has_records(false), m_acc_dist(0),
                         m_previous_record{0, 0, 0, 0, 0, 0},
                         m_output_side(BoatSide::Unknown)
{
    // Initialize the point records
    m_interval_swath[BoatSide::Port] = std::vector<double>();
    m_interval_swath[BoatSide::Stbd] = std::vector<double>();
}

bool SwathRecorder::AddRecord(const SwathRecord &r)
{
    // Dont add records at duplicate location
    if (r.loc_x == m_previous_record.loc_x && r.loc_y == m_previous_record.loc_y
            && r.heading == m_previous_record.heading)
        return false;

    m_interval_record.push_back(r);
    m_interval_swath[BoatSide::Stbd].push_back(r.swath_stbd);
    m_interval_swath[BoatSide::Port].push_back(r.swath_port);

    if (m_has_records)
    {
        m_acc_dist += hypot((m_last_x - r.loc_x), (m_last_y - r.loc_y));

        ROS_DEBUG_STREAM_COND(DEBUG, "Accumulated distance: " + std::to_string(m_acc_dist) + "\n");

        // Save minimum over interval
        if (m_acc_dist >= m_interval)
        {
            ROS_DEBUG_STREAM_COND(DEBUG, "Running MinInterval()\n");
            m_acc_dist = 0;
            MinInterval();
        }
        else if(not m_min_record.empty())
        {
            //Override the min interval on turns to the outside
            double turn = angle180(angle180(r.heading) - angle180(m_min_record.back().heading));
            if ((turn > TURN_THRESHOLD && m_output_side == BoatSide::Port)
                    || (turn < -TURN_THRESHOLD && m_output_side == BoatSide::Stbd))
            {
                ROS_DEBUG_STREAM_COND(DEBUG, "Adding Turn Based Point\n");
                m_min_record.push_back(r);
                m_interval_record.clear();
                m_interval_swath.clear();
            }
        }
    }

    m_last_x = r.loc_x;
    m_last_y = r.loc_y;
    m_has_records = true;
    m_previous_record = r;

    return true;
}

void SwathRecorder::MinInterval()
{
    // Get the record from the side we are offsetting
    if (m_output_side == BoatSide::Unknown)
    {
        throw std::runtime_error("Cannot find swath minimum without output side.");
    }
    std::vector<double>& side_record = m_interval_swath[m_output_side];

    std::size_t min_index = 0;
    if (side_record.size() > 0)
    {
      auto min_elm = std::min_element(side_record.begin(), side_record.end());
      min_index = static_cast<size_t>(std::distance(side_record.begin(), min_elm));
    }

    if (m_interval_record.size() > min_index)
    {
        // Add the first point if this is the first interval in the record
        // and the min is not that first point
        if (m_min_record.size() == 0 && min_index != 0)
        {
            ROS_DEBUG_STREAM_COND(DEBUG, "Saving First record of line\n");
            m_min_record.push_back(m_interval_record[0]);
        }
        m_min_record.push_back(m_interval_record[min_index]);

        // These are always cleared in the python version
        m_interval_record.clear();
        m_interval_swath.clear();
    }
}

bool SwathRecorder::SaveLast()
{
    if (m_min_record.size() > 0 && m_interval_record.size() > 0)
    {
        SwathRecord last_min = m_min_record.back();
        SwathRecord last_rec = m_interval_record.back();
        if (last_min.loc_x != last_rec.loc_x || last_min.loc_y != last_rec.loc_y)
        {
            ROS_DEBUG_STREAM_COND(DEBUG, "Saving last record of line, (" << last_rec.loc_x << ", "
                                                             << last_rec.loc_y << ")\n");
            m_min_record.push_back(last_rec);
        }
        return true;
    }
    return false;
}

void SwathRecorder::ResetLine()
{
    m_interval_record.clear();
    m_min_record.clear();
    m_interval_swath[BoatSide::Stbd].clear();
    m_interval_swath[BoatSide::Port].clear();
    m_acc_dist = 0;
    m_has_records = false;
}

EPointVec SwathRecorder::SwathOuterPts(BoatSide side)
{
    EPointVec points;
    for (const auto &record : m_min_record)
    {
        auto outer_pt = record.outerPoint(side);
        points.push_back(outer_pt);
    }
    return points;
}

OuterPoints SwathRecorder::LastOuterPoints()
{
    if (m_has_records)
    {
      return m_previous_record.outerPoints();
    }
    return {};
}

double SwathRecorder::SwathWidth(BoatSide side, size_t index)
{
    if (m_min_record.size() > index)
    {
        const auto r = m_min_record[index];
        if (side == BoatSide::Stbd)
        {
            return r.swath_stbd;
        }
        else if (side == BoatSide::Port)
        {
            return r.swath_port;
        }
    }
    // TODO: proper error handling ?
    return 0;
}

std::vector<double> SwathRecorder::AllSwathWidths(BoatSide side)
{
    std::vector<double> widths;
    widths.reserve(m_min_record.size());
    for (const auto& r : m_min_record)
    {
        if (side == BoatSide::Stbd)
        {
            widths.push_back(r.swath_stbd);
        }
        else if (side == BoatSide::Port)
        {
            widths.push_back(r.swath_port);
        }
    }
    return widths;
}

EPoint SwathRecorder::SwathLocation(unsigned int index)
{
    if (m_min_record.size() > index)
    {
        const auto r = m_min_record[index];
        return EPoint(r.loc_x, r.loc_y);
    }
    throw std::out_of_range("Swath index out of range.");
}

bool SwathRecorder::ValidRecord()
{
    return (m_min_record.size() > 1);
}
