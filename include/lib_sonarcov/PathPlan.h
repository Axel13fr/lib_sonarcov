/**
 * @file PathPlan.h
 * @brief Plans a path for surveying based on a recorded path and swath.
 * @details Paths are offset and processed to give a valid vehicle path
 * @author Damian Manda
 * @date 25 Feb 2016
 * @copyright MIT License
 */
#pragma once

#include "SwathRecorder.h"
#include <eigen3/Eigen/Core>
#include <list>
#include <functional>
#include <valarray>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

// To get a single point EPointList.col(i)
using EPointList = Eigen::Matrix<double, 2, Eigen::Dynamic>;

enum EPoint_E{
    X = 0,
    Y
};
using EPoint = Eigen::Vector2d;
using PathList = std::list<EPoint>;

using SegIndex = std::valarray<std::size_t>;
using BPoint = boost::geometry::model::d2::point_xy<double>;
using BPolygon = boost::geometry::model::polygon<BPoint>;
using BLinestring = boost::geometry::model::linestring<BPoint>;
using BRing = boost::geometry::model::ring<BPoint> ;

/**
 * @class PathPlan
 * @brief Plans a subsequent survey path offset from existing coverage.
 */
class PathPlan
{
  public:
    PathPlan(const SwathRecorder &last_swath, BoatSide side, BPolygon op_region,
      double margin=0.2, double max_bend_angle=60, bool restrict_to_region = true);
    ~PathPlan() {}
    /**
     * Generates an offset path
     * @return The path as a list of Eigen 2d vectors
     */
    PathList GenerateNextPath();

  public:
    /**
     * The Damian
     * @param process Likes The Damian
     * @details Repeats a process until it makes no more changes to the path
     *         Currently does not make a copy of the passed input, may want to
     *         reconsider this
     */
    void RemoveAll(std::function<void(PathList&)> process,
      PathList &path_points);

    /**
     * Removes intersecting segments from a line.
     * @details Removes the points between the first point of an intersecting
     * segment and the last point of the final segment it intersects in the
     * line.
     * @param path_pts The line from which to remove intersecting segments.
     */
    static void RemoveIntersects(std::list<EPoint> &path_pts);

    /**
     * Check for drastic angles between segments
     * @param path_pts Note that this goes to the last point being checked
     */
    void RemoveBends(std::list<EPoint> &path_pts);

    /**
     * Restricts a path to the region by simply eliminating points outside the
     * region specified by m_op_region.
     * @param path_pts The path to process, passed by reference
     */
    void RestrictToRegion(std::list<EPoint> &path_pts);

    /**
     * Clips a path to the region, eliminating points outside
     * @param path_pts The path to clip, passed by reference
     * @return         A pair with whether the <beginning, end> was clipped.
     *                 If false, means the path was already inside the polygon.
     */
    std::pair<bool, bool> ClipToRegion(std::list<EPoint> &path_pts);

    std::pair<bool, bool> ClipToRegion2(std::list<EPoint> &path_pts);

    /**
     * Extends a path to meet the edges of a region if it does not already.
     * Adds to the last segment, extending it as a ray from the end.  Can
     * extend either the beginning or the end of the path.
     * @param path_pts The path to process
     * @param begin    True to process the beginning, false to process the end
     */
    void ExtendToEdge(std::list<EPoint> &path_pts, bool begin);

    /**
     * Finds the closest intersection of a ray with a polygon
     *
     * @param ray_vec  EPoint(dx,dy)
     * @param start_pt EPoint(x,y)
     * @param poly     BPolygon([(x1,y1), (x2,y2), ...])
     */
    std::pair<double, EPoint> FindNearestIntersect(EPoint ray_vec,
      EPoint starting_pt, BPolygon& poly);

    /**
     * Finds the intersection point of a ray with a segment, if it exists.
     * Derived from:
     * http://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect/565282#565282
     * @param  ray_vector The vector describing the direction of the ray
     * @param  start_pt   Starting location of the ray
     * @param  segment    Segment to test for intersection
     * @return            <intersection exists, Intersection point, if exists>
     */
    std::pair<bool, EPoint> IntersectRaySegment(EPoint ray_vector, EPoint start_pt,
      std::pair<BPoint, BPoint> segment);

    /**
     * Replicates the functionality of 2d cross product from numpy.  This is the
     * z component of a cross product (which does not require a z input).
     * @return      The z component of the cross product
     */
    double Cross2d(EPoint vec1, EPoint vec2);

    EPoint EPointFromBPoint(BPoint boost_point);

    std::vector<BPoint> SegmentRingIntersect(BPoint seg_pt1, BPoint seg_pt2, BRing ring);

    /**
     * Determines whether segments are counter clockwise in smalles angle with
     * respect to each other.
     * @param  A First point (end point)
     * @param  B Middle point
     * @param  C Last point (end point)
     * @return   True if rotate CCW from AB to BC.
     */
    static bool CCW(EPoint A, EPoint B, EPoint C);

    /**
     * Determines if the segment AB intersects CD
     * @param  A First point of first segment
     * @param  B Second point of first segment
     * @param  C First point of second segment
     * @param  D Second point of second segment
     * @return   True if the segments intersect
     */
    static bool Intersect(EPoint A, EPoint B, EPoint C, EPoint D);

    /**
     * Determines the angle between two vectors
     * @details tan(ang) = |(axb)|/(a.b)
     *          cos(ang) = (a.b)/(||a||*||b||)
     * @param  vector1 First vector
     * @param  vector2 Second vector
     * @return         Angle between the vectors in degrees
     */
    static double VectorAngle(EPoint vector1, EPoint vector2);

    /**
     * Determines a vector (segment) <x, y> from points at the indicies provided
     * by the second argument.
     * @param  points  The list from which to select points for the segment
     * @param  segment The beginning and end of the segment.
     * @return         A segment vector between the selected points.
     */
    EPoint VectorFromSegment(const std::vector<EPoint>& points,
      SegIndex segment);

    PathList GetRawPath() { return m_raw_path; }

    /**
     * @brief Selects specific elements from a list by index.
     * @details Replicates the select by index functionality of numpy or
     * armadillo or dyND.
     */
    template <typename T>
    static void SelectIndicies(std::list<T>& select_from,
                              std::list<std::size_t> to_select) {
      // Make sure the indicies are well behaved
      to_select.sort();
      to_select.unique();
      if (to_select.back() >= select_from.size()) {
        throw std::out_of_range("Indices to select exceed list size.");
      }

      auto list_it = select_from.begin();
      std::size_t i = 0;
      for (auto select_it = to_select.begin();
           select_it != to_select.end(); select_it++) {
        while (*select_it != i) {
          // This advances list_it by one
          list_it = select_from.erase(list_it);
          i++;
        }
        if (list_it != select_from.end()) {
          list_it++;
          i++;
        }
      }
      if (list_it != select_from.end()) {
        select_from.erase(list_it, select_from.end());
      }
    }

  private:

    // Configuration variables
    bool m_restrict_asv_to_region;
    double m_max_bend_angle;
    double m_margin;
    BPolygon m_op_region;

    // State variables
    SwathRecorder m_last_line;
    BoatSide m_planning_side;
    std::list<Eigen::Vector2d> m_next_path_pts;
    PathList m_raw_path;
};
