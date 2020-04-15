/**
 * @file PathPlan.cpp
 * @brief Plans a path for surveying based on a recorded path and swath.
 * @author Damian Manda
 * @date 25 Feb 2016
 * @copyright MIT License
 */

#include "lib_sonarcov/PathPlan.h"

#include <ros/console.h>
#include <eigen3/Eigen/Geometry>
#include <cmath>
#include <iterator>
#include <iostream>


using namespace scov;
static constexpr auto DEBUG = false;
namespace bg = boost::geometry;
static const Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

PathPlan::PathPlan(const SwathRecorder &last_swath, BoatSide side, BPolygon op_region,
  double margin, double max_bend_angle, bool restrict_to_region) :
  m_restrict_asv_to_region(restrict_to_region), m_max_bend_angle(max_bend_angle),
  m_margin(margin), m_op_region{op_region},m_last_line(last_swath), m_planning_side(side) {

  //1 is half stepping, we don't really need to go larger
  if (m_margin > 1) {
    m_margin = 1;
  }

}

PathList PathPlan::GenerateNextPath() {
  ROS_DEBUG_STREAM_COND(DEBUG,"\n======== Generating Next Path ========\n");

  std::vector<EPoint> edge_pts = m_last_line.SwathOuterPts(m_planning_side);

  ROS_DEBUG_COND(DEBUG,"Basis Points: %ld\n", edge_pts.size());
//  ROS_DEBUG_STREAM_COND(DEBUG,edge_pts[2].get_spec_pts(2) + "\n");

  if (edge_pts.size() < 2)
    return PathList();

  bool all_zero = true;
  double swath_width = 0;

  double rot_angle = 0;
  if (m_planning_side == BoatSide::Stbd) {
    rot_angle = -M_PI/2;
  } else if (m_planning_side == BoatSide::Port) {
    rot_angle = M_PI/2;
  }
  Eigen::Rotation2D<double> rot_matrix(rot_angle);

  for(size_t i = 0; i < edge_pts.size(); i++) {
    Eigen::Vector2d back_vec;
    Eigen::Vector2d forward_vec;
    bool back_vec_set = false;
    if (i > 0) {
      back_vec =  EPoint(edge_pts[i][X] - edge_pts[i-1][X],
                         edge_pts[i][Y] - edge_pts[i-1][Y]);
      back_vec_set = true;
    }
    if (i < edge_pts.size() - 1) {
      forward_vec = EPoint(edge_pts[i+1][X] - edge_pts[i][X],
          edge_pts[i+1][Y] - edge_pts[i][Y]);
    } else {
      forward_vec = back_vec;
    }
    if (!back_vec_set) {
      back_vec = forward_vec;
    }

    // Make sure this point is unique, could also test point locations
    if (back_vec.squaredNorm() != 0 && forward_vec.squaredNorm() != 0) {
      // Average the headings
      back_vec.normalize();
      forward_vec.normalize();
      Eigen::Vector2d avg_vec = (back_vec + forward_vec) / 2;

      // Get the offset vector
      Eigen::Vector2d offset_vec = rot_matrix * avg_vec;
      offset_vec.normalize();
      swath_width = m_last_line.SwathWidth(m_planning_side, i);
      offset_vec *= swath_width * (1 - m_margin);

      // Get offset location and save
      Eigen::Vector2d swath_loc(edge_pts[i][X], edge_pts[i][Y]);
      m_next_path_pts.push_back(swath_loc + offset_vec);

//       ROS_DEBUG_STREAM_COND(DEBUG,"Swath Width: %0.2f  Offset X: %0.2f Offset Y: %0.2f Avg Vec: <%0.2f, %0.2f>\n",
//         swath_width, offset_vec.x(), offset_vec.y(), avg_vec.x(), avg_vec.y());
//       ROS_DEBUG_STREAM_COND(DEBUG,"Back Vec: <%0.2f, %0.2f>, Forward Vec: <%0.2f, %0.2f>\n",
//         back_vec.x(), back_vec.y(), forward_vec.x(), forward_vec.y());
    }
    all_zero = all_zero && (swath_width == 0);
  }

  // Completion of the survey is when all swath widths are zero (depth threshold
  // reached).  This is done by the simulator or pSonarFilter, should eventually
  // move into this processing.
  if (all_zero) {
    ROS_DEBUG_STREAM_COND(DEBUG,"Reached end of path by depth threshold\n");
    return PathList();
  }

  // Next line is in opposite direction
  m_next_path_pts.reverse();
  m_raw_path = m_next_path_pts;

  // ---------- Intersections -----------
  size_t pre_len = m_next_path_pts.size();
  ROS_DEBUG_STREAM_COND(DEBUG,"Eliminating path intersects itself.\n");
  RemoveAll(RemoveIntersects, m_next_path_pts);

  ROS_DEBUG_COND(DEBUG,"Removed %ld points.\n", pre_len - m_next_path_pts.size());
  pre_len = m_next_path_pts.size();

  // ---------- Bends -----------
  ROS_DEBUG_STREAM_COND(DEBUG,"Eliminating sharp bends.\n");
  //XYSegList pts = VectorListToSegList(m_next_path_pts);
//  ROS_DEBUG_STREAM_COND(DEBUG,pts.get_spec_pts(2) + "\n");


  std::function<void(std::list<EPoint>&)> remove_func =
    std::bind(&PathPlan::RemoveBends, this, std::placeholders::_1);
  RemoveAll(remove_func, m_next_path_pts);

  ROS_DEBUG_COND(DEBUG,"Removed %ld points.\n", pre_len - m_next_path_pts.size());
  pre_len = m_next_path_pts.size();

  // ---------- Restrict to Region -----------
  // Would be good to check for segment intersecting with the border and use
  // these points instead
  std::pair<bool, bool> clipped = std::make_pair(false, false);
  if (m_restrict_asv_to_region) {
    ROS_DEBUG_STREAM_COND(DEBUG,"Eliminating points outside op region.\n");

    //RestrictToRegion(m_next_path_pts);
    clipped = ClipToRegion(m_next_path_pts);

    ROS_DEBUG_COND(DEBUG,"Removed %ld points.\n", pre_len - m_next_path_pts.size());
    pre_len = m_next_path_pts.size();

    if (pre_len <= 1) {
      return m_next_path_pts;
    }
  }

  // ---------- Extend -----------
  // Idea: Maybe extend from the point to the nearest edge, not along the
  // vector of the last segment, or add swath width along the edge from last
  ROS_DEBUG_STREAM_COND(DEBUG,"Extending ends of path to edge of region.\n");

  if (!clipped.first)
    ExtendToEdge(m_next_path_pts, true);
  if (!clipped.second)
    ExtendToEdge(m_next_path_pts, false);

  ROS_DEBUG_COND(DEBUG,"Removed %ld points.\n", pre_len - m_next_path_pts.size());
  pre_len = m_next_path_pts.size();

  return m_next_path_pts;
}

void PathPlan::RemoveAll(std::function<void(std::list<Eigen::Vector2d>&)> process,
                         PathList &path_points) {
  size_t pre_len = path_points.size();
  process(path_points);
  // keep repeating until no more changes
  while (path_points.size() < pre_len) {
    pre_len = path_points.size();
    process(path_points);
  }
}

void PathPlan::RemoveIntersects(std::list<EPoint> &path_pts) {
  ROS_DEBUG_STREAM_COND(DEBUG,"Running Remove Intersects\n");

  // Can't be an intersection between two segments (unless collinear)
  if (path_pts.size() < 4)
    return;

  auto path_iter = path_pts.begin();
  auto last_initial_seg = std::prev(path_pts.end(), 4);
  auto back_3 = std::next(last_initial_seg);
  auto back_2  = std::next(back_3);
  // This is the actual end of the list
  auto last_test_seg = std::next(back_2);

  // The < operator doesn't work with non-random-access iterators
  while (path_iter != last_test_seg &&
    path_iter != back_3 && path_iter != back_2) {
    // Segment to test
    Eigen::Vector2d this_seg_a = *path_iter;
    Eigen::Vector2d this_seg_b = *(++path_iter);
    ROS_DEBUG_STREAM("First Seg Point 1: " << this_seg_a.transpose());
    ROS_DEBUG_STREAM("First Seg Point 2: " << this_seg_b.transpose());

    // Test following segments in the list
    auto j = std::next(path_iter);
    auto next_non_intersect(path_iter);
    while(j != last_test_seg) {
      Eigen::Vector2d check_seg_a = *j;
      Eigen::Vector2d check_seg_b = *(++j);
      ROS_DEBUG_STREAM("Check Seg Point 1: " << check_seg_a.transpose());
      ROS_DEBUG_STREAM("Check Seg Point 2: " << check_seg_b.transpose());
      if (Intersect(this_seg_a, this_seg_b, check_seg_a, check_seg_b)) {
        next_non_intersect = j;
        ROS_DEBUG_STREAM("Found Intersect!\n");
      }
    }
    // If an intersection was found, remove the elements causing it.  Otherwise
    // the iterator advances to the next element
    if (next_non_intersect != path_iter) {
      path_iter = path_pts.erase(path_iter, next_non_intersect);
    }
  }

  // Should not need to port the code adding the end points, as this removes
  // from the list instead
}

void PathPlan::RemoveBends(std::list<EPoint> &path_pts) {
  // Maybe should process in both directions, remove pts common to both
  // Or take the method with less points removed
  ROS_DEBUG_STREAM("Running remove bends.\n");

  std::list<std::size_t> non_bend_idx = {0};
  // Need to be able to randomly access the path points
  //std::vector<EPoint> v_pts{ std::begin(path_pts), std::end(path_pts) };
  std::vector<EPoint> v_pts;
  v_pts.reserve(path_pts.size());
  std::copy(std::begin(path_pts), std::end(path_pts), std::back_inserter(v_pts));
  ROS_DEBUG_STREAM("Copied elements to vector, size = " << v_pts.size() << "\n");

  SegIndex this_seg = {0, 1};
  SegIndex next_seg = {1, 2};
  SegIndex inc_seg = {0, 1};

  std::size_t last_index = v_pts.size() - 1;

  while (next_seg[1] < last_index) {
    ROS_DEBUG_STREAM("Looping through path: (" << this_seg[0] << "," << this_seg[1] << ")");
    ROS_DEBUG_STREAM(" - (" << next_seg[0] << "," << next_seg[1] << ")\n");

    EPoint this_vec = v_pts[this_seg[1]] - v_pts[this_seg[0]];
    EPoint next_vec = v_pts[next_seg[1]] - v_pts[next_seg[0]];

    // Angle between this and following segment
    if (VectorAngle(this_vec, next_vec) > m_max_bend_angle) {
      // If too large, have to find the next point that makes it small enough
      // -- Method 1: Points from end of current --
      SegIndex test_seg = next_seg + inc_seg;
      // TODO: Added this in response to an error, not sure if correct sln
      double angle1 = 500;
      double angle2 = 500;
      while (test_seg[1] < last_index) {
        EPoint test_vec = v_pts[test_seg[1]] - v_pts[test_seg[0]];
        angle1 = VectorAngle(this_vec, test_vec);
        if (angle1 < m_max_bend_angle)
          break;
        test_seg += inc_seg;
      }
      std::size_t pts_elim1 = test_seg[1] - test_seg[0];

      // -- Method 2: Points from beginning of current --
      // check if it would be better to remove the end of "this_vec"
      // TODO: Not entirely sure why we can't use first point

      // force it to not choose the second method by default
      std::size_t pts_elim2 = pts_elim1 + 1;
      SegIndex test2_seg2;
      if (non_bend_idx.size() > 1) {
        EPoint test2_vec1 = VectorFromSegment(v_pts, SegIndex{
          *(std::prev(non_bend_idx.end(), 3)), this_seg[0]});
        test2_seg2 = SegIndex{this_seg[0], next_seg[1]};
        if (test2_seg2[1] > last_index)
          break;  // from outer while loop
        while (test2_seg2[1] < last_index) {
          EPoint test2_vec2 = VectorFromSegment(v_pts, test2_seg2);
          angle2 = VectorAngle(test2_vec1, test2_vec2);
          if (angle2 < m_max_bend_angle)
            break;
          test2_seg2 += inc_seg;
        }
        pts_elim2 = test2_seg2[1] - test2_seg2[0];

        // Check if one method angle is better
        // may need to refine this threshold or only use angle (or add
        // a buffer to the angle for ones that are close)
        // really needs to be total points removed in region that would
        // have been affected by the other method, but this requires additional
        // loops to know
        if (angle1 == 500 || angle2 == 500) {
          // Means we are checking a segment at the end of the line
            ROS_DEBUG_STREAM_COND(DEBUG,"Encountered default angle state, this is not good.\n");
        } else {
          if (pts_elim1 > pts_elim2 && pts_elim1 < (pts_elim2 * 2)
              && angle1 < angle2) {

             ROS_DEBUG_COND(DEBUG,"Bend Fudging - pts_elim1: %ld, pts_elim2: %ld\n\t"
                        "angle1: %.2f, angle2: %.2f\n", pts_elim1, pts_elim2,
                        angle1, angle2);

            pts_elim2 = pts_elim1 + 1;
          } else if (pts_elim2 >= pts_elim1 && pts_elim2 < (pts_elim1 * 2)
                     && angle2 < angle1) {
             ROS_DEBUG_COND(DEBUG,"Bend Fudging - pts_elim1: %ld, pts_elim2: %ld\n\t"
                       "angle1: %.2f, angle2: %.2f\n", pts_elim1, pts_elim2,
                       angle1, angle2);

            pts_elim1 = pts_elim2 + 1;
          }
        }
      }

      // -- Method 3: Remove in Both Directions [Maybe Most preferable?] --
      size_t pts_elim3 = std::max(pts_elim1, pts_elim2) + 1;
      double fwd_angle = 500;
      double back_angle = 500;
      SegIndex test_fwd = next_seg;
      SegIndex test_back = this_seg;
      //SegIndex inc_both = {1, 1};
      bool move_fwd = false;
      bool move_back = false;
      bool meth3_found_soln = true;

      ROS_DEBUG_STREAM_COND(DEBUG,"Running Method 3\n");
      while (fwd_angle > m_max_bend_angle || back_angle > m_max_bend_angle) {
        move_fwd = false;
        move_back = false;
        if (test_fwd[1] < last_index) {
          test_fwd += 1;
          move_fwd = true;
        }
        if (test_back[0] > 0) {
          test_back -= 1;
          move_back = true;
        }

        ROS_DEBUG_STREAM("Test_Fwd: (" << test_fwd[0] << "," << test_fwd[1] << ")");
        ROS_DEBUG_STREAM(" - Test_Back (" << test_back[0] << "," << test_back[1] << ")\n");

        if (!move_back && !move_fwd) {
          // Reached both ends
          meth3_found_soln = false;
          break;
        }
        EPoint fwd_vec = VectorFromSegment(v_pts, test_fwd);
        EPoint back_vec = VectorFromSegment(v_pts, test_back);
        EPoint btwn_vec = VectorFromSegment(v_pts, SegIndex{test_back[1], 
                                            test_fwd[0]});
        fwd_angle = VectorAngle(btwn_vec, fwd_vec);
        back_angle = VectorAngle(back_vec, btwn_vec);
      }
      if (meth3_found_soln)
        pts_elim3 = test_fwd[0] - test_back[0];

      // Make the new vector to check next
      if (pts_elim1 <= pts_elim2 && pts_elim1 <= pts_elim3) {
        this_seg = test_seg;
        next_seg = SegIndex{test_seg[1], test_seg[1]+1};
      } else if (pts_elim2 <= pts_elim3) {
        this_seg = test2_seg2;
        next_seg = SegIndex{test2_seg2[1], test2_seg2[1]+1};
      } else {
        this_seg = test_fwd;
        next_seg = SegIndex{test_fwd[1], test_fwd[1]+1};
      }
    } else {
      this_seg = next_seg;
      next_seg += 1;
    }
    // don't re-add an existing index
    if (*(--non_bend_idx.end()) != this_seg[0]) {
      non_bend_idx.push_back(this_seg[0]);
    }
  } // end main while

  //------------ Final Processing to ensure good path ----------------

  if (this_seg[1] == path_pts.size() - 1) {
    // Looped to the end, this indicates a large skip, otherwise we don't
    // get to the last index for this_seg, so remove the test point which
    // could be causing the problem
    //this might also work with this_seg[0]+1

    // does this work now that the lists are passed by reference?
    path_pts.erase(std::next(path_pts.begin(), this_seg[0]));
    // might want to put this recursion path at the end
    ROS_DEBUG_STREAM("Recursing due to skip to end.\n");
    RemoveBends(path_pts);
    return;
  } else {
    // Extend to the end of the path
    for (auto i = this_seg[1]; i < path_pts.size(); i++) {
      non_bend_idx.push_back(i);
    }
  }

  // Remove the end if it causes a bend
  while (non_bend_idx.size() > 2) {
    SegIndex first_vec{*(std::prev(non_bend_idx.end(), 3)),
                       *(std::prev(non_bend_idx.end(), 2))};
    SegIndex second_vec{*(std::prev(non_bend_idx.end(), 2)),
                        *(std::prev(non_bend_idx.end(), 1))};
    double end_angle = VectorAngle(VectorFromSegment(v_pts, first_vec),
                                   VectorFromSegment(v_pts, second_vec));
    if (end_angle > m_max_bend_angle) {
      non_bend_idx.pop_back();
    } else {
      break;
    }
  }

  // if only have first seg + last point
  //  need to possibly worry about eliminating first point
  // Better way - advance along edge by distance of last swath offset
  if (non_bend_idx.size() <= 3 && v_pts.size() > 5) {
    // Try again eliminating the first segment
    path_pts.erase(++path_pts.begin());
    ROS_DEBUG_STREAM("Recursing due to bend at beginning.\n");
    RemoveBends(path_pts);
  } else {
    SelectIndicies(path_pts, non_bend_idx);
  }

}

void PathPlan::RestrictToRegion(std::list<EPoint>& path_points) {
  for(auto point = path_points.begin(); point != path_points.end();) {
    BPoint boost_point(point->x(), point->y());
    if (!boost::geometry::within(boost_point, m_op_region)) {
      point = path_points.erase(point);
    } else {
      ++point;
    }
  }
}

std::pair<bool, bool> PathPlan::ClipToRegion2(std::list<EPoint> &path_pts) {
  if (path_pts.size() < 1) {
    return std::make_pair(false, false);
  }

  bool begin_clipped{false}, end_clipped{false};
  auto outer_ring = m_op_region.outer();
  bool any_within = false;

  // Test if beginning outside the region
  auto first_point = path_pts.begin();
  BPoint last_pt(first_point->x(), first_point->y());

  if (path_pts.size() == 1) {
    if (!boost::geometry::within(last_pt, outer_ring)) {
      path_pts.clear();
    }
    return std::make_pair(false, false);
  }

  BPoint last_outside, last_inside;
  std::list<EPoint>::iterator begin_inside;
  bool found_crossing = false;

  // find last point outside
  for(auto point = std::next(first_point);
        point != path_pts.end(); point++) {
    BPoint this_pt(point->x(), point->y());
    // So there doesn't have to be an extra loop just to check if any are
    // within the geometry, since this only happens once (end of survey).
    bool point_inside = boost::geometry::within(this_pt, outer_ring);
    if (!any_within && point_inside)
      any_within = true;
    // If transistion from out to inside
    if (!boost::geometry::within(last_pt, outer_ring)
          && point_inside) {
      last_outside = last_pt;
      begin_inside = point;
      last_inside = this_pt;
      found_crossing = true;
    }
    last_pt = this_pt;
  }

  if (found_crossing) {
    // Find the intersection point
    #if ((BOOST_VERSION / 100000) >= 1 && ((BOOST_VERSION / 100) % 1000) >= 59)
    //#if !defined(BOOST_NO_CXX11_UNIFIED_INITIALIZATION_SYNTAX) \
    //  && !defined(BOOST_NO_CXX11_HDR_INITIALIZER_LIST)
    BLinestring segment({last_outside, last_inside});
    #else
    BLinestring segment;
    boost::geometry::append(segment, last_outside);
    boost::geometry::append(segment, last_inside);
    #endif
    std::vector<BPoint> intersect_pts;

    boost::geometry::intersection(segment, outer_ring, intersect_pts);
    if (intersect_pts.size() > 0) {
      // Remove the points before the first inside one
      path_pts.erase(first_point, begin_inside);
      // Right now assume there is only one intersect, would have to find
      // furthest along the segment otherwise
      path_pts.push_front(EPointFromBPoint(intersect_pts[0]));
    }
    // it is "clipped" even when on the border, don't want to extend
    begin_clipped = true;
  }


  // All points are outside the region
  if (!any_within) {
    path_pts.clear();
    return std::make_pair(false, false);
  }

  // Run the same thing from the end
  // This can probably be streamlined
  auto end_point = std::prev(path_pts.end());
  last_pt = BPoint(end_point->x(), end_point->y());
  found_crossing = false;

  if (!boost::geometry::within(last_pt, outer_ring)) {
    // find last point outside
    for(auto point = std::prev(end_point);
          point != path_pts.begin(); point--) {
      BPoint this_pt(point->x(), point->y());
      // If transistion from out to inside
      if (!boost::geometry::within(last_pt, outer_ring)
            && boost::geometry::within(this_pt, outer_ring)) {
        last_outside = last_pt;
        begin_inside = point;
        last_inside = this_pt;
        found_crossing = true;
      }
      last_pt = this_pt;
    }
    // Find the intersection point
    if (found_crossing) {
      #if ((BOOST_VERSION / 100000) >= 1 && ((BOOST_VERSION / 100) % 1000) >= 59)
      BLinestring segment({last_outside, last_inside});
      #else
      BLinestring segment;
      boost::geometry::append(segment, last_outside);
      boost::geometry::append(segment, last_inside);
      #endif
      std::vector<BPoint> intersect_pts;
      boost::geometry::intersection(segment, outer_ring, intersect_pts);
      if (intersect_pts.size() > 0) {
        // Remove the points before the first inside one
        path_pts.erase(std::next(begin_inside), path_pts.end());
        path_pts.push_back(EPointFromBPoint(intersect_pts[0]));
      }
      end_clipped = true;
    }
  }

  return std::make_pair(begin_clipped, end_clipped);
}

std::pair<bool, bool> PathPlan::ClipToRegion(std::list<EPoint> &path_pts) {
  if (path_pts.size() < 1) {
    return std::make_pair(false, false);
  }

  ROS_DEBUG_STREAM_COND(DEBUG,"Processing Line");

  bool begin_clipped{false}, end_clipped{false};
  auto outer_ring = m_op_region.outer();
  bool any_within = false;

  // Test if beginning outside the region
  auto first_point = path_pts.begin();
  BPoint last_pt(first_point->x(), first_point->y());

  if (path_pts.size() == 1) {
    if (!boost::geometry::within(last_pt, outer_ring)) {
      path_pts.clear();
    }
    return std::make_pair(false, false);
  }

  std::list<EPoint>::iterator start_erase, end_erase, insert_loc;
  int clip_mode = 0;    // 1 = out to in, 2 = in to out
  int count_in = 0;

  //Within is only true if it is fully contained, not on the border
  bool first_intersects = boost::geometry::intersects(last_pt, outer_ring);
  bool first_within = boost::geometry::within(last_pt, outer_ring);
  if (first_intersects && !first_within)
    begin_clipped = true;
  bool point_inside = first_within || first_intersects;
  if (point_inside) {
    insert_loc = first_point;
    clip_mode = 2;
    any_within = true;
    // Give this a bonus so two inside when starting counts
    count_in = 1;
  } else {
    clip_mode = 1;
    start_erase = first_point;
  }
  auto point = std::next(first_point);

  // Loop through the points
  while (point != path_pts.end()) {
    BPoint this_pt(point->x(), point->y());
    point_inside = boost::geometry::within(this_pt, outer_ring);
    if (!any_within && point_inside)
      any_within = true;

    if (clip_mode == 1 && point_inside) {
      ROS_DEBUG_STREAM("Clip Out to In, (" << point->x() << ", " << point->y() << ")");
      end_erase = point;
      insert_loc = path_pts.erase(start_erase, end_erase);
      auto intersect_pts = SegmentRingIntersect(last_pt, this_pt, outer_ring);
      if (intersect_pts.size() > 0) {
        insert_loc = path_pts.insert(insert_loc, EPointFromBPoint(intersect_pts[0]));
        count_in = 0;
      } else if (boost::geometry::intersects(last_pt, outer_ring)){
        // Last point is on the edge, re-insert it
        insert_loc = path_pts.insert(insert_loc, EPointFromBPoint(last_pt));
        count_in = 0;
      } else {
        // If no inserting, we skip this point for next test
        count_in = 1;
      }
      any_within = true;
      begin_clipped = true;
      clip_mode = 2;
      // Insert loc is the intersection, next pt is inside (but may not have
      // inserted).  So this will duplicate a point in tests
      point = std::next(insert_loc);
    } else if (clip_mode == 2 && !point_inside) {
      auto insert_out = point;
      ROS_DEBUG_STREAM("Clip In to Out, " << count_in << "pts (" << point->x() << ", " << point->y() << ")");
      if (count_in > 2) {
        start_erase = point;
        auto intersect_pts = SegmentRingIntersect(last_pt, this_pt, outer_ring);
        if (intersect_pts.size() > 0) {
          insert_out = path_pts.insert(point,
              EPointFromBPoint(intersect_pts[intersect_pts.size()-1]));
          start_erase = std::next(insert_out);
        }
      } else {
        start_erase = insert_loc;
      }
      clip_mode = 1;
      end_clipped = true;
      point = std::next(insert_out);
    } else if (clip_mode == 2) {
      count_in++;
      point = std::next(point);
    } else {
      point = std::next(point);
    }

    //prev_point_inside = point_inside;
    last_pt = this_pt;
  }

  //reached the end without clipping
  if (clip_mode == 1 && !point_inside) {
    path_pts.erase(start_erase, path_pts.end());
    end_clipped = true;
  }

  // All points are outside the region
  if (!any_within || path_pts.size() == 0) {
    path_pts.clear();
    return std::make_pair(false, false);
  }

  return std::make_pair(begin_clipped, end_clipped);
}

std::vector<BPoint> PathPlan::SegmentRingIntersect(BPoint seg_pt1, BPoint seg_pt2, BRing ring) {
  #if ((BOOST_VERSION / 100000) >= 1 && ((BOOST_VERSION / 100) % 1000) >= 59)
  BLinestring segment({seg_pt1, seg_pt2});
  #else
  BLinestring segment;
  boost::geometry::append(segment, seg_pt1);
  boost::geometry::append(segment, seg_pt2);
  #endif
  std::vector<BPoint> intersect_pts;

  boost::geometry::intersection(segment, ring, intersect_pts);
  return intersect_pts;
}

void PathPlan::ExtendToEdge(std::list<EPoint> &path_points, bool begin) {
  // Start of the path
  EPoint starting_pt;
  EPoint extend_vec;

  if (begin) {
    auto first_pt = std::next(path_points.begin()); // [1]
    auto second_pt = path_points.begin();           // [0]
    starting_pt = *second_pt;
    extend_vec = *second_pt - *first_pt;
  } else {
    auto second_pt = std::prev(path_points.end());  // [-1]
    auto first_pt = std::prev(second_pt);           // [-2]
    starting_pt = *second_pt;
    extend_vec = *second_pt - *first_pt;
  }
  std::pair<double, EPoint> intersection = FindNearestIntersect(extend_vec,
    starting_pt, m_op_region);
  if(std::isnan(intersection.first))
      return;

  double extend_max = 15 * m_last_line.IntervalDist();
  if (intersection.first < extend_max) {
    if (begin) {
      path_points.push_front(intersection.second);
    } else {
      path_points.push_back(intersection.second);
    }
  } else {
    ROS_DEBUG_STREAM_COND(DEBUG,"Reached edge extension max.\n");
  }
}

std::pair<double, EPoint> PathPlan::FindNearestIntersect(EPoint ray_vector,
  EPoint start_pt, BPolygon& poly) {

  //The ring is a vector of points_xy<double>
  //This should make it closed by default definition
  boost::geometry::correct(poly);
  auto ext_ring = poly.outer();

  // These store all the found intersections
  std::vector<double> intersect_dist;
  std::vector<EPoint> intersect_pts;

  for (auto poly_vertex = ext_ring.begin();
       poly_vertex != std::prev(ext_ring.end()); ++poly_vertex) {
    auto next_vertex = std::next(poly_vertex);
    std::pair<BPoint, BPoint> poly_seg = std::make_pair(*poly_vertex, *next_vertex);
    std::pair<bool, EPoint> intersection = IntersectRaySegment(ray_vector,
      start_pt, poly_seg);
    // If an intersection exists
    if (intersection.first) {
      intersect_dist.push_back((intersection.second - start_pt).norm());
      intersect_pts.push_back(intersection.second);
    }
  }

  ROS_WARN_STREAM("PathPlan::FindNearestIntersect intersect_dist.size(): " << intersect_dist.size());
  if(intersect_dist.empty())
      return std::make_pair(NAN,EPoint());
  
  // What should be done when equadistant from two?
  auto min_index = std::min_element(intersect_dist.begin(), intersect_dist.end())
    - intersect_dist.begin();
  return std::make_pair(intersect_dist[min_index], intersect_pts[min_index]);
}

std::pair<bool, EPoint> PathPlan::IntersectRaySegment(EPoint ray_vector, EPoint start_pt,
  std::pair<BPoint, BPoint> segment) {
  EPoint seg_start = EPointFromBPoint(segment.first);
  EPoint seg_vector = EPointFromBPoint(segment.second) -
    EPointFromBPoint(segment.first);
  double rxs = Cross2d(ray_vector, seg_vector);
  if (rxs != 0) {
    // EPoint t = seg_start - start_pt;
    // EPoint u{t};
    double t = Cross2d(seg_start - start_pt, seg_vector) / rxs;
    double u = Cross2d(seg_start - start_pt, ray_vector) / rxs;
    // EPoint u = seg_start - start_pt;
    // u = u.cross(ray_vector) / rxs;
    if (t >= 0 && u >= 0 && u <= 1) {
      return std::make_pair(true, start_pt + (t * ray_vector));
    }
  }
  return std::make_pair(false, EPoint(0,0));
}

double PathPlan::Cross2d(EPoint vec1, EPoint vec2) {
  // This is the determinant of ||v1||v2||
  return (vec1[0] * vec2[1]) - (vec2[0] * vec1[1]);
}

EPoint PathPlan::EPointFromBPoint(BPoint boost_point) {
  return EPoint(boost_point.x(), boost_point.y());
}

bool PathPlan::Intersect(EPoint A, EPoint B, EPoint C, EPoint D) {
  return CCW(A, C, D) != CCW(B, C, D) && CCW(A, B, C) != CCW(A, B, D);
}

bool PathPlan::CCW(EPoint A, EPoint B, EPoint C) {
  return (C.y() - A.y()) * (B.x() - A.x()) > (B.y() - A.y()) * (C.x() - A.x());
}

double PathPlan::VectorAngle(EPoint vector1, EPoint vector2) {
  double dotp = vector1.dot(vector2);
  double mags = vector1.norm() * vector2.norm();

  if (mags == 0)
    return 0;
  return acos(dotp/mags) * 180/M_PI;
}

EPoint PathPlan::VectorFromSegment(const std::vector<EPoint>& points, SegIndex segment) {
  EPoint vector;
  try {
    vector = points[segment[1]] - points[segment[0]];
  } catch (std::exception e) {
    vector = EPoint(NAN, NAN);
  }
  return vector;
}

// Eigen::Vector2d PathPlan::UnitVector(Eigen::Vector2d vector_in) {
//   double mag = vector_in.norm()
// }
