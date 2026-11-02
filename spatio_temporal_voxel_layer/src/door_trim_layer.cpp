// Door trim layer, see door_trim_layer.hpp.

#include "spatio_temporal_voxel_layer/door_trim_layer.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/exceptions.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace spatio_temporal_voxel_layer
{

/*****************************************************************************/
void DoorTrimLayer::onInitialize()
/*****************************************************************************/
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }
  std::string map_file;
  std::vector<std::string> types{"gate_private", "gate_public"};
  int floor = -1;
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("map_file", rclcpp::ParameterValue(std::string("")));
  declareParameter("map_frame", rclcpp::ParameterValue(std::string("map")));
  declareParameter("place_types", rclcpp::ParameterValue(types));
  declareParameter("floor", rclcpp::ParameterValue(-1));
  declareParameter("trim", rclcpp::ParameterValue(0.02));
  declareParameter("zone_half_depth", rclcpp::ParameterValue(0.8));
  declareParameter("zone_half_width", rclcpp::ParameterValue(1.0));
  declareParameter("min_wall_run", rclcpp::ParameterValue(0.10));
  declareParameter("bridge_gap", rclcpp::ParameterValue(0.04));
  declareParameter("max_tf_age", rclcpp::ParameterValue(2.0));
  declareParameter("require_zone_edge", rclcpp::ParameterValue(false));
  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "map_file", map_file);
  node->get_parameter(name_ + "." + "map_frame", _map_frame);
  node->get_parameter(name_ + "." + "place_types", types);
  node->get_parameter(name_ + "." + "floor", floor);
  node->get_parameter(name_ + "." + "trim", _trim);
  node->get_parameter(name_ + "." + "zone_half_depth", _half_depth);
  node->get_parameter(name_ + "." + "zone_half_width", _half_width);
  node->get_parameter(name_ + "." + "min_wall_run", _min_wall_run);
  node->get_parameter(name_ + "." + "bridge_gap", _bridge_gap);
  node->get_parameter(name_ + "." + "max_tf_age", _max_tf_age);
  node->get_parameter(name_ + "." + "require_zone_edge", _require_zone_edge);

  if (!LoadDoors(map_file, types, floor)) {
    RCLCPP_ERROR(logger_, "%s: no doors loaded from '%s', layer does nothing", name_.c_str(), map_file.c_str());
    enabled_ = false;
    return;
  }
  RCLCPP_INFO(
    logger_, "%s: %zu doors from %s, trim %.3f m, zone %.2f x %.2f m", name_.c_str(), _doors.size(),
    map_file.c_str(), _trim, 2.0 * _half_width, 2.0 * _half_depth);
  current_ = true;
}

/*****************************************************************************/
bool DoorTrimLayer::LoadDoors(
  const std::string & file, const std::vector<std::string> & types, int floor)
/*****************************************************************************/
{
  // the loader is shared with the voxel layer's door trust rule (door_zones.hpp)
  if (file.empty()) {
    return false;   // the caller reports the empty map_file
  }
  std::string err;
  size_t skipped = 0;
  if (!LoadDoorNodes(file, types, floor, _doors, err, &skipped)) {
    RCLCPP_ERROR(logger_, "%s: %s", name_.c_str(), err.c_str());
    return false;
  }
  if (skipped > 0) {
    RCLCPP_WARN(logger_, "%s: %zu door nodes without x/y skipped", name_.c_str(), skipped);
  }
  return !_doors.empty();
}

/*****************************************************************************/
void DoorTrimLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double * /*min_x*/,
  double * /*min_y*/, double * /*max_x*/, double * /*max_y*/)
/*****************************************************************************/
{
  // only trims what the obstacle layers drew; adds no area of its own
}

/*****************************************************************************/
void DoorTrimLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j)
/*****************************************************************************/
{
  if (!enabled_ || _doors.empty()) {
    return;
  }
  geometry_msgs::msg::TransformStamped map_to_grid;
  try {
    map_to_grid = tf_->lookupTransform(
      layered_costmap_->getGlobalFrameID(), _map_frame, tf2::TimePointZero);
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000, "%s: no %s pose, doors not trimmed: %s",
      name_.c_str(), _map_frame.c_str(), e.what());
    return;
  }
  const rclcpp::Time stamp(map_to_grid.header.stamp, clock_->get_clock_type());
  if (stamp.nanoseconds() > 0 && (clock_->now() - stamp).seconds() > _max_tf_age) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000, "%s: %s pose is %.1f s old, doors not trimmed",
      name_.c_str(), _map_frame.c_str(), (clock_->now() - stamp).seconds());
    return;
  }
  const auto & t = map_to_grid.transform;
  const double yaw = tf2::getYaw(t.rotation), c = std::cos(yaw), s = std::sin(yaw);
  const double reach = std::hypot(_half_width, _half_depth);
  const double ox = master_grid.getOriginX(), oy = master_grid.getOriginY();
  const double sx = master_grid.getSizeInMetersX(), sy = master_grid.getSizeInMetersY();
  for (const auto & d : _doors) {
    const double px = t.translation.x + c * d.x - s * d.y;
    const double py = t.translation.y + s * d.x + c * d.y;
    if (px + reach < ox || px - reach > ox + sx || py + reach < oy || py - reach > oy + sy) {
      continue;
    }
    TrimDoor(master_grid, px, py, c * d.ux - s * d.uy, s * d.ux + c * d.uy, min_i, min_j, max_i, max_j);
  }
}

/*****************************************************************************/
void DoorTrimLayer::TrimDoor(
  nav2_costmap_2d::Costmap2D & grid, double px, double py, double ux, double uy,
  int min_i, int min_j, int max_i, int max_j)
/*****************************************************************************/
{
  // door coordinates: l across the opening (along n), s along the travel direction u
  const double nx = -uy, ny = ux;
  const double res = grid.getResolution();
  // bins across the opening are 0..nl, each res wide; nl is the last one (the zone edge)
  const int nl = static_cast<int>(std::lround(2.0 * _half_width / res)) - 1;
  const int ns = static_cast<int>(std::floor(2.0 * _half_depth / res)) + 1;
  const int k = static_cast<int>(std::lround(_trim / res));
  const int kmin = static_cast<int>(std::lround(_min_wall_run / res));
  const int bridge = static_cast<int>(std::lround(_bridge_gap / res));
  const int c0 = static_cast<int>(std::lround(_half_width / res));
  if (k <= 0) {
    return;
  }

  // cells of the zone's bounding box
  const double reach = std::hypot(_half_width, _half_depth);
  int i0, j0, i1, j1;
  grid.worldToMapEnforceBounds(px - reach, py - reach, i0, j0);
  grid.worldToMapEnforceBounds(px + reach, py + reach, i1, j1);
  auto bin = [&](int i, int j, int & li, int & si) {
      double wx, wy;
      grid.mapToWorld(i, j, wx, wy);
      const double l = (wx - px) * nx + (wy - py) * ny;
      const double sv = (wx - px) * ux + (wy - py) * uy;
      if (std::fabs(sv) > _half_depth || std::fabs(l) > _half_width) {
        return false;
      }
      li = std::clamp(static_cast<int>(std::floor((l + _half_width) / res)), 0, nl);
      si = static_cast<int>(std::floor((sv + _half_depth) / res));
      return si >= 0 && si < ns;
    };
  auto occupied = [](unsigned char cost) {
      return cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE && cost != nav2_costmap_2d::NO_INFORMATION;
    };

  // occupancy per (row across the opening, bin)
  std::vector<uint8_t> occ(static_cast<size_t>(ns) * (nl + 1), 0);
  for (int j = j0; j <= j1; ++j) {
    for (int i = i0; i <= i1; ++i) {
      int li, si;
      if (occupied(grid.getCost(i, j)) && bin(i, j, li, si)) {
        occ[static_cast<size_t>(si) * (nl + 1) + li] = 1;
      }
    }
  }

  // per row: the free run around the node, and the wall bins to trim on each side of it
  std::vector<uint8_t> cut(occ.size(), 0);
  bool any_cut = false;
  for (int si = 0; si < ns; ++si) {
    uint8_t * row = occ.data() + static_cast<size_t>(si) * (nl + 1);
    // the row before bridging: only a face that is really there may lose its inner bins
    const std::vector<uint8_t> orig(row, row + nl + 1);
    // a frame's ragged edge reads as one run: bridge free gaps up to bridge bins
    int prev = -1;
    for (int li = 0; li <= nl; ++li) {
      if (!row[li]) {
        continue;
      }
      if (prev >= 0 && li - prev > 1 && li - prev <= bridge + 1) {
        std::fill(row + prev, row + li, 1);
      }
      prev = li;
    }
    // something on the door node (a closed door, a person in the opening): leave the row whole
    if (row[c0]) {
      continue;
    }
    int a = c0, b = c0;
    while (a - 1 >= 0 && !row[a - 1]) {
      --a;
    }
    while (b + 1 <= nl && !row[b + 1]) {
      ++b;
    }
    uint8_t * cut_row = cut.data() + static_cast<size_t>(si) * (nl + 1);
    // Trim a side only when the unbridged segment at its inner face is wider than the trim, so
    // at least one bin of it stays. A thin object (a pole, a door leaf) within the bridge gap
    // of the jamb would otherwise be joined to the jamb and become the bin that is cut away.
    auto face_kept = [&](int edge, int step) {
        int seg = 0;
        for (int li = edge; li >= 0 && li <= nl && orig[li]; li += step) {
          ++seg;
        }
        return seg > k;
      };
    int run = 0;
    while (a - 1 - run >= 0 && row[a - 1 - run]) {
      ++run;
    }
    if (run >= kmin && (!_require_zone_edge || a - run == 0) && face_kept(a - 1, -1)) {
      for (int li = std::max(a - k, 0); li < a; ++li) {
        cut_row[li] = 1;
        any_cut = true;
      }
    }
    run = 0;
    while (b + 1 + run <= nl && row[b + 1 + run]) {
      ++run;
    }
    if (run >= kmin && (!_require_zone_edge || b + run == nl) && face_kept(b + 1, 1)) {
      for (int li = b + 1; li <= std::min(b + k, nl); ++li) {
        cut_row[li] = 1;
        any_cut = true;
      }
    }
  }
  if (!any_cut) {
    return;
  }

  // free the occupied cells of the trimmed bins, inside this update's window only
  for (int j = std::max(j0, min_j); j <= std::min(j1, max_j - 1); ++j) {
    for (int i = std::max(i0, min_i); i <= std::min(i1, max_i - 1); ++i) {
      int li, si;
      if (occupied(grid.getCost(i, j)) && bin(i, j, li, si) &&
        cut[static_cast<size_t>(si) * (nl + 1) + li])
      {
        grid.setCost(i, j, nav2_costmap_2d::FREE_SPACE);
      }
    }
  }
}

}  // namespace spatio_temporal_voxel_layer

PLUGINLIB_EXPORT_CLASS(spatio_temporal_voxel_layer::DoorTrimLayer, nav2_costmap_2d::Layer)
