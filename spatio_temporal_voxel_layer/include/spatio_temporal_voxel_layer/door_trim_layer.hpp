// Door trim layer: gives mapped doors back the width the sensors and the grid take from them.

#ifndef SPATIO_TEMPORAL_VOXEL_LAYER__DOOR_TRIM_LAYER_HPP_
#define SPATIO_TEMPORAL_VOXEL_LAYER__DOOR_TRIM_LAYER_HPP_

#include <string>
#include <vector>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/layer.hpp"

namespace spatio_temporal_voxel_layer
{

/**
 * \brief Trims the innermost cells of the walls beside the doors of the robot's map.
 *
 * A door frame reaches the costmap about one cell inside the real opening: the obstacle sources
 * leave a little spread at the frame edge, and a cell that holds any part of the frame is filled
 * whole. At every map door (a node whose Place.type is listed in place_types), each row across
 * the opening in a zone around the node is searched for the free run through the node; a run on
 * either side of it at least min_wall_run long loses trim metres on the opening side. A row with
 * anything on the node itself (a closed door, a person on the path) stays whole. The sensors see
 * a jamb only as its face, a band of cells much like a person standing at the edge of the zone,
 * so such a person can lose one trimmed cell too; require_zone_edge limits the trim to runs that
 * reach the zone edge, which is safer but keeps most of the narrowing on real doors.
 * Load it after the obstacle layers and before any inflation layer.
 */
class DoorTrimLayer : public nav2_costmap_2d::Layer
{
public:
  DoorTrimLayer() = default;

  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y,
    double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j) override;
  void reset() override {}
  bool isClearable() override {return false;}

private:
  // a door node: position and the unit direction of travel through it, in the map frame
  struct Door
  {
    double x, y, ux, uy;
  };

  bool LoadDoors(const std::string & file, const std::vector<std::string> & types, int floor);
  void TrimDoor(
    nav2_costmap_2d::Costmap2D & grid, double px, double py, double ux, double uy,
    int min_i, int min_j, int max_i, int max_j);

  std::vector<Door> _doors;
  std::string _map_frame;
  double _trim{0.02}, _half_depth{0.8}, _half_width{1.0};
  double _min_wall_run{0.10}, _bridge_gap{0.04}, _max_tf_age{2.0};
  bool _require_zone_edge{false};
};

}  // namespace spatio_temporal_voxel_layer

#endif  // SPATIO_TEMPORAL_VOXEL_LAYER__DOOR_TRIM_LAYER_HPP_
