// Door nodes of the robot's map, shared by the layers that treat doors specially.

#ifndef SPATIO_TEMPORAL_VOXEL_LAYER__DOOR_ZONES_HPP_
#define SPATIO_TEMPORAL_VOXEL_LAYER__DOOR_ZONES_HPP_

#include <string>
#include <vector>

namespace spatio_temporal_voxel_layer
{

// a door node: position and the unit direction of travel through it, in the map frame; with a
// trigger, the stations (along u from the node) of the standby nodes before and after it
struct Door
{
  double x, y, ux, uy;
  bool triggered{false};
  double trig_lo{0.0}, trig_hi{0.0};
};

/**
 * \brief Reads the door nodes of a cona map.json.
 *
 * A door is a node whose Place.type is one of `types` (and whose floor is `floor` when floor >= 0).
 * The direction of travel runs from the first to the second neighbour in edge_to, else along the
 * node heading th (degrees). Door nodes without a position are skipped and counted in `skipped`.
 * Returns false with `err` set when the file is missing, does not parse or has no nodes array;
 * true otherwise, also when no node matched (out is then empty).
 */
bool LoadDoorNodes(
  const std::string & file, const std::vector<std::string> & types, int floor,
  std::vector<Door> & out, std::string & err, size_t * skipped = nullptr);

/**
 * \brief Pairs each door with the standby nodes the robot waits at before and after it.
 *
 * A standby node is one whose Place.type is `type` (any floor: the outside one sits on another
 * floor number at the same height). It belongs to a door when it lies on the door's travel axis
 * (within max_lateral across it) and within max_dist of the node; the nearest one on each side
 * is taken. A door with both gets triggered = true and trig_lo < 0 < trig_hi. Returns the number
 * of doors paired, or -1 with `err` set when the file cannot be read.
 */
int PairStandbyNodes(
  const std::string & file, const std::string & type, double max_dist, double max_lateral,
  std::vector<Door> & doors, std::string & err);

}  // namespace spatio_temporal_voxel_layer

#endif  // SPATIO_TEMPORAL_VOXEL_LAYER__DOOR_ZONES_HPP_
