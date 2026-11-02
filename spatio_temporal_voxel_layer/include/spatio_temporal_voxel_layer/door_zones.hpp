// Door nodes of the robot's map, shared by the layers that treat doors specially.

#ifndef SPATIO_TEMPORAL_VOXEL_LAYER__DOOR_ZONES_HPP_
#define SPATIO_TEMPORAL_VOXEL_LAYER__DOOR_ZONES_HPP_

#include <string>
#include <vector>

namespace spatio_temporal_voxel_layer
{

// a door node: position and the unit direction of travel through it, in the map frame
struct Door
{
  double x, y, ux, uy;
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

}  // namespace spatio_temporal_voxel_layer

#endif  // SPATIO_TEMPORAL_VOXEL_LAYER__DOOR_ZONES_HPP_
