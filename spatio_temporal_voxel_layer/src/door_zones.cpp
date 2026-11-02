// Door nodes of the robot's map, see door_zones.hpp.

#include "spatio_temporal_voxel_layer/door_zones.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <string>
#include <vector>

#include "nlohmann/json.hpp"

namespace spatio_temporal_voxel_layer
{

/*****************************************************************************/
bool LoadDoorNodes(
  const std::string & file, const std::vector<std::string> & types, int floor,
  std::vector<Door> & out, std::string & err, size_t * skipped)
/*****************************************************************************/
{
  out.clear();
  if (skipped != nullptr) {
    *skipped = 0;
  }
  std::ifstream in(file);
  if (!in) {
    err = "cannot open " + file;
    return false;
  }
  nlohmann::json map;
  try {
    in >> map;
  } catch (const nlohmann::json::exception & e) {
    err = file + " does not parse: " + e.what();
    return false;
  }
  if (!map.is_object() || !map.contains("nodes") || !map["nodes"].is_array()) {
    err = file + " has no nodes array";
    return false;
  }
  const auto & nodes = map["nodes"];
  auto number = [](const nlohmann::json & o, const char * key, double & v) {
      if (!o.is_object() || !o.contains(key) || !o[key].is_number()) {
        return false;
      }
      v = o[key].get<double>();
      return true;
    };
  for (size_t idx = 0; idx < nodes.size(); ++idx) {
    const auto & n = nodes[idx];
    if (!n.is_object() || !n.contains("Place") || !n["Place"].is_object() ||
      !n["Place"].contains("type") || !n["Place"]["type"].is_string())
    {
      continue;
    }
    const std::string type = n["Place"]["type"].get<std::string>();
    if (std::find(types.begin(), types.end(), type) == types.end()) {
      continue;
    }
    double node_floor = -1.0, x, y, th = 0.0;
    number(n, "floor", node_floor);
    if (floor >= 0 && static_cast<int>(node_floor) != floor) {
      continue;
    }
    if (!number(n, "x", x) || !number(n, "y", y)) {
      if (skipped != nullptr) {
        ++*skipped;
      }
      continue;
    }
    // direction of travel: from the first to the second neighbour, else the node heading
    number(n, "th", th);
    double ux = std::cos(th * M_PI / 180.0), uy = std::sin(th * M_PI / 180.0);
    if (n.contains("edge_to") && n["edge_to"].is_array() && n["edge_to"].size() >= 2 &&
      n["edge_to"][0].is_number_unsigned() && n["edge_to"][1].is_number_unsigned())
    {
      const size_t ia = n["edge_to"][0].get<size_t>(), ib = n["edge_to"][1].get<size_t>();
      double ax, ay, bx, by;
      if (ia < nodes.size() && ib < nodes.size() && number(nodes[ia], "x", ax) && number(nodes[ia], "y", ay) &&
        number(nodes[ib], "x", bx) && number(nodes[ib], "y", by))
      {
        ux = bx - ax;
        uy = by - ay;
      }
    }
    const double len = std::hypot(ux, uy);
    if (len < 1e-6) {
      continue;
    }
    out.push_back({x, y, ux / len, uy / len});
  }
  return true;
}

}  // namespace spatio_temporal_voxel_layer
