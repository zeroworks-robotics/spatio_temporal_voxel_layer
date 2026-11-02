// Door nodes of the robot's map, see door_zones.hpp.

#include "spatio_temporal_voxel_layer/door_zones.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
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

/*****************************************************************************/
int PairStandbyNodes(
  const std::string & file, const std::string & type, double max_dist, double max_lateral,
  std::vector<Door> & doors, std::string & err)
/*****************************************************************************/
{
  std::ifstream in(file);
  if (!in) {
    err = "cannot open " + file;
    return -1;
  }
  nlohmann::json map;
  try {
    in >> map;
  } catch (const nlohmann::json::exception & e) {
    err = file + " does not parse: " + e.what();
    return -1;
  }
  if (!map.is_object() || !map.contains("nodes") || !map["nodes"].is_array()) {
    err = file + " has no nodes array";
    return -1;
  }
  std::vector<std::pair<double, double>> standby;
  for (const auto & n : map["nodes"]) {
    if (n.is_object() && n.contains("Place") && n["Place"].is_object() && n["Place"].contains("type") &&
      n["Place"]["type"].is_string() && n["Place"]["type"].get<std::string>() == type &&
      n.contains("x") && n["x"].is_number() && n.contains("y") && n["y"].is_number())
    {
      standby.emplace_back(n["x"].get<double>(), n["y"].get<double>());
    }
  }
  int paired = 0;
  for (auto & d : doors) {
    double lo = -std::numeric_limits<double>::infinity(), hi = std::numeric_limits<double>::infinity();
    for (const auto & p : standby) {
      const double dx = p.first - d.x, dy = p.second - d.y;
      const double s = dx * d.ux + dy * d.uy, l = -dx * d.uy + dy * d.ux;
      if (std::hypot(dx, dy) > max_dist || std::fabs(l) > max_lateral) {
        continue;
      }
      if (s < 0.0 && s > lo) {
        lo = s;
      } else if (s > 0.0 && s < hi) {
        hi = s;
      }
    }
    d.triggered = std::isfinite(lo) && std::isfinite(hi);
    if (d.triggered) {
      d.trig_lo = lo;
      d.trig_hi = hi;
      ++paired;
    }
  }
  return paired;
}

}  // namespace spatio_temporal_voxel_layer
