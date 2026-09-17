#pragma once
// A floor height the cameras agree on, remembered in odom and shared between buffers.
//
// Why this exists. The column walk starts by finding a sample near the plane the robot
// stands on, and then follows the floor outward from there. That fails in exactly the
// place this layer is meant to help. Approaching a ramp, the nearest floor the FRONT
// camera can see is 1.52 m out; stand 1.5 m back from the ramp and every sample it has is
// already on the slope, with no level ground anywhere in the column to measure from. The
// walk cannot know the surface is climbing, predicts the flat floor it last had evidence
// for, and marks the whole ramp as obstacle. Measured on the synthetic course, that peaks
// at ~780 lethal cells about 1.5 m short of the ramp and falls to zero once the robot is
// actually on it.
//
// The information is not missing, it is just a second old: the robot drove over that level
// floor moments ago, and fdown watched it pass 0.34 m beneath. A grid in odom keeps it.
//
// WHY odom AND NOT base_link. A base_link grid is invalidated by the robot's own motion --
// an earlier attempt relayed floor heights between cameras in base_link and had to expire
// them in 0.5 s, which left the lookup answering only 62% of the time and changed nothing
// downstream. In odom an observation stays where the world put it and accumulates.
//
// WHY CELLS EXPIRE ANYWAY. odom is dead reckoning. Measured on a real recorded drive,
// odom->base_link z drifts from -1.27 m to -5.61 m over 121 s -- monotonically, on a flat
// indoor floor, so it is integration bias rather than real motion. Over the 2.5 s a cell
// is allowed to live that is under 0.09 m, which the tolerance absorbs; over a minute it
// would be meters and the grid would describe a floor that is not there.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <utility>
#include <vector>

namespace ground_seg
{

class GroundReference
{
public:
  // 12 cm cells over a 24 m window: fine enough that floor height is near-constant within
  // a cell at the ranges that matter, coarse enough that one camera frame fills the cells
  // it covers densely rather than speckling them.
  static constexpr float kCell = 0.12f;
  static constexpr int kSide = 200;            // 24 m
  static constexpr double kMaxAge = 2.5;       // see the odom-drift note above

  // Move the window to follow the robot. Only cells that leave are cleared, so the floor
  // behind stays remembered while the robot drives forward.
  void Recenter(const double & cx, const double & cy)
  {
    std::lock_guard<std::mutex> lk(_m);
    if (_cells.empty()) {
      _cells.assign(static_cast<std::size_t>(kSide) * kSide, Cell());
      _ox = cx - kSide * kCell / 2;
      _oy = cy - kSide * kCell / 2;
      return;
    }
    const int dx = static_cast<int>(std::floor((cx - kSide * kCell / 2 - _ox) / kCell));
    const int dy = static_cast<int>(std::floor((cy - kSide * kCell / 2 - _oy) / kCell));
    if (dx == 0 && dy == 0) {
      return;
    }
    if (std::abs(dx) >= kSide || std::abs(dy) >= kSide) {
      _cells.assign(static_cast<std::size_t>(kSide) * kSide, Cell());
    } else {
      std::vector<Cell> next(static_cast<std::size_t>(kSide) * kSide);
      for (int y = 0; y < kSide; ++y) {
        const int sy = y + dy;
        if (sy < 0 || sy >= kSide) {
          continue;
        }
        for (int x = 0; x < kSide; ++x) {
          const int sx = x + dx;
          if (sx < 0 || sx >= kSide) {
            continue;
          }
          next[Idx(x, y)] = _cells[Idx(sx, sy)];
        }
      }
      _cells.swap(next);
    }
    _ox += dx * kCell;
    _oy += dy * kCell;
  }

  // Record ground observations. `xy` and `z` are in the grid's frame (odom).
  //
  // A cell keeps the LOWEST height it has seen recently, not a running mean. Ground is the
  // bottom of what a camera sees: a mean is pulled up by anything standing on the floor
  // that slipped through classification, and once the stored height rises the walk seeded
  // from it treats real floor as an obstacle. Taking the minimum fails the safe way --
  // toward calling something floor-height that is slightly above the floor.
  void Publish(
    const std::vector<std::pair<float, float>> & xy, const std::vector<float> & z,
    const double & stamp)
  {
    std::lock_guard<std::mutex> lk(_m);
    if (_cells.empty()) {
      return;
    }
    for (std::size_t i = 0; i < xy.size() && i < z.size(); ++i) {
      int cx, cy;
      if (!CellOf(xy[i].first, xy[i].second, cx, cy)) {
        continue;
      }
      Cell & c = _cells[Idx(cx, cy)];
      if (stamp - c.stamp > kMaxAge) {
        c.z = z[i];                    // stale: replace rather than min against old floor
      } else {
        c.z = std::min(c.z, z[i]);
      }
      c.stamp = stamp;
    }
  }

  // Floor height at (x, y), from this cell or the nearest neighbour that has one. Returns
  // false when nothing recent is stored nearby, which leaves the caller to its own rule --
  // the grid is an aid, never a requirement.
  bool Lookup(const float & x, const float & y, const double & now, float & z_out) const
  {
    std::lock_guard<std::mutex> lk(_m);
    if (_cells.empty()) {
      return false;
    }
    int cx, cy;
    if (!CellOf(x, y, cx, cy)) {
      return false;
    }
    // One ring of neighbours: a cell can be empty because the camera's rays fell either
    // side of it, and the floor 12 cm away is the same floor.
    bool found = false;
    float best = 0.0f;
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        const int nx = cx + dx, ny = cy + dy;
        if (nx < 0 || nx >= kSide || ny < 0 || ny >= kSide) {
          continue;
        }
        const Cell & c = _cells[Idx(nx, ny)];
        if (c.stamp < 0.0 || now - c.stamp > kMaxAge) {
          continue;
        }
        if (!found || c.z < best) {
          found = true;
          best = c.z;
        }
      }
    }
    if (found) {
      z_out = best;
    }
    return found;
  }

  // Floor heights along a bearing, as (range, z) pairs from `r0` outward while the grid
  // still has them. This is the call the column walk actually needs.
  //
  // A single height is not enough, and measuring that was the whole lesson. Seeding front
  // from the grid at a ramp works perfectly -- the lookup answers 100% of the time -- and
  // changes nothing, because after the seed the walk still has to learn which way the
  // floor is going, and standing 1.5 m short of a ramp it has no level sample anywhere in
  // its view to learn from. Handing it a profile hands it the gradient too: the run starts
  // already in motion rather than from a single point with no direction.
  std::vector<std::pair<float, float>> Profile(
    const float & ox, const float & oy, const float & dx, const float & dy,
    const float & r0, const float & r1, const double & now) const
  {
    std::vector<std::pair<float, float>> out;
    const float len = std::sqrt(dx * dx + dy * dy);
    if (len < 1e-6f) {
      return out;
    }
    const float ux = dx / len, uy = dy / len;
    // One sample per cell: finer only repeats a cell, coarser skips the slope between two.
    for (float r = r0; r <= r1; r += kCell) {
      float z;
      if (Lookup(ox + ux * r, oy + uy * r, now, z)) {
        out.emplace_back(r, z);
      }
    }
    return out;
  }

  void Clear()
  {
    std::lock_guard<std::mutex> lk(_m);
    _cells.clear();
  }

  // Cells currently holding a recent observation -- for logging and tests.
  int Populated(const double & now) const
  {
    std::lock_guard<std::mutex> lk(_m);
    int n = 0;
    for (const Cell & c : _cells) {
      if (c.stamp >= 0.0 && now - c.stamp <= kMaxAge) {
        ++n;
      }
    }
    return n;
  }

private:
  struct Cell
  {
    float z = 0.0f;
    double stamp = -1.0;
  };

  static std::size_t Idx(const int & x, const int & y)
  {
    return static_cast<std::size_t>(y) * kSide + static_cast<std::size_t>(x);
  }

  bool CellOf(const float & x, const float & y, int & cx, int & cy) const
  {
    cx = static_cast<int>(std::floor((x - _ox) / kCell));
    cy = static_cast<int>(std::floor((y - _oy) / kCell));
    return cx >= 0 && cx < kSide && cy >= 0 && cy < kSide;
  }

  mutable std::mutex _m;
  std::vector<Cell> _cells;
  double _ox = 0.0, _oy = 0.0;
};

}  // namespace ground_seg
