/*********************************************************************
 *
 * Software License Agreement
 *
 *  Copyright (c) 2018, Simbe Robotics, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Simbe Robotics, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Steve Macenski (steven.macenski@simberobotics.com)
 *********************************************************************/

#include <algorithm>
#include <memory>
#include <unordered_map>
#include <string>
#include <vector>

#include "spatio_temporal_voxel_layer/spatio_temporal_voxel_grid.hpp"

namespace volume_grid
{

/*****************************************************************************/
SpatioTemporalVoxelGrid::SpatioTemporalVoxelGrid(
  rclcpp::Clock::SharedPtr clock,
  const float & voxel_size, const double & background_value,
  const int & decay_model, const double & voxel_decay, const bool & pub_voxels)
: _clock(clock), _decay_model(decay_model), _background_value(background_value),
  _voxel_size(voxel_size), _voxel_decay(voxel_decay), _pub_voxels(pub_voxels),
  _grid_points(std::make_unique<std::vector<geometry_msgs::msg::Point32>>()),
  _costmap_origin_x(0.), _costmap_origin_y(0.), _costmap_resolution(0.),
  _costmap_size_x(0), _costmap_size_y(0)
/*****************************************************************************/
{
  this->InitializeGrid();
}

/*****************************************************************************/
SpatioTemporalVoxelGrid::~SpatioTemporalVoxelGrid(void)
/*****************************************************************************/
{
  // pcl pointclouds free themselves
}

/*****************************************************************************/
void SpatioTemporalVoxelGrid::InitializeGrid(void)
/*****************************************************************************/
{
  // initialize the OpenVDB Grid volume
  openvdb::initialize();

  // make it default to background value
  _grid = openvdb::DoubleGrid::create(_background_value);

  // setup scale and tranform
  openvdb::Mat4d m = openvdb::Mat4d::identity();
  m.preScale(openvdb::Vec3d(_voxel_size, _voxel_size, _voxel_size));
  m.preTranslate(openvdb::Vec3d(0, 0, 0));
  m.preRotate(openvdb::math::Z_AXIS, 0);

  // setup transform and other metadata
  _grid->setTransform(openvdb::math::Transform::createLinearTransform(m));
  _grid->setName("SpatioTemporalVoxelLayer");
  _grid->insertMeta("Voxel Size", openvdb::FloatMetadata(_voxel_size));
  _grid->setGridClass(openvdb::GRID_LEVEL_SET);
}

/*****************************************************************************/
void SpatioTemporalVoxelGrid::SetCostmapWindow(
  const double & origin_x, const double & origin_y, const double & resolution,
  const unsigned int & size_x, const unsigned int & size_y)
/*****************************************************************************/
{
  boost::unique_lock<boost::mutex> lock(_grid_lock);

  _costmap_origin_x = origin_x;
  _costmap_origin_y = origin_y;
  _costmap_resolution = resolution;
  _costmap_size_x = size_x;
  _costmap_size_y = size_y;

  // Resize only when the window resizes; a rolling window keeps its size and merely
  // moves. Zeroing, on the other hand, happens on every call, and this is the right
  // place for it precisely because it is the one step the layer runs in every mode --
  // mapping_mode skips ClearFrustums entirely, and a window that moved under stale
  // counts would smear last cycle's obstacles across the new one.
  const size_t cells = static_cast<size_t>(size_x) * static_cast<size_t>(size_y);
  if (_cost_map.size() != cells) {
    _cost_map.assign(cells, 0);
  } else {
    std::fill(_cost_map.begin(), _cost_map.end(), 0);
  }
}

/*****************************************************************************/
void SpatioTemporalVoxelGrid::ClearFrustums(
  const std::vector<observation::MeasurementReading> & clearing_readings)
/*****************************************************************************/
{
  boost::unique_lock<boost::mutex> lock(_grid_lock);

  // accelerate the decay of voxels interior to the frustum.
  //
  // The column counts were zeroed by SetCostmapWindow, which the layer calls at the top
  // of every updateBounds, ahead of this.
  if (this->IsGridEmpty()) {
    _grid_points->clear();
    return;
  }

  _grid_points->clear();

  std::vector<frustum_model> obs_frustums;

  if (clearing_readings.size() == 0) {
    TemporalClearAndGenerateCostmap(obs_frustums);
    return;
  }

  obs_frustums.reserve(clearing_readings.size());

  std::vector<observation::MeasurementReading>::const_iterator it =
    clearing_readings.begin();
  for (; it != clearing_readings.end(); ++it) {
    // A source with decay_acceleration 0 cannot change any voxel's fate, so building
    // its frustum and testing every voxel against it is pure cost. GetFrustumAcceleration
    // is (1/6) * factor * t^3, which is identically zero when the factor is, and the two
    // branches in TemporalClearAndGenerateCostmap then reduce to exactly the code that
    // runs for a voxel outside every frustum -- except that the inside branch also writes
    // the value back unchanged. So the frustum was not merely useless, it was the more
    // expensive of the two paths.
    //
    // This is not a corner case on this robot: every source is configured with
    // decay_acceleration 0.0, and lidar_clear is a 360-degree, 90-degree-tall dome that
    // essentially every live voxel falls inside. Skipping these deletes both the frustum
    // tests and the redundant write for the whole grid, and the result is bit-identical.
    if (it->_decay_acceleration == 0.0) {
      continue;
    }

    geometry::Frustum * frustum = nullptr;
    if (it->_model_type == DEPTH_CAMERA) {
      frustum = new geometry::DepthCameraFrustum(
        it->_vertical_fov_in_rad,
        it->_horizontal_fov_in_rad, it->_min_z_in_m, it->_max_z_in_m);
    } else if (it->_model_type == THREE_DIMENSIONAL_LIDAR) {
      frustum = new geometry::ThreeDimensionalLidarFrustum(
        it->_vertical_fov_in_rad, it->_vertical_fov_padding_in_m,
        it->_horizontal_fov_in_rad, it->_min_z_in_m, it->_max_z_in_m);
    } else {
      // add else if statement for each implemented model
      delete frustum;
      continue;
    }

    frustum->SetPosition(it->_origin);
    frustum->SetOrientation(it->_orientation);
    frustum->TransformModel();
    obs_frustums.emplace_back(frustum, it->_decay_acceleration);
  }
  TemporalClearAndGenerateCostmap(obs_frustums);
}

/*****************************************************************************/
void SpatioTemporalVoxelGrid::TemporalClearAndGenerateCostmap(
  std::vector<frustum_model> & frustums)
/*****************************************************************************/
{
  // sample time once for all clearing readings
  const double cur_time = _clock->now().seconds();

  // One accessor for the whole traversal. See MarkGridPoint for why.
  openvdb::DoubleGrid::Accessor accessor = _grid->getAccessor();

  // check each point in the grid for inclusion in a frustum
  openvdb::DoubleGrid::ValueOnCIter cit_grid = _grid->cbeginValueOn();
  for (; cit_grid.test(); ++cit_grid) {
    const openvdb::Coord pt_index(cit_grid.getCoord());
    const openvdb::Vec3d pose_world = this->IndexToWorld(pt_index);

    std::vector<frustum_model>::iterator frustum_it = frustums.begin();
    bool frustum_cycle = false;
    bool cleared_point = false;

    const double time_since_marking = cur_time - cit_grid.getValue();
    const double base_duration_to_decay = GetTemporalClearingDuration(
      time_since_marking);

    for (; frustum_it != frustums.end(); ++frustum_it) {
      if (!frustum_it->frustum) {
        continue;
      }

      if (frustum_it->frustum->IsInside(pose_world) ) {
        frustum_cycle = true;

        const double frustum_acceleration = GetFrustumAcceleration(
          time_since_marking, frustum_it->accel_factor);

        const double time_until_decay = base_duration_to_decay -
          frustum_acceleration;
        if (time_until_decay < 0.) {
          // expired by acceleration
          cleared_point = true;
          if (!this->ClearGridPoint(accessor, pt_index)) {
            std::cout << "Failed to clear point." << std::endl;
          }
          break;
        } else {
          const double updated_mark = cit_grid.getValue() -
            frustum_acceleration;
          if (!this->MarkGridPoint(accessor, pt_index, updated_mark)) {
            std::cout << "Failed to update mark." << std::endl;
          }
          break;
        }
      }
    }

    // if not inside any, check against nominal decay model
    if (!frustum_cycle) {
      if (base_duration_to_decay < 0.) {
        // expired by temporal clearing
        cleared_point = true;
        if (!this->ClearGridPoint(accessor, pt_index)) {
          std::cout << "Failed to clear point." << std::endl;
        }
      }
    }

    // A cleared voxel used to be recorded in a hash set so the layer could touch() its
    // cell and so widen the update bounds enough for the clearing to reach the master
    // grid. The set cost one insert per cleared voxel per cycle; the layer now simply
    // declares the whole window dirty instead, which is a strict superset of those cells
    // and costs two touch() calls. See UpdateROSCostmap.
    if (!cleared_point) {
      // if here, we can add to costmap and PC2
      PopulateCostmapAndPointcloud(pose_world);
    }
  }

  // free memory taken by expired voxels
  _grid->pruneGrid();
}

/*****************************************************************************/
void SpatioTemporalVoxelGrid::PopulateCostmapAndPointcloud(
  const openvdb::Vec3d & pose_world)
/*****************************************************************************/
{
  if (_pub_voxels) {
    geometry_msgs::msg::Point32 point;
    point.x = pose_world[0];
    point.y = pose_world[1];
    point.z = pose_world[2];
    _grid_points->push_back(point);
  }

  if (_cost_map.empty()) {
    return;
  }

  // Deliberately Costmap2D::worldToMap's arithmetic, character for character, including
  // the unsigned wrap it relies on to reject anything left of or below the origin. A
  // cell counted here that the layer would have rejected -- or the reverse -- would put
  // obstacles a costmap cell away from where the layer believes they are.
  if (pose_world[0] < _costmap_origin_x || pose_world[1] < _costmap_origin_y) {
    return;
  }
  const unsigned int mx = static_cast<unsigned int>(
    (pose_world[0] - _costmap_origin_x) / _costmap_resolution);
  const unsigned int my = static_cast<unsigned int>(
    (pose_world[1] - _costmap_origin_y) / _costmap_resolution);
  if (mx >= _costmap_size_x || my >= _costmap_size_y) {
    return;
  }

  _cost_map[static_cast<size_t>(my) * _costmap_size_x + mx] += 1;
}

/*****************************************************************************/
void SpatioTemporalVoxelGrid::Mark(
  const std::vector<observation::MeasurementReading> & marking_readings)
/*****************************************************************************/
{
  boost::unique_lock<boost::mutex> lock(_grid_lock);

  // mark the grid
  if (marking_readings.size() > 0) {
    for (uint i = 0; i != marking_readings.size(); i++) {
      (*this)(marking_readings.at(i));
    }
  }
}

/*****************************************************************************/
void SpatioTemporalVoxelGrid::operator()(
  const observation::MeasurementReading & obs) const
/*****************************************************************************/
{
  if (obs._marking) {
    float mark_range_2 = obs._obstacle_range_in_m * obs._obstacle_range_in_m;
    const double cur_time = _clock->now().seconds();

    // One accessor for the whole cloud. See MarkGridPoint.
    openvdb::DoubleGrid::Accessor accessor = _grid->getAccessor();

    const sensor_msgs::msg::PointCloud2 & cloud = *(obs._cloud);
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

    for (; iter_x != iter_x.end();
      ++iter_x, ++iter_y, ++iter_z)
    {
      float distance_2 =
        (*iter_x - obs._origin.x) * (*iter_x - obs._origin.x) +
        (*iter_y - obs._origin.y) * (*iter_y - obs._origin.y) +
        (*iter_z - obs._origin.z) * (*iter_z - obs._origin.z);
      if (distance_2 > mark_range_2 || distance_2 < 0.0001) {
        continue;
      }

      double x = *iter_x < 0 ? *iter_x - _voxel_size : *iter_x;
      double y = *iter_y < 0 ? *iter_y - _voxel_size : *iter_y;
      double z = *iter_y < 0 ? *iter_z - _voxel_size : *iter_z;

      openvdb::Vec3d mark_grid(this->WorldToIndex(
          openvdb::Vec3d(x, y, z)));

      if (!this->MarkGridPoint(
          accessor,
          openvdb::Coord(
            mark_grid[0], mark_grid[1],
            mark_grid[2]), cur_time))
      {
        std::cout << "Failed to mark point." << std::endl;
      }
    }
  }
}

/*****************************************************************************/
const std::vector<uint16_t> &
SpatioTemporalVoxelGrid::GetFlattenedCostmap() const
/*****************************************************************************/
{
  return _cost_map;
}

/*****************************************************************************/
double SpatioTemporalVoxelGrid::GetTemporalClearingDuration(
  const double & time_delta)
/*****************************************************************************/
{
  // use configurable model to get desired decay time
  if (_decay_model == 0) {  // Linear
    return _voxel_decay - time_delta;
  } else if (_decay_model == 1) {  // Exponential
    return _voxel_decay * std::exp(-time_delta);
  }
  return _voxel_decay;  // PERSISTENT
}

/*****************************************************************************/
double SpatioTemporalVoxelGrid::GetFrustumAcceleration(
  const double & time_delta, const double & acceleration_factor)
/*****************************************************************************/
{
  const double acceleration = 1. / 6. * acceleration_factor *
    (time_delta * time_delta * time_delta);
  return acceleration;
}

/*****************************************************************************/
void SpatioTemporalVoxelGrid::GetOccupancyPointCloud(
  std::unique_ptr<sensor_msgs::msg::PointCloud2> & pc2)
/*****************************************************************************/
{
  // convert the grid points stored in a PointCloud2
  pc2->width = _grid_points->size();
  pc2->height = 1;
  pc2->is_dense = true;

  sensor_msgs::PointCloud2Modifier modifier(*pc2);

  modifier.setPointCloud2Fields(
    3,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32);
  modifier.setPointCloud2FieldsByString(1, "xyz");

  sensor_msgs::PointCloud2Iterator<float> iter_x(*pc2, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*pc2, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*pc2, "z");

  for (std::vector<geometry_msgs::msg::Point32>::iterator it =
    _grid_points->begin();
    it != _grid_points->end(); ++it)
  {
    const geometry_msgs::msg::Point32 & pt = *it;
    *iter_x = pt.x;
    *iter_y = pt.y;
    *iter_z = pt.z;
    ++iter_x; ++iter_y; ++iter_z;
  }
}

/*****************************************************************************/
bool SpatioTemporalVoxelGrid::ResetGrid(void)
/*****************************************************************************/
{
  boost::unique_lock<boost::mutex> lock(_grid_lock);

  // clear the voxel grid
  try {
    _grid->clear();
    if (this->IsGridEmpty()) {
      return true;
    }
  } catch (...) {
    std::cout << "Failed to reset costmap, please try again." << std::endl;
  }
  return false;
}

/*****************************************************************************/
void SpatioTemporalVoxelGrid::ResetGridArea(
  const occupany_cell & start, const occupany_cell & end, bool invert_area)
/*****************************************************************************/
{
  boost::unique_lock<boost::mutex> lock(_grid_lock);

  openvdb::DoubleGrid::Accessor accessor = _grid->getAccessor();

  openvdb::DoubleGrid::ValueOnCIter cit_grid = _grid->cbeginValueOn();
  for (; cit_grid.test(); ++cit_grid)
  {
    const openvdb::Coord pt_index(cit_grid.getCoord());
    const openvdb::Vec3d pose_world = this->IndexToWorld(pt_index);

    const bool in_x_range = pose_world.x() > start.x && pose_world.x() < end.x;
    const bool in_y_range = pose_world.y() > start.y && pose_world.y() < end.y;
    const bool in_range = in_x_range && in_y_range;

    if(in_range != invert_area)
    {
      ClearGridPoint(accessor, pt_index);
    }
  }
}

/*****************************************************************************/
bool SpatioTemporalVoxelGrid::MarkGridPoint(
  openvdb::DoubleGrid::Accessor & accessor, const openvdb::Coord & pt,
  const double & value) const
/*****************************************************************************/
{
  // marking the OpenVDB set.
  //
  // The accessor arrives from the caller and is reused across a whole traversal. This is
  // the documented way to write a VDB tree: the accessor caches the node stack from the
  // last access, so a run of nearby coordinates -- which is what both a point cloud and a
  // value iterator produce -- resolves inside the cached leaf instead of descending from
  // the root each time. Constructing one per call, as this used to, guarantees a cold
  // cache on every single point. Node insertion is safe through a cached accessor; that
  // case is exactly what ValueAccessor is built to handle.
  //
  // The read-back is kept: with a warm accessor it lands in the leaf the write just
  // touched, so it now costs an array index rather than a second descent.
  accessor.setValueOn(pt, value);
  return accessor.getValue(pt) == value;
}

/*****************************************************************************/
bool SpatioTemporalVoxelGrid::ClearGridPoint(
  openvdb::DoubleGrid::Accessor & accessor, const openvdb::Coord & pt) const
/*****************************************************************************/
{
  // clearing the OpenVDB set. Caller-owned accessor, same reasoning as MarkGridPoint --
  // and it matters more here, because this does three tree accesses per call.
  if (accessor.isValueOn(pt)) {
    accessor.setValueOff(pt, _background_value);
  }
  return !accessor.isValueOn(pt);
}

/*****************************************************************************/
openvdb::Vec3d SpatioTemporalVoxelGrid::IndexToWorld(
  const openvdb::Coord & coord) const
/*****************************************************************************/
{
  // Applies tranform stored in getTransform.
  openvdb::Vec3d pose_world =  _grid->indexToWorld(coord);

  // Using the center for world coordinate
  const double & center_offset = _voxel_size / 2.0;
  pose_world[0] += center_offset;
  pose_world[1] += center_offset;
  pose_world[2] += center_offset;

  return pose_world;
}

/*****************************************************************************/
openvdb::Vec3d SpatioTemporalVoxelGrid::WorldToIndex(
  const openvdb::Vec3d & vec) const
/*****************************************************************************/
{
  // Applies inverse tranform stored in getTransform.
  return _grid->worldToIndex(vec);
}

/*****************************************************************************/
bool SpatioTemporalVoxelGrid::IsGridEmpty(void) const
/*****************************************************************************/
{
  // Returns grid's population status
  return _grid->empty();
}

/*****************************************************************************/
bool SpatioTemporalVoxelGrid::SaveGrid(
  const std::string & file_name, double & map_size_bytes)
/*****************************************************************************/
{
  try {
    openvdb::io::File file(file_name + ".vdb");
    openvdb::GridPtrVec grids = {_grid};
    file.write(grids);
    file.close();
    map_size_bytes = _grid->memUsage();
    return true;
  } catch (...) {
    map_size_bytes = 0.;
    return false;
  }
  return false;  // best offense is a good defense
}

}  // namespace volume_grid
