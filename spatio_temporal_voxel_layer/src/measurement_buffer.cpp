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

#include <string>
#include <memory>
#include <vector>
#include "spatio_temporal_voxel_layer/measurement_buffer.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace buffer
{

using namespace std::chrono_literals;

/*****************************************************************************/
MeasurementBuffer::MeasurementBuffer(
  const std::string & source_name,
  const std::string & topic_name,
  const double & observation_keep_time, const double & expected_update_rate,
  const double & min_obstacle_height, const double & max_obstacle_height,
  const double & obstacle_range, tf2_ros::Buffer & tf, const std::string & global_frame,
  const std::string & sensor_frame, const double & tf_tolerance,
  const double & min_d, const double & max_d, const double & vFOV,
  const double & vFOVPadding, const double & hFOV,
  const double & decay_acceleration, const bool & marking,
  const bool & clearing, const double & voxel_size, const Filters & filter,
  const int & voxel_min_points, const bool & enabled,
  const bool & clear_buffer_after_reading, const ModelType & model_type,
  const std::string & height_filter_frame,
  const bool & ground_relative_height, const double & ground_max_grade_deg,
  const double & ground_height_tol, const double & ground_height_tol_per_m,
  const bool & ground_reference_publish, const bool & ground_reference_use,
  std::shared_ptr<ground_seg::GroundReference> ground_reference,
  rclcpp::Clock::SharedPtr clock, rclcpp::Logger logger)
: _buffer(tf),
  _observation_keep_time(rclcpp::Duration::from_seconds(observation_keep_time)),
  _expected_update_rate(rclcpp::Duration::from_seconds(expected_update_rate)),
  _last_updated(clock->now()),
  _global_frame(global_frame), _sensor_frame(sensor_frame), _source_name(source_name),
  _topic_name(topic_name), _min_obstacle_height(min_obstacle_height),
  _max_obstacle_height(max_obstacle_height), _obstacle_range(obstacle_range),
  _tf_tolerance(tf_tolerance), _min_z(min_d), _max_z(max_d),
  _vertical_fov(vFOV), _vertical_fov_padding(vFOVPadding),
  _horizontal_fov(hFOV), _decay_acceleration(decay_acceleration),
  _voxel_size(voxel_size), _marking(marking), _clearing(clearing),
  _filter(filter), _voxel_min_points(voxel_min_points),
  _clear_buffer_after_reading(clear_buffer_after_reading),
  _enabled(enabled), _model_type(model_type),
  _height_filter_frame(height_filter_frame),
  _ground_relative_height(ground_relative_height),
  _ground_max_grade_deg(ground_max_grade_deg),
  _ground_height_tol(ground_height_tol),
  _ground_height_tol_per_m(ground_height_tol_per_m),
  _ground_reference_publish(ground_reference_publish),
  _ground_reference_use(ground_reference_use),
  _ground_reference(ground_reference),
  clock_(clock), logger_(logger)
/*****************************************************************************/
{
}

/*****************************************************************************/
MeasurementBuffer::~MeasurementBuffer(void)
/*****************************************************************************/
{
}

/*****************************************************************************/
bool MeasurementBuffer::FilterGroundRelative(point_cloud_ptr & cld) const
/*****************************************************************************/
{
  // `cld` is organized and already in _height_filter_frame, whose z is up and whose
  // z ~ 0 is the plane the robot stands on. Both facts are what let the column walk
  // mean anything, and neither is checked here -- the caller establishes them.
  const std::size_t n = static_cast<std::size_t>(cld->width) * cld->height;

  sensor_msgs::PointCloud2ConstIterator<float> ix(*cld, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iy(*cld, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iz(*cld, "z");

  std::vector<ground_seg::GroundPoint> pts(n);
  for (std::size_t i = 0; i < n; ++i, ++ix, ++iy, ++iz) {
    const float x = *ix, y = *iy, z = *iz;
    // A missing pixel reaches us as exactly (0,0,0), not as NaN, on this robot's depth
    // driver -- about 15% of every frame, with is_dense already false. Left in, those
    // all transform to the camera's own origin and read as a floor sample directly
    // under the robot, which is both wrong and exactly where a wrong sample does most
    // damage. Measured on real recorded frames; NaN is handled too, for drivers
    // that use it.
    const bool ok = std::isfinite(x) && std::isfinite(y) && std::isfinite(z) &&
      !(x == 0.0f && y == 0.0f && z == 0.0f);
    pts[i] = {x, y, z, ok};
  }

  ground_seg::GroundColumnsConfig cfg;
  cfg.max_grade_deg = static_cast<float>(_ground_max_grade_deg);
  cfg.height_tol = static_cast<float>(_ground_height_tol);
  cfg.height_tol_per_m = static_cast<float>(_ground_height_tol_per_m);
  // The band this layer is configured with is what the walk should consider "not floor",
  // so the two stay in step instead of being tuned against each other.
  cfg.max_ground_z = static_cast<float>(_max_obstacle_height);

  // Seeding may consult what the other cameras have seen. Only the seed uses it; the walk
  // that follows is still this camera's own observation.
  // The grid lives in the global frame (odom); the walk works in the filter frame
  // (base_link). Both transforms are already available, and the round trip is two 3x3
  // multiplies per lookup, which is why the grid is consulted only at seeding.
  ground_seg::GroundSeedReference seed_ref;
  const double now = clock_->now().seconds();
  geometry_msgs::msg::TransformStamped f2g;
  bool have_f2g = false;
  if ((_ground_reference_use || _ground_reference_publish) && _ground_reference) {
    try {
      f2g = _buffer.lookupTransform(
        _global_frame, _height_filter_frame, tf2_ros::fromMsg(cld->header.stamp));
      have_f2g = true;
    } catch (tf2::TransformException &) {
      have_f2g = false;   // no grid this frame; the walk seeds on its own as before
    }
  }
  if (have_f2g && _ground_reference_use) {
    std::shared_ptr<ground_seg::GroundReference> ref = _ground_reference;
    const geometry_msgs::msg::TransformStamped tf = f2g;
    seed_ref = [ref, tf, now](float x, float y, float & z) {
        geometry_msgs::msg::PointStamped in, out;
        in.point.x = x; in.point.y = y; in.point.z = 0.0;
        tf2::doTransform(in, out, tf);
        float gz;
        if (!ref->Lookup(
            static_cast<float>(out.point.x), static_cast<float>(out.point.y), now, gz))
        {
          return false;
        }
        // The grid answers in odom; the caller is asking in the filter frame, and only
        // the height differs between them on this robot. Subtracting the frame's own
        // height converts it back.
        z = gz - static_cast<float>(tf.transform.translation.z);
        return true;
      };
  
  }

  const ground_seg::GroundColumnsResult seg =
    ground_seg::segment_ground_columns(pts, cld->width, cld->height, cfg, seed_ref);
  if (seg.classes.size() != n) {
    return false;
  }

  if (_ground_reference_publish && _ground_reference && have_f2g) {
    // Only columns that seeded from their own view are published. A column seeded FROM the
    // grid writing its own result back would let the estimate drift with nothing holding
    // it to an observation.
    std::vector<std::pair<float, float>> xy;
    std::vector<float> gz;
    xy.reserve(n / 8);
    gz.reserve(n / 8);
    for (std::size_t i = 0; i < n; ++i) {
      if (!pts[i].valid || seg.classes[i] != ground_seg::GroundClass::GROUND) {
        continue;
      }
      const std::size_t u = i % cld->width;
      if (u >= seg.column_native_seed.size() || !seg.column_native_seed[u]) {
        continue;
      }
      geometry_msgs::msg::PointStamped in, out;
      in.point.x = pts[i].x; in.point.y = pts[i].y; in.point.z = pts[i].z;
      tf2::doTransform(in, out, f2g);
      xy.emplace_back(static_cast<float>(out.point.x), static_cast<float>(out.point.y));
      gz.push_back(static_cast<float>(out.point.z));
    }
    _ground_reference->Recenter(
      f2g.transform.translation.x, f2g.transform.translation.y);
    _ground_reference->Publish(xy, gz, now);
  }

  // A column that never found floor cannot say how high anything in it is. Falling back
  // to the fixed band for those columns rather than dropping them: not knowing where the
  // floor is, is not a reason to stop reporting obstacles.
  std::vector<uint8_t> unseeded(cld->width, 0u);
  for (std::size_t u = 0; u < seg.column_ground.size(); ++u) {
    unseeded[u] = seg.column_ground[u].empty() ? 1u : 0u;
  }

  std::vector<float> keep;
  keep.reserve(n * 3);
  sensor_msgs::PointCloud2ConstIterator<float> jx(*cld, "x");
  sensor_msgs::PointCloud2ConstIterator<float> jy(*cld, "y");
  sensor_msgs::PointCloud2ConstIterator<float> jz(*cld, "z");
  for (std::size_t i = 0; i < n; ++i, ++jx, ++jy, ++jz) {
    if (!pts[i].valid) {
      continue;
    }
    // Ground and holes are never obstacles. Dropping HOLE here is deliberate and is not
    // the same as ignoring a drop-off: a hole is an absence of floor, and turning it
    // into a lethal cell would put a wall at the top of every staircase. Detecting the
    // drop itself is a separate job from marking obstacles.
    const ground_seg::GroundClass c = seg.classes[i];
    if (c == ground_seg::GroundClass::GROUND || c == ground_seg::GroundClass::HOLE) {
      continue;
    }

    const std::size_t u = i % cld->width;
    float base = 0.0f;
    if (!unseeded[u]) {
      const float range = std::sqrt((*jx) * (*jx) + (*jy) * (*jy));
      base = NearestGroundZ(seg.column_ground[u], range);
    }
    const float h = *jz - base;
    if (h < _min_obstacle_height || h > _max_obstacle_height) {
      continue;
    }
    keep.push_back(*jx);
    keep.push_back(*jy);
    keep.push_back(*jz);
  }

  // Rebuild as an unorganized cloud of survivors. The organization has served its
  // purpose by this point and everything downstream treats the cloud as a bag of points.
  point_cloud_ptr out(new sensor_msgs::msg::PointCloud2());
  out->header = cld->header;
  sensor_msgs::PointCloud2Modifier mod(*out);
  mod.setPointCloud2FieldsByString(1, "xyz");
  mod.resize(keep.size() / 3);
  sensor_msgs::PointCloud2Iterator<float> ox(*out, "x");
  sensor_msgs::PointCloud2Iterator<float> oy(*out, "y");
  sensor_msgs::PointCloud2Iterator<float> oz(*out, "z");
  for (std::size_t k = 0; k + 2 < keep.size(); k += 3, ++ox, ++oy, ++oz) {
    *ox = keep[k];
    *oy = keep[k + 1];
    *oz = keep[k + 2];
  }
  out->is_dense = true;
  cld.swap(out);
  return true;
}

/*****************************************************************************/
float MeasurementBuffer::NearestGroundZ(
  const std::vector<std::pair<float, float>> & profile, const float & range)
/*****************************************************************************/
{
  // The profile is ascending in range (segment_ground_columns walks near to far), so
  // this is a binary search for the nearer of the two samples bracketing `range`.
  // Beyond either end the nearest sample is the best estimate available; extrapolating
  // a slope past the last floor actually seen would invent ground where the walk
  // deliberately stopped.
  if (profile.empty()) {
    return 0.0f;
  }
  std::vector<std::pair<float, float>>::const_iterator it =
    std::lower_bound(
    profile.begin(), profile.end(), range,
    [](const std::pair<float, float> & a, const float & r) {return a.first < r;});
  if (it == profile.begin()) {
    return profile.front().second;
  }
  if (it == profile.end()) {
    return profile.back().second;
  }
  std::vector<std::pair<float, float>>::const_iterator pr = it - 1;
  return (range - pr->first <= it->first - range) ? pr->second : it->second;
}

/*****************************************************************************/
void MeasurementBuffer::BufferROSCloud(
  const sensor_msgs::msg::PointCloud2 & cloud)
/*****************************************************************************/
{
  // add a new measurement to be populated
  _observation_list.push_front(observation::MeasurementReading());

  const std::string origin_frame =
    _sensor_frame == "" ? cloud.header.frame_id : _sensor_frame;

  try {
    // transform into global frame
    geometry_msgs::msg::PoseStamped local_pose, global_pose;
    local_pose.pose.position.x = 0;
    local_pose.pose.position.y = 0;
    local_pose.pose.position.z = 0;
    local_pose.pose.orientation.x = 0;
    local_pose.pose.orientation.y = 0;
    local_pose.pose.orientation.z = 0;
    local_pose.pose.orientation.w = 1;
    local_pose.header.stamp = cloud.header.stamp;
    local_pose.header.frame_id = origin_frame;

    _buffer.canTransform(
      _global_frame, local_pose.header.frame_id,
      tf2_ros::fromMsg(local_pose.header.stamp), tf2::durationFromSec(0.5));
    _buffer.transform(local_pose, global_pose, _global_frame);

    _observation_list.front()._origin.x = global_pose.pose.position.x;
    _observation_list.front()._origin.y = global_pose.pose.position.y;
    _observation_list.front()._origin.z = global_pose.pose.position.z;

    _observation_list.front()._orientation = global_pose.pose.orientation;
    _observation_list.front()._obstacle_range_in_m = _obstacle_range;
    _observation_list.front()._min_z_in_m = _min_z;
    _observation_list.front()._max_z_in_m = _max_z;
    _observation_list.front()._vertical_fov_in_rad = _vertical_fov;
    _observation_list.front()._vertical_fov_padding_in_m =
      _vertical_fov_padding;
    _observation_list.front()._horizontal_fov_in_rad = _horizontal_fov;
    _observation_list.front()._decay_acceleration = _decay_acceleration;
    _observation_list.front()._clearing = _clearing;
    _observation_list.front()._marking = _marking;
    _observation_list.front()._model_type = _model_type;

    if (_clearing && !_marking) {
      // no need to buffer points
      return;
    }

    // Height gating happens in _height_filter_frame, which is the global frame unless a
    // source names another one. Filtering in the global frame means the band is measured
    // from the world's horizontal, so on a ramp it no longer describes the robot: drive
    // 1 m down a slope and a 0.30 m obstacle sits at global z = -0.70, below
    // min_obstacle_height, and is dropped. Naming base_link instead keeps the band tied
    // to the chassis and tilts with it.
    //
    // The cloud goes to the filter frame first because both filters gate on the z FIELD
    // and so measure whatever frame the points are already in, then on to the global
    // frame afterwards -- second transform included, this is cheaper than it looks, since
    // it runs on the cloud the filter has already thinned.
    point_cloud_ptr cld_global(new sensor_msgs::msg::PointCloud2());
    const bool filter_elsewhere =
      !_height_filter_frame.empty() && _height_filter_frame != _global_frame;
    const std::string first_frame =
      filter_elsewhere ? _height_filter_frame : _global_frame;

    geometry_msgs::msg::TransformStamped tf_stamped =
      _buffer.lookupTransform(
      first_frame, cloud.header.frame_id,
      tf2_ros::fromMsg(cloud.header.stamp));
    tf2::doTransform(cloud, *cld_global, tf_stamped);

    // Ground-relative height gating. Replaces the fixed band below with the same band
    // measured from the ground found under each point, so a ramp's own surface is
    // ground rather than a 0.5 m obstacle. Runs here because this is the last point at
    // which the cloud is still ORGANIZED -- tf2::doTransform preserves width/height,
    // whereas the PCL filters that follow do not -- and the column walk needs that.
    //
    // It rewrites cld_global into an unorganized cloud of surviving points, then hands
    // the fixed band a range wide enough to be inert so the VoxelGrid below only
    // downsamples. Points are otherwise untouched: the gate decides membership, it does
    // not move anything.
    bool ground_gated = false;
    if (_ground_relative_height) {
      if (cld_global->height <= 1) {
        RCLCPP_WARN_ONCE(
          logger_,
          "%s: ground_relative_height needs an organized cloud; %s is %ux%u. "
          "Falling back to the fixed obstacle-height band.",
          _source_name.c_str(), _topic_name.c_str(), cld_global->width, cld_global->height);
      } else {
        ground_gated = FilterGroundRelative(cld_global);
      }
    }

    pcl::PCLPointCloud2::Ptr cloud_pcl(new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());

    // Already gated above; leave the band open so the filters only downsample.
    const double band_min = ground_gated ? -std::numeric_limits<double>::infinity()
                                         : _min_obstacle_height;
    const double band_max = ground_gated ? std::numeric_limits<double>::infinity()
                                         : _max_obstacle_height;

    // remove points that are below or above our height restrictions, and
    // in the same time, remove NaNs and if user wants to use it, combine with a
    if (_filter == Filters::VOXEL) {
      pcl_conversions::toPCL(*cld_global, *cloud_pcl);
      pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
      sor.setInputCloud(cloud_pcl);
      sor.setFilterFieldName("z");
      sor.setFilterLimits(band_min, band_max);
      sor.setDownsampleAllData(false);
      float v_s = static_cast<float>(_voxel_size);
      sor.setLeafSize(v_s, v_s, v_s);
      sor.setMinimumPointsNumberPerVoxel(static_cast<unsigned int>(_voxel_min_points));
      sor.filter(*cloud_filtered);
      pcl_conversions::fromPCL(*cloud_filtered, *cld_global);
    } else if (_filter == Filters::PASSTHROUGH) {
      pcl_conversions::toPCL(*cld_global, *cloud_pcl);
      pcl::PassThrough<pcl::PCLPointCloud2> pass_through_filter;
      pass_through_filter.setInputCloud(cloud_pcl);
      pass_through_filter.setKeepOrganized(false);
      pass_through_filter.setFilterFieldName("z");
      pass_through_filter.setFilterLimits(band_min, band_max);
      pass_through_filter.filter(*cloud_filtered);
      pcl_conversions::fromPCL(*cloud_filtered, *cld_global);
    }

    if (filter_elsewhere) {
      point_cloud_ptr cld_out(new sensor_msgs::msg::PointCloud2());
      geometry_msgs::msg::TransformStamped tf_to_global =
        _buffer.lookupTransform(
        _global_frame, _height_filter_frame,
        tf2_ros::fromMsg(cloud.header.stamp));
      tf2::doTransform(*cld_global, *cld_out, tf_to_global);
      cld_global.swap(cld_out);
    }

    _observation_list.front()._cloud.reset(cld_global.release());
  } catch (tf2::TransformException & ex) {
    // if fails, remove the empty observation
    _observation_list.pop_front();
    RCLCPP_ERROR(
      logger_,
      "TF Exception for sensor frame: %s, cloud frame: %s, %s",
      _sensor_frame.c_str(), cloud.header.frame_id.c_str(), ex.what());
    return;
  }

  _last_updated = clock_->now();
  RemoveStaleObservations();
}

/*****************************************************************************/
void MeasurementBuffer::GetReadings(
  std::vector<observation::MeasurementReading> & observations)
/*****************************************************************************/
{
  RemoveStaleObservations();

  for (readings_iter it = _observation_list.begin();
    it != _observation_list.end(); ++it)
  {
    observations.push_back(*it);
  }
}

/*****************************************************************************/
void MeasurementBuffer::RemoveStaleObservations(void)
/*****************************************************************************/
{
  if (_observation_list.empty()) {
    return;
  }

  readings_iter it = _observation_list.begin();
  if (_observation_keep_time == rclcpp::Duration(rclcpp::Duration::from_seconds(0.0))) {
    _observation_list.erase(++it, _observation_list.end());
    return;
  }

  for (it = _observation_list.begin(); it != _observation_list.end(); ++it) {
    const rclcpp::Duration time_diff = clock_->now() - it->_cloud->header.stamp;

    if (time_diff > _observation_keep_time) {
      _observation_list.erase(it, _observation_list.end());
      return;
    }
  }
}

/*****************************************************************************/
void MeasurementBuffer::ResetAllMeasurements(void)
/*****************************************************************************/
{
  _observation_list.clear();
}

/*****************************************************************************/
bool MeasurementBuffer::ClearAfterReading(void)
/*****************************************************************************/
{
  return _clear_buffer_after_reading;
}

/*****************************************************************************/
bool MeasurementBuffer::UpdatedAtExpectedRate(void) const
/*****************************************************************************/
{
  if (_expected_update_rate == rclcpp::Duration(rclcpp::Duration::from_seconds(0.0))) {
    return true;
  }

  const rclcpp::Duration update_time = clock_->now() - _last_updated;
  bool current = update_time.seconds() <= _expected_update_rate.seconds();
  if (!current) {
    RCLCPP_WARN(
      logger_,
      "%s buffer updated in %.2fs, it should be updated every %.2fs.",
      _topic_name.c_str(), update_time.seconds(),
      _expected_update_rate.seconds());
  }
  return current;
}

/*****************************************************************************/
bool MeasurementBuffer::IsEnabled(void) const
/*****************************************************************************/
{
  return _enabled;
}

/*****************************************************************************/
void MeasurementBuffer::SetEnabled(const bool & enabled)
/*****************************************************************************/
{
  _enabled = enabled;
}

/*****************************************************************************/
std::string MeasurementBuffer::GetSourceName(void) const
/*****************************************************************************/
{
  return _source_name;
}

/*****************************************************************************/
void MeasurementBuffer::SetMinObstacleHeight(const double & min_obstacle_height)
/*****************************************************************************/
{
  _min_obstacle_height = min_obstacle_height;
}

/*****************************************************************************/
void MeasurementBuffer::SetMaxObstacleHeight(const double & max_obstacle_height)
/*****************************************************************************/
{
  _max_obstacle_height = max_obstacle_height;
}

/*****************************************************************************/
void MeasurementBuffer::SetMinZ(const double & min_z)
/*****************************************************************************/
{
  _min_z = min_z;
}

/*****************************************************************************/
void MeasurementBuffer::SetMaxZ(const double & max_z)
/*****************************************************************************/
{
  _max_z = max_z;
}

/*****************************************************************************/
void MeasurementBuffer::SetVerticalFovAngle(const double & vertical_fov_angle)
/*****************************************************************************/
{
  _vertical_fov = vertical_fov_angle;
}

/*****************************************************************************/
void MeasurementBuffer::SetVerticalFovPadding(const double & vertical_fov_padding)
/*****************************************************************************/
{
  _vertical_fov_padding = vertical_fov_padding;
}

/*****************************************************************************/
void MeasurementBuffer::SetHorizontalFovAngle(const double & horizontal_fov_angle)
/*****************************************************************************/
{
  _horizontal_fov = horizontal_fov_angle;
}

/*****************************************************************************/
void MeasurementBuffer::ResetLastUpdatedTime(void)
/*****************************************************************************/
{
  _last_updated = clock_->now();
}

/*****************************************************************************/
void MeasurementBuffer::Lock(void)
/*****************************************************************************/
{
  _lock.lock();
}

/*****************************************************************************/
void MeasurementBuffer::Unlock(void)
/*****************************************************************************/
{
  _lock.unlock();
}

}  // namespace buffer
