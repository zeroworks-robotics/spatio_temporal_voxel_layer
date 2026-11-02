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
#include <cstdint>
#include <limits>
#include <unordered_set>
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
  const bool & ground_relative_height, const double & ground_min_obstacle_height,
  const double & ground_seed_z_tol, const double & ground_seed_z_tol_per_m,
  const bool & ground_seed_gravity_aligned,
  const double & ground_max_grade_deg,
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
  _ground_min_obstacle_height(ground_min_obstacle_height),
  _ground_seed_z_tol(ground_seed_z_tol),
  _ground_seed_z_tol_per_m(ground_seed_z_tol_per_m),
  _ground_seed_gravity_aligned(ground_seed_gravity_aligned),
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
  cfg.seed_z_tol = static_cast<float>(_ground_seed_z_tol);
  cfg.seed_z_tol_per_m = static_cast<float>(_ground_seed_z_tol_per_m);
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
  if (_ground_seed_gravity_aligned ||
    ((_ground_reference_use || _ground_reference_publish) && _ground_reference))
  {
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

  if (_ground_seed_gravity_aligned && have_f2g) {
    // Gravity, expressed in the filter frame: the third ROW of the filter->global
    // rotation, which is the global frame's z axis seen from here. The global frame is
    // gravity-aligned, so this is the direction a level floor is perpendicular to.
    const auto & q = f2g.transform.rotation;
    cfg.up_x = static_cast<float>(2.0 * (q.x * q.z - q.y * q.w));
    cfg.up_y = static_cast<float>(2.0 * (q.y * q.z + q.x * q.w));
    cfg.up_z = static_cast<float>(1.0 - 2.0 * (q.x * q.x + q.y * q.y));
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
    // Against the GATE's own band, not the fixed one. The fixed band is what this source
    // falls back to when the gate is off, and it is set for a floor at height_filter_frame's
    // origin; reusing it here would tie the two together and make the switch below change
    // more than it says.
    // Same measure the walk used, so `base` and the point are the same kind of height.
    // Mixing them would put the pitch back in exactly where it was taken out.
    const float h =
      (*jx) * cfg.up_x + (*jy) * cfg.up_y + (*jz) * cfg.up_z - base;
    if (h < _ground_min_obstacle_height || h > _max_obstacle_height) {
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
    _observation_list.front()._voxel_decay = _source_voxel_decay;
    _observation_list.front()._voxel_decay_range = _source_voxel_decay_range;
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
    if (_range_bias_k > 0.0) {
      // correct the ranges in the sensor frame, where every beam starts at the origin. Only x and y
      // move: the height gates and the ground walk keep the heights they were tuned on, and the
      // bias matters for where a wall is, not how high it is. Needs a sensor mounted near level.
      sensor_msgs::msg::PointCloud2 corrected(cloud);
      sensor_msgs::PointCloud2Iterator<float> cx(corrected, "x"), cy(corrected, "y"), cz(corrected, "z");
      for (; cx != cx.end(); ++cx, ++cy, ++cz) {
        const float r = std::sqrt(*cx * *cx + *cy * *cy + *cz * *cz);
        if (!std::isfinite(r) || r < 0.05f) {
          continue;
        }
        const float scale = 1.0f + static_cast<float>(std::min(_range_bias_k / r, _range_bias_max)) / r;
        *cx *= scale;
        *cy *= scale;
      }
      tf2::doTransform(corrected, *cld_global, tf_stamped);
    } else {
      tf2::doTransform(cloud, *cld_global, tf_stamped);
    }

    if (_edge_px > 0 && _filter != Filters::NONE) {
      if (cloud.height <= 1) {
        RCLCPP_WARN_ONCE(
          logger_, "%s: edge_filter_px needs an organized cloud; %s is %ux%u. Filter off.",
          _source_name.c_str(), _topic_name.c_str(), cloud.width, cloud.height);
      } else {
        EdgeFilter(cloud, *cld_global);
        cld_global->is_dense = false;
      }
    }
    if (_support_min > 0 && _filter != Filters::NONE) {
      if (cloud.height <= 1) {
        RCLCPP_WARN_ONCE(
          logger_, "%s: support_filter_min needs an organized cloud; %s is %ux%u. Filter off.",
          _source_name.c_str(), _topic_name.c_str(), cloud.width, cloud.height);
      } else {
        // NaN must be dropped by a filter stage before the grid; the voxel and passthrough
        // filters do, so the support filter needs one of them
        SupportFilter(cloud, *cld_global);
        cld_global->is_dense = false;
      }
    }

    // Door ray trust (lidar). Runs on the organized cloud in height_filter_frame, after the support
    // filter so a stray return cannot become a face. When it ran, the upper band it kept must pass
    // the z band below; when it did not (no door near, no map pose), nothing changed and the band is
    // exactly today's.
    bool trust_applied = false;
    if (_door_trust.enabled && _door_trust_on) {
      trust_applied = DoorTrustFilter(*cld_global, tf_stamped, cloud.header.stamp, cloud.header.frame_id);
      if (trust_applied) {
        cld_global->is_dense = false;
      }
    }

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

    // Points beyond obstacle_range never mark (the grid drops them), but they still stretch the
    // VoxelGrid's index box: on a box of more than INT32 voxels PCL warns ("Leaf size is too
    // small") and returns the cloud UNFILTERED, NaN and every height included, and the grid then
    // marks floor and ceiling. A lidar looking out of a door at a yard reaches that with a 2 cm
    // leaf. Cap the range before the filter; 2 x 14 m across stays far below the limit.
    // (the grid measures range from sensor_frame when one is set; the cut is skipped then)
    if (_filter == Filters::VOXEL && _obstacle_range > 0.0 && _sensor_frame.empty()) {
      const double ox = tf_stamped.transform.translation.x, oy = tf_stamped.transform.translation.y,
        oz = tf_stamped.transform.translation.z, r2 = _obstacle_range * _obstacle_range;
      const float nan = std::numeric_limits<float>::quiet_NaN();
      sensor_msgs::PointCloud2Iterator<float> cx(*cld_global, "x"), cy(*cld_global, "y"), cz(*cld_global, "z");
      for (; cx != cx.end(); ++cx, ++cy, ++cz) {
        const double dx = *cx - ox, dy = *cy - oy, dz = *cz - oz;
        if (std::isfinite(dx) && std::isfinite(dy) && std::isfinite(dz) && dx * dx + dy * dy + dz * dz > r2) {
          *cx = nan;
          *cy = nan;
          *cz = nan;
        }
      }
      cld_global->is_dense = false;
    }

    pcl::PCLPointCloud2::Ptr cloud_pcl(new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());

    // Already gated above; leave the band open so the filters only downsample.
    const double band_min = ground_gated ? -std::numeric_limits<double>::infinity()
                                         : _min_obstacle_height;
    const double band_max = ground_gated ? std::numeric_limits<double>::infinity()
                                         : (trust_applied ? std::nextafter(_door_trust.up_max, 0.0) : _max_obstacle_height);

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
void MeasurementBuffer::SetVoxelDecay(const double & voxel_decay)
/*****************************************************************************/
{
  _source_voxel_decay = voxel_decay;
}

/*****************************************************************************/
void MeasurementBuffer::SetEdgeFilter(const int & px, const int & min_run)
/*****************************************************************************/
{
  _edge_px = std::max(px, 0);
  // a surface must keep a core after both its edges go, or a thin pole would vanish
  _edge_min_run = std::max(min_run, 2 * _edge_px + 2);
}

/*****************************************************************************/
size_t MeasurementBuffer::EdgeFilter(
  const sensor_msgs::msg::PointCloud2 & raw, sensor_msgs::msg::PointCloud2 & out)
/*****************************************************************************/
{
  // Stereo matching widens a near surface into the far background beside it by a pixel or two
  // ("foreground fattening"), so the side of a door jamb lands a few cm inside the opening, more
  // the farther away it is. Drop those edge pixels along each image row, but only on surfaces wide
  // enough to keep a core, so thin objects stay whole. Rows of the organized cloud are image rows.
  const int W = static_cast<int>(raw.width), H = static_cast<int>(raw.height);
  if (raw.row_step != raw.width * raw.point_step || out.row_step != out.width * out.point_step ||
    out.width != raw.width || out.height != raw.height)
  {
    RCLCPP_WARN_ONCE(logger_, "%s: edge filter skipped, padded or mismatched rows", _source_name.c_str());
    return 0;
  }
  const size_t n = static_cast<size_t>(W) * H;
  const float inf = std::numeric_limits<float>::infinity();
  std::vector<float> rng(n, inf);
  sensor_msgs::PointCloud2ConstIterator<float> rx(raw, "x"), ry(raw, "y"), rz(raw, "z");
  for (size_t i = 0; i < n; ++i, ++rx, ++ry, ++rz) {
    if (std::isfinite(*rx) && std::isfinite(*ry) && std::isfinite(*rz)) {
      const float r = std::sqrt(*rx * *rx + *ry * *ry + *rz * *rz);
      if (r > 0.05f) {
        rng[i] = r;
      }
    }
  }
  std::vector<uint8_t> drop(n, 0);
  std::vector<int> run(W);
  size_t seen = 0, dropped = 0;
  for (int row = 0; row < H; ++row) {
    const float * r = rng.data() + static_cast<size_t>(row) * W;
    // width of the surface each pixel sits on: neighbours within 3 % range of each other
    for (int c = 0; c < W; ) {
      int e = c;
      while (e + 1 < W && std::isfinite(r[e]) && std::isfinite(r[e + 1]) &&
        std::fabs(r[e + 1] - r[e]) < 0.03f * r[e])
      {
        ++e;
      }
      for (int k = c; k <= e; ++k) {
        run[k] = e - c + 1;
      }
      c = e + 1;
    }
    for (int c = 0; c < W; ++c) {
      if (!std::isfinite(r[c])) {
        continue;
      }
      ++seen;
      if (run[c] < _edge_min_run) {
        continue;
      }
      const float jump = std::max(0.05f, 0.08f * r[c]);
      bool edge = false;
      for (int k = 1; k <= _edge_px && !edge; ++k) {
        // the image border is not a depth step
        edge = (c - k >= 0 && r[c - k] > r[c] + jump) || (c + k < W && r[c + k] > r[c] + jump);
      }
      if (edge) {
        drop[static_cast<size_t>(row) * W + c] = 1;
        ++dropped;
      }
    }
  }
  sensor_msgs::PointCloud2Iterator<float> ox(out, "x"), oy(out, "y"), oz(out, "z");
  const float nan = std::numeric_limits<float>::quiet_NaN();
  for (size_t i = 0; i < n; ++i, ++ox, ++oy, ++oz) {
    if (drop[i]) {
      *ox = nan;
      *oy = nan;
      *oz = nan;
    }
  }
  _edge_seen += seen;
  _edge_dropped += dropped;
  const rclcpp::Time now = clock_->now();
  if (_edge_report_time.nanoseconds() == 0 || _edge_report_time.get_clock_type() != now.get_clock_type()) {
    _edge_report_time = now;
  } else if ((now - _edge_report_time).seconds() > 30.0) {
    RCLCPP_INFO(
      logger_, "%s: edge filter dropped %.2f%% of %zu pixels in the last 30 s", _source_name.c_str(),
      100.0 * _edge_dropped / std::max<size_t>(_edge_seen, 1), _edge_seen);
    _edge_seen = _edge_dropped = 0;
    _edge_report_time = now;
  }
  return dropped;
}

/*****************************************************************************/
void MeasurementBuffer::SetVoxelDecayRange(const double & range)
/*****************************************************************************/
{
  _source_voxel_decay_range = range;
}

/*****************************************************************************/
void MeasurementBuffer::SetRangeBias(const double & k, const double & max)
/*****************************************************************************/
{
  _range_bias_k = k;
  _range_bias_max = max;
}

/*****************************************************************************/
void MeasurementBuffer::SetDoorTrust(
  const DoorTrustParams & params, const DoorZoneParams & zones, door_feed_ptr doors)
/*****************************************************************************/
{
  _door_trust = params;
  _door_trust.margin = std::max(_door_trust.margin, 0.0);
  _door_trust.depth = std::max(_door_trust.depth, _door_trust.margin);
  _door_trust.range = std::max(_door_trust.range, 0.0);
  _door_trust.az_bin_deg = std::min(std::max(_door_trust.az_bin_deg, 0.1), 45.0);
  // pooling more than half the circle would make every bin the same face
  const int nbin = std::max(1, static_cast<int>(std::lround(360.0 / _door_trust.az_bin_deg)));
  _door_trust.az_neighbours = std::min(std::max(_door_trust.az_neighbours, 0), nbin / 2);
  _door_trust.low_cap_depth = std::min(std::max(_door_trust.low_cap_depth, 0.0), _door_trust.depth);
  _door_zone = zones;
  _doors = doors;
}

/*****************************************************************************/
bool MeasurementBuffer::DoorTrustFilter(
  sensor_msgs::msg::PointCloud2 & cld, const geometry_msgs::msg::TransformStamped & cloud_to_filter,
  const builtin_interfaces::msg::Time & stamp, const std::string & cloud_frame)
/*****************************************************************************/
{
  // ranges are measured from cloud_to_filter's translation, which is the sensor only while the
  // cloud comes in its own frame (configure checks sensor_frame; a driver or a fallback topic
  // publishing in the robot's frame would move the origin without warning)
  if (cloud_frame == _height_filter_frame || cloud_frame == _global_frame || cloud_frame == _door_zone.map_frame) {
    RCLCPP_WARN_ONCE(
      logger_, "%s: door trust needs the cloud in the sensor frame, got %s; off", _source_name.c_str(),
      cloud_frame.c_str());
    return false;
  }
  // The Airy reads a surface it meets head-on within about 1.3 m a few cm short, more so near the
  // sensor's own height, so a door jamb lands 2..8 cm inside the opening in the band the costmap
  // marks (base 0.08..1.5 m). The same jamb seen by the rings above (1.5..1.8 m, farther along the
  // beam and steeper) reads where it is. A jamb is the same width at every height, so in a door
  // zone a lower point that is nearer than the upper face in its azimuth by more than margin and
  // less than depth is not trusted and is not marked; the upper band is marked there instead. No
  // point is moved. Nothing taller behind a lower point (a child, a box, a person in the opening)
  // leaves it as it is; a lower point more than depth nearer is another object and stays too.
  ++_dt_clouds;
  const auto report = [this]() {
      const rclcpp::Time now = clock_->now();
      if (_dt_report_time.nanoseconds() == 0 || _dt_report_time.get_clock_type() != now.get_clock_type()) {
        _dt_report_time = now;
      } else if ((now - _dt_report_time).seconds() > 30.0) {
        RCLCPP_INFO(
          logger_, "%s: door trust ran on %zu of %zu clouds in the last 30 s (skipped: no map pose %zu, "
          "map pose too old %zu, no door near %zu, robot not between standby nodes %zu, no door from the feed %zu); lower points in the zones %zu, not trusted %.1f%%, "
          "upper points marked %zu", _source_name.c_str(), _dt_applied, _dt_clouds, _dt_skip_tf,
          _dt_skip_age, _dt_skip_far, _dt_skip_trigger, _dt_skip_feed, _dt_low_seen,
          100.0 * _dt_low_dropped / std::max<size_t>(_dt_low_seen, 1), _dt_up_marked);
        _dt_clouds = _dt_applied = _dt_skip_tf = _dt_skip_age = _dt_skip_far = _dt_skip_trigger = _dt_skip_feed = 0;
        _dt_low_seen = _dt_low_dropped = _dt_up_marked = 0;
        _dt_report_time = now;
      }
    };
  const spatio_temporal_voxel_layer::DoorFeed::List door_list =
    _doors ? _doors->Get(clock_->now().seconds(), _door_zone.feed_max_age) : nullptr;
  if (!door_list || door_list->empty()) {
    ++(door_list ? _dt_skip_far : _dt_skip_feed);
    report();
    return false;
  }
  // map -> filter frame = (global -> filter frame at the cloud's time) * (map -> global, latest).
  // map -> global comes from the localizer and lags the lidar, so it is taken at its latest and
  // checked for age rather than looked up at the cloud's time, which would fail on most clouds.
  tf2::Transform map_to_filter;
  try {
    const geometry_msgs::msg::TransformStamped map_to_global =
      _buffer.lookupTransform(_global_frame, _door_zone.map_frame, tf2::TimePointZero);
    const geometry_msgs::msg::TransformStamped global_to_filter =
      _buffer.lookupTransform(_height_filter_frame, _global_frame, tf2_ros::fromMsg(stamp));
    const rclcpp::Time now = clock_->now();
    const rclcpp::Time map_stamp(map_to_global.header.stamp, now.get_clock_type());
    if (map_stamp.nanoseconds() > 0 && std::fabs((now - map_stamp).seconds()) > _door_zone.max_tf_age) {
      ++_dt_skip_age;
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 5000, "%s: %s pose is %.1f s old, door trust off for now", _source_name.c_str(),
        _door_zone.map_frame.c_str(), (now - map_stamp).seconds());
      report();
      return false;
    }
    tf2::Transform a, b;
    tf2::fromMsg(map_to_global.transform, a);
    tf2::fromMsg(global_to_filter.transform, b);
    map_to_filter = b * a;
  } catch (const tf2::TransformException & e) {
    ++_dt_skip_tf;
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 5000, "%s: no %s pose, door trust off for now: %s", _source_name.c_str(),
      _door_zone.map_frame.c_str(), e.what());
    report();
    return false;
  }

  // doors within reach of the lidar, in the filter frame: position, travel direction u, across n
  struct Zone
  {
    double px, py, ux, uy;
  };
  const double ox = cloud_to_filter.transform.translation.x, oy = cloud_to_filter.transform.translation.y;
  const double hw = _door_zone.half_width, hd = _door_zone.half_depth;
  const double range_up = _door_trust.range + _door_trust.depth;
  // the upper band's zone grows by depth (a face up to depth behind a dropped point) plus the
  // sideways reach of the pooled azimuth bins at range_up, so a jamb at the zone edge keeps its column
  const double grow = _door_trust.depth +
    range_up * std::tan((_door_trust.az_neighbours + 1) * _door_trust.az_bin_deg * M_PI / 180.0);
  const double reach = std::hypot(hw + grow, hd + grow) + range_up;
  std::vector<Zone> zones;
  bool near_door = false;
  for (const auto & d : *door_list) {
    const tf2::Vector3 p = map_to_filter * tf2::Vector3(d.x, d.y, 0.0);
    if (std::hypot(p.x() - ox, p.y() - oy) > reach) {
      continue;
    }
    tf2::Vector3 u = map_to_filter.getBasis() * tf2::Vector3(d.ux, d.uy, 0.0);
    const double len = std::hypot(u.x(), u.y());
    if (len < 1e-6) {
      continue;
    }
    const double ux = u.x() / len, uy = u.y() / len;
    near_door = true;
    if (_door_zone.trigger) {
      // the robot (the filter frame's origin) must be on its way through this door: between the
      // standby nodes before and after it, as the robot's own gate procedure is
      const double s_robot = -p.x() * ux - p.y() * uy, l_robot = p.x() * uy - p.y() * ux;
      const double m = _door_zone.trigger_margin;
      if (!d.triggered || s_robot < d.trig_lo - m || s_robot > d.trig_hi + m || std::fabs(l_robot) > hw) {
        continue;
      }
    }
    zones.push_back({p.x(), p.y(), ux, uy});
  }
  if (zones.empty()) {
    ++(near_door ? _dt_skip_trigger : _dt_skip_far);
    report();
    return false;
  }
  const auto in_zone = [&zones, hw, hd](const double x, const double y, const double grow) {
      for (const auto & z : zones) {
        const double dx = x - z.px, dy = y - z.py;
        const double s = dx * z.ux + dy * z.uy, l = -dx * z.uy + dy * z.ux;
        if (std::fabs(s) <= hd + grow && std::fabs(l) <= hw + grow) {
          return true;
        }
      }
      return false;
    };

  // pass 1: per azimuth bin, the nearest upper-band return (the face) and the nearest lower-band
  // return, both within range_up. Neither is gated on the zones: a surface is a surface. The Airy's
  // azimuth jitters by a few degrees across rings, which the pooled neighbour bins absorb. The
  // band limit is inclusive below (z == max_obstacle_height is lower), as the z band keeps it today.
  const double low_max = _max_obstacle_height, up_max = _door_trust.up_max;
  const int nbin = std::max(1, static_cast<int>(std::lround(360.0 / _door_trust.az_bin_deg)));
  const double bin_scale = nbin / 360.0;
  const float inf = std::numeric_limits<float>::infinity();
  std::vector<float> face(nbin, inf), low_min(nbin, inf);
  const auto bin_of = [&](const double dx, const double dy) {
      const int b = static_cast<int>(std::floor((std::atan2(dy, dx) * 180.0 / M_PI + 180.0) * bin_scale));
      return ((b % nbin) + nbin) % nbin;
    };
  {
    sensor_msgs::PointCloud2ConstIterator<float> x(cld, "x"), y(cld, "y"), z(cld, "z");
    for (; x != x.end(); ++x, ++y, ++z) {
      if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z) || *z <= _min_obstacle_height ||
        *z >= up_max)
      {
        continue;
      }
      const double dx = *x - ox, dy = *y - oy, rh = std::sqrt(dx * dx + dy * dy);
      if (rh <= 0.05 || rh >= range_up) {
        continue;
      }
      float & f = (*z > low_max ? face : low_min)[bin_of(dx, dy)];
      f = std::min(f, static_cast<float>(rh));
    }
  }
  const auto pool = [&](std::vector<float> & t) {
      std::vector<float> pooled(t);
      for (int b = 0; b < nbin; ++b) {
        for (int k = 1; k <= _door_trust.az_neighbours; ++k) {
          pooled[b] = std::min({pooled[b], t[(b + k) % nbin], t[((b - k) % nbin + nbin) % nbin]});
        }
      }
      t.swap(pooled);
    };
  if (_door_trust.az_neighbours > 0) {
    pool(face);
    pool(low_min);
  }

  // pass 2 (upper band): it stays only where the lower band has a return within depth of it in
  // the same azimuth (a structure that reaches down into the marked band: a jamb, a leaf stile,
  // a person), inside the zones (grown, so a face at the zone edge keeps its column) and within
  // range_up. A lintel, a sign or anything else with nothing below it is not marked, and
  // everywhere else the upper band goes, as the z band took it before. The kept points are also
  // noted on a voxel-size grid for the lower pass.
  const float nan = std::numeric_limits<float>::quiet_NaN();
  size_t low_seen = 0, low_dropped = 0, up_marked = 0;
  // kept upper returns on a dense cell grid around the lidar (everything the rule touches lies
  // within range_up of it); a hash set here cost several ms per cloud
  const double cell = _voxel_size > 0.0 ? _voxel_size : 0.02;
  const int half = static_cast<int>(std::ceil((range_up + 2.0 * cell) / cell)) + 2, side = 2 * half + 1;
  // per cell, a bit per voxel-size height step of the kept upper returns: a column the grid will
  // mark needs mark_threshold voxels, so one stray return must not count as confirmation
  std::vector<uint32_t> up_grid(static_cast<size_t>(side) * side, 0);
  const auto grid_index = [&](const double x, const double y) {
      const int ix = static_cast<int>(std::floor((x - ox) / cell)) + half;
      const int iy = static_cast<int>(std::floor((y - oy) / cell)) + half;
      return (ix < 0 || ix >= side || iy < 0 || iy >= side) ? -1 : ix * side + iy;
    };
  {
    sensor_msgs::PointCloud2Iterator<float> x(cld, "x"), y(cld, "y"), z(cld, "z");
    for (; x != x.end(); ++x, ++y, ++z) {
      if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z) || *z <= low_max || *z >= up_max) {
        continue;
      }
      const double dx = *x - ox, dy = *y - oy, rh = std::sqrt(dx * dx + dy * dy);
      const float below = rh < range_up ? low_min[bin_of(dx, dy)] : inf;
      if (std::isfinite(below) && std::fabs(rh - below) <= _door_trust.depth && in_zone(*x, *y, grow)) {
        ++up_marked;
        const int gi = grid_index(*x, *y);
        if (gi >= 0) {
          const int step = std::min(31, std::max(0, static_cast<int>(std::floor((*z - low_max) / cell))));
          up_grid[gi] |= (1u << step);
        }
      } else {
        *x = nan;
        *y = nan;
        *z = nan;
      }
    }
  }
  // pass 3 (lower band): a point in a zone within range that sits margin..depth in front of its
  // azimuth's face is not trusted, but only when that face will itself be marked (the support
  // test above) and kept upper returns confirm the surface where the beam says it is: a cell with
  // returns at three heights, within two cells of the point moved out along its beam to the face range. A flat jamb face has its
  // upper returns right there; the thin front edge of a deep reveal, seen by a stray return or
  // two, has them farther in, so those returns stay and the edge keeps its column (without this a
  // jamb edge held by one lidar and one camera voxel lost its face when the lidar voxel went).
  const auto upper_confirms = [&](const double fx, const double fy) {
      const int gi = grid_index(fx, fy);
      if (gi < 0) {
        return false;
      }
      const int ix = gi / side, iy = gi % side;
      for (int da = std::max(ix - 2, 0); da <= std::min(ix + 2, side - 1); ++da) {
        for (int db = std::max(iy - 2, 0); db <= std::min(iy + 2, side - 1); ++db) {
          // three heights, not two: the buffer's grid and the voxel grid's are not aligned
          if (__builtin_popcount(up_grid[static_cast<size_t>(da) * side + db]) >= 3) {
            return true;
          }
        }
      }
      return false;
    };
  sensor_msgs::PointCloud2Iterator<float> x(cld, "x"), y(cld, "y"), z(cld, "z");
  for (; x != x.end(); ++x, ++y, ++z) {
    if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) {
      continue;
    }
    const double h = *z;
    if (h > low_max || h <= _min_obstacle_height) {
      continue;   // upper band done above; below the band the z band decides, as it does today
    }
    const double dx = *x - ox, dy = *y - oy, rh = std::sqrt(dx * dx + dy * dy);
    if (rh <= 0.05 || rh >= _door_trust.range || !in_zone(*x, *y, 0.0)) {
      continue;
    }
    ++low_seen;
    const int b = bin_of(dx, dy);
    const float f = face[b], below = low_min[b];
    if (!std::isfinite(f) || !std::isfinite(below) || std::fabs(f - below) > _door_trust.depth) {
      continue;
    }
    const double gap = f - rh;
    const double depth = h < _door_trust.low_cap_height ? _door_trust.low_cap_depth : _door_trust.depth;
    if (gap > _door_trust.margin && gap < depth && upper_confirms(ox + dx * f / rh, oy + dy * f / rh)) {
      *x = nan;
      *y = nan;
      *z = nan;
      ++low_dropped;
    }
  }
  if (!_dt_first_logged) {
    _dt_first_logged = true;
    RCLCPP_INFO(logger_, "%s: door trust applied to its first cloud (%zu lower points in the zones, %zu not trusted)",
      _source_name.c_str(), low_seen, low_dropped);
  }
  ++_dt_applied;
  _dt_low_seen += low_seen;
  _dt_low_dropped += low_dropped;
  _dt_up_marked += up_marked;
  report();
  return true;
}

/*****************************************************************************/
void MeasurementBuffer::SetSupportFilter(
  const int & min_support, const double & tol, const double & tol_per_m, const bool & wrap_rows)
/*****************************************************************************/
{
  // 8 or more would drop every point and blind the sensor; a negative tolerance likewise
  _support_min = std::min(min_support, 7);
  _support_tol = std::max(tol, 0.0);
  _support_tol_per_m = std::max(tol_per_m, 0.0);
  _support_wrap_rows = wrap_rows;
}

/*****************************************************************************/
size_t MeasurementBuffer::SupportFilter(
  const sensor_msgs::msg::PointCloud2 & raw, sensor_msgs::msg::PointCloud2 & out)
/*****************************************************************************/
{
  // A real surface returns more than one sample: its neighbours in the organized cloud (the
  // next pixel, the next ring or firing) land at about the same range. A raindrop, a speck
  // of dust or a stray return stands alone in front of the background. Ranges are measured
  // from the sensor in the raw cloud; a dropped point becomes NaN in out (same layout).
  // A thin pole keeps its neighbours along its length, so it survives a small min_support.
  const int W = static_cast<int>(raw.width), H = static_cast<int>(raw.height);
  // the pixel <-> (row, column) mapping below assumes rows without padding
  if (raw.row_step != raw.width * raw.point_step || out.row_step != out.width * out.point_step ||
    out.width != raw.width || out.height != raw.height)
  {
    RCLCPP_WARN_ONCE(logger_, "%s: support filter skipped, padded or mismatched rows", _source_name.c_str());
    return 0;
  }
  std::vector<float> rng(static_cast<size_t>(W) * H, std::numeric_limits<float>::quiet_NaN());
  sensor_msgs::PointCloud2ConstIterator<float> rx(raw, "x"), ry(raw, "y"), rz(raw, "z");
  for (size_t i = 0; i < rng.size(); ++i, ++rx, ++ry, ++rz) {
    if (std::isfinite(*rx) && std::isfinite(*ry) && std::isfinite(*rz) &&
      !(*rx == 0.0f && *ry == 0.0f && *rz == 0.0f))
    {
      rng[i] = std::sqrt(*rx * *rx + *ry * *ry + *rz * *rz);
    }
  }
  sensor_msgs::PointCloud2Iterator<float> ox(out, "x"), oy(out, "y"), oz(out, "z");
  const float nan = std::numeric_limits<float>::quiet_NaN();
  size_t dropped = 0;
  for (int r = 0; r < H; ++r) {
    for (int c = 0; c < W; ++c, ++ox, ++oy, ++oz) {
      const float d = rng[static_cast<size_t>(r) * W + c];
      if (!std::isfinite(d)) {
        continue;
      }
      const float tol = static_cast<float>(_support_tol + _support_tol_per_m * d);
      int support = 0;
      for (int dr = -1; dr <= 1 && support < _support_min; ++dr) {
        for (int dc = -1; dc <= 1; ++dc) {
          int rr = r + dr;
          const int cc = c + dc;
          if (_support_wrap_rows) {
            // a spinning lidar's rows are azimuth steps: the last row neighbours the first
            rr = (rr + H) % H;
          }
          if ((dr == 0 && dc == 0) || rr < 0 || rr >= H || cc < 0 || cc >= W) {
            continue;
          }
          const float dn = rng[static_cast<size_t>(rr) * W + cc];
          if (std::isfinite(dn) && std::fabs(dn - d) <= tol) {
            ++support;
          }
        }
      }
      if (support < _support_min) {
        *ox = nan;
        *oy = nan;
        *oz = nan;
        ++dropped;
      }
    }
  }
  // report the drop ratio every 30 s, so a field run shows what the filter costs per sensor
  _support_seen += rng.size() - static_cast<size_t>(std::count_if(
      rng.begin(), rng.end(), [](float v) {return !std::isfinite(v);}));
  _support_dropped += dropped;
  const rclcpp::Time now = clock_->now();
  if (_support_report_time.nanoseconds() == 0 ||
    _support_report_time.get_clock_type() != now.get_clock_type())
  {
    _support_report_time = now;
  } else if ((now - _support_report_time).seconds() > 30.0) {
    RCLCPP_INFO(
      logger_, "%s: support filter dropped %.2f%% of %zu returns in the last 30 s", _source_name.c_str(),
      100.0 * _support_dropped / std::max<size_t>(_support_seen, 1), _support_seen);
    _support_seen = _support_dropped = 0;
    _support_report_time = now;
  }
  return dropped;
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
