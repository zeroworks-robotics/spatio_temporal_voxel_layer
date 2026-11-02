/********************************************************************
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
 * Purpose: buffer incoming measurements for grid
 *********************************************************************/

#ifndef SPATIO_TEMPORAL_VOXEL_LAYER__MEASUREMENT_BUFFER_HPP_
#define SPATIO_TEMPORAL_VOXEL_LAYER__MEASUREMENT_BUFFER_HPP_

// STL
#include <vector>
#include <list>
#include <string>
#include <chrono>
#include <memory>
// measurement structs
#include "spatio_temporal_voxel_layer/measurement_reading.h"
#include "spatio_temporal_voxel_layer/ground_columns.hpp"
#include "spatio_temporal_voxel_layer/ground_reference.hpp"
#include "spatio_temporal_voxel_layer/door_zones.hpp"
// PCL
#include "pcl/common/transforms.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/passthrough.h"
// ROS
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
// TF
#include "tf2_ros/buffer.h"
#include "message_filters/subscriber.h"
// msgs
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
// Mutex
#include "boost/thread.hpp"

namespace buffer
{

enum class Filters
{
  NONE = 0,
  VOXEL = 1,
  PASSTHROUGH = 2
};

// conveniences for line lengths
typedef std::list<observation::MeasurementReading>::iterator readings_iter;
typedef std::unique_ptr<sensor_msgs::msg::PointCloud2> point_cloud_ptr;
typedef std::shared_ptr<const std::vector<spatio_temporal_voxel_layer::Door>> door_list_ptr;

// Door ray trust, a lidar marking source only (see MeasurementBuffer::DoorTrustFilter).
// Heights are in height_filter_frame (base_link); ranges are horizontal, from the lidar.
struct DoorTrustParams
{
  bool enabled{false};
  double up_max{1.8};          // top of the upper band [max_obstacle_height, up_max) marked in door zones (m)
  double range{1.3};           // the rule applies to lower points within this range (m)
  double margin{0.01};         // a lower point this much nearer than the upper face is still trusted (m)
  double depth{0.10};          // ... and one more than this much nearer is another object (m)
  double az_bin_deg{1.0};      // azimuth bin of the face table (deg)
  int az_neighbours{1};        // neighbour bins pooled into a face (jitter of the lidar's azimuths)
  double low_cap_height{0.90}; // lower points below this height (m, the robot's body height plus 10 cm) ...
  double low_cap_depth{0.03};  // ... use this depth instead
};

// Door zones of the map (DoorTrimLayer uses the same definition): |s| <= half_depth along the
// travel direction through the node, |l| <= half_width across it.
struct DoorZoneParams
{
  std::string map_frame{"map"};
  double half_width{1.0};
  double half_depth{0.8};
  double max_tf_age{2.0};      // oldest map -> global transform the rule accepts (s)
  // the rule runs at a door only while the robot is between its standby nodes (Door::trig_*),
  // widened by trigger_margin; false = whenever the lidar sees the zone
  bool trigger{false};
  double trigger_margin{0.10};
};

// Measurement buffer
class MeasurementBuffer
{
public:
  MeasurementBuffer(
    const std::string & source_name,
    const std::string & topic_name,
    const double & observation_keep_time,
    const double & expected_update_rate,
    const double & min_obstacle_height,
    const double & max_obstacle_height,
    const double & obstacle_range,
    tf2_ros::Buffer & tf,
    const std::string & global_frame,
    const std::string & sensor_frame,
    const double & tf_tolerance,
    const double & min_d,
    const double & max_d,
    const double & vFOV,
    const double & vFOVPadding,
    const double & hFOV,
    const double & decay_acceleration,
    const bool & marking,
    const bool & clearing,
    const double & voxel_size,
    const Filters & filter,
    const int & voxel_min_points,
    const bool & enabled,
    const bool & clear_buffer_after_reading,
    const ModelType & model_type,
    const std::string & height_filter_frame,
    const bool & ground_relative_height, const double & ground_min_obstacle_height,
    const double & ground_seed_z_tol, const double & ground_seed_z_tol_per_m,
    const bool & ground_seed_gravity_aligned,
    const double & ground_max_grade_deg,
    const double & ground_height_tol, const double & ground_height_tol_per_m,
    const bool & ground_reference_publish, const bool & ground_reference_use,
    std::shared_ptr<ground_seg::GroundReference> ground_reference,
    rclcpp::Clock::SharedPtr clock,
    rclcpp::Logger logger);

  ~MeasurementBuffer(void);

  // Buffers for different types of pointclouds
  void BufferROSCloud(const sensor_msgs::msg::PointCloud2 & cloud);

  // Get measurements from the buffer
  void GetReadings(std::vector<observation::MeasurementReading> & observations);

  // enabler setter getter
  bool IsEnabled(void) const;
  void SetEnabled(const bool & enabled);

  // Source name getter
  std::string GetSourceName(void) const;

  // params setters
  void SetMinObstacleHeight(const double & min_obstacle_height);
  void SetMaxObstacleHeight(const double & max_obstacle_height);
  void SetMinZ(const double & min_z);
  void SetMaxZ(const double & max_z);
  void SetVerticalFovPadding(const double & vertical_fov_padding);
  void SetHorizontalFovAngle(const double & horizontal_fov_angle);
  void SetVerticalFovAngle(const double & vertical_fov_angle);
  // Drop marking points with fewer than min_support of their 8 neighbours in the organized
  // cloud at about the same range (tol + tol_per_m * range); -1 = off. See SupportFilter.
  void SetSupportFilter(
    const int & min_support, const double & tol, const double & tol_per_m, const bool & wrap_rows);
  // Lifetime of this source's marks, s, shorter than the layer's voxel_decay; negative = layer's.
  void SetVoxelDecay(const double & voxel_decay);
  // The short life applies within range (m) of the sensor only; negative = to all points.
  void SetVoxelDecayRange(const double & range);
  // Near-range bias of the sensor: each return is pushed out along its beam by min(k / range, max) (m),
  // horizontally only (z kept). Needs the cloud in the frame of a near-level sensor. k <= 0 = off.
  void SetRangeBias(const double & k, const double & max);
  // Depth-camera edge pixels: in each image row, a pixel next (within px columns) to a much farther or
  // missing neighbour is dropped when its surface is at least min_run pixels wide. px <= 0 = off.
  // See EdgeFilter.
  void SetEdgeFilter(const int & px, const int & min_run);
  // Door ray trust: in the map's door zones, a lower-band point that reads 0..depth nearer than the
  // upper band in the same azimuth is not marked, and the upper band is marked there. `doors` may be
  // empty (the rule then never runs). See DoorTrustFilter.
  void SetDoorTrust(const DoorTrustParams & params, const DoorZoneParams & zones, door_list_ptr doors);
  bool DoorTrustEnabled(void) const {return _door_trust.enabled;}
  // Switches a configured door trust rule off and on at run time (parameter door_trust).
  void SetDoorTrustOn(const bool & on) {_door_trust_on = on;}
  // the parameters as clamped by SetDoorTrust
  const DoorTrustParams & DoorTrust(void) const {return _door_trust;}

  // State knoweldge if sensors are operating as expected
  bool UpdatedAtExpectedRate(void) const;
  void ResetLastUpdatedTime(void);
  void ResetAllMeasurements(void);
  bool ClearAfterReading(void);

  // Public mutex locks
  void Lock(void);
  void Unlock(void);

private:
  // Removing old observations from buffer
  void RemoveStaleObservations(void);

  tf2_ros::Buffer & _buffer;
  const rclcpp::Duration _observation_keep_time, _expected_update_rate;
  rclcpp::Time _last_updated;
  boost::recursive_mutex _lock;
  std::string _global_frame, _sensor_frame, _source_name, _topic_name;
  std::list<observation::MeasurementReading> _observation_list;
  double _min_obstacle_height, _max_obstacle_height, _obstacle_range, _tf_tolerance;
  double _min_z, _max_z, _vertical_fov, _vertical_fov_padding, _horizontal_fov;
  double _decay_acceleration, _voxel_size;
  bool _marking, _clearing;
  Filters _filter;
  int _voxel_min_points;
  bool _clear_buffer_after_reading, _enabled;
  ModelType _model_type;
  // Frame the min/max_obstacle_height band is measured in. Empty means the global frame,
  // which is the upstream behaviour; naming base_link ties the band to the chassis so it
  // tilts with the robot on a ramp instead of staying level with the world.
  // Drop ground and hole points and gate the rest on height above the ground beneath
  // them. Returns false if it could not run, leaving `cld` untouched.
  bool FilterGroundRelative(point_cloud_ptr & cld) const;
  size_t SupportFilter(
    const sensor_msgs::msg::PointCloud2 & raw, sensor_msgs::msg::PointCloud2 & out);
  size_t EdgeFilter(
    const sensor_msgs::msg::PointCloud2 & raw, sensor_msgs::msg::PointCloud2 & out);
  // Lifetime of this source's marks, s; negative = the layer's voxel_decay. See SetVoxelDecay.
  double _source_voxel_decay{-1.0}, _source_voxel_decay_range{-1.0};
  double _range_bias_k{0.0}, _range_bias_max{0.0};
  int _edge_px{0}, _edge_min_run{4};
  size_t _edge_seen{0}, _edge_dropped{0};
  rclcpp::Time _edge_report_time{0, 0, RCL_ROS_TIME};
  int _support_min{-1};
  double _support_tol{0.03}, _support_tol_per_m{0.02};
  bool _support_wrap_rows{false};
  // points seen / dropped by the support filter since the last report
  size_t _support_seen{0}, _support_dropped{0};
  rclcpp::Time _support_report_time{0, 0, RCL_ROS_TIME};
  // Door ray trust on the organized cloud in height_filter_frame. Returns true when the rule ran
  // on this cloud (its upper band then stays in the cloud and the z band must reach up_max);
  // false leaves the cloud untouched, which is today's behaviour.
  bool DoorTrustFilter(
    sensor_msgs::msg::PointCloud2 & cld, const geometry_msgs::msg::TransformStamped & cloud_to_filter,
    const builtin_interfaces::msg::Time & stamp, const std::string & cloud_frame);
  DoorTrustParams _door_trust;
  bool _door_trust_on{true};
  DoorZoneParams _door_zone;
  door_list_ptr _doors;
  // door trust counters since the last report: clouds seen / rule ran / skipped by reason
  size_t _dt_clouds{0}, _dt_applied{0}, _dt_skip_tf{0}, _dt_skip_age{0}, _dt_skip_far{0}, _dt_skip_trigger{0};
  size_t _dt_low_seen{0}, _dt_low_dropped{0}, _dt_up_marked{0};
  rclcpp::Time _dt_report_time{0, 0, RCL_ROS_TIME};
  bool _dt_first_logged{false};
  // Ground height at `range` from an ascending (range, z) profile.
  static float NearestGroundZ(
    const std::vector<std::pair<float, float>> & profile, const float & range);

  std::string _height_filter_frame;
  // When true, min/max_obstacle_height are measured from the ground found beneath each
  // point rather than from _height_filter_frame's origin. See the ramp note in
  // ground_columns.hpp; requires an ORGANIZED cloud and falls back to the fixed band
  // with a warning when it does not get one.
  bool _ground_relative_height;
  // The band the gate applies, measured from the ground under each point rather than from
  // height_filter_frame's origin. Separate from _min_obstacle_height because the two mean
  // different things: 0.05 above the local floor is a sensitivity, 0.05 above base_link is
  // most of this rig's floor. Keeping one number for both would make turning the gate off
  // silently change what the other number means.
  double _ground_min_obstacle_height;
  // How far from the expected floor a sample may sit and still seed its column. The walk
  // cannot start without a seed, and a column that never seeds falls back to the fixed
  // band -- so this is the parameter that decides whether the gate is in play at all when
  // the floor is not where the robot expects it.
  double _ground_seed_z_tol;
  double _ground_seed_z_tol_per_m;
  bool _ground_seed_gravity_aligned;
  double _ground_max_grade_deg;
  double _ground_height_tol;
  double _ground_height_tol_per_m;
  // Shared across every buffer in the layer, so a camera that can see flat floor
  // beneath the robot can seed one that cannot. See ground_reference.hpp.
  bool _ground_reference_publish;
  bool _ground_reference_use;
  std::shared_ptr<ground_seg::GroundReference> _ground_reference;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_;
};

}  // namespace buffer

#endif  // SPATIO_TEMPORAL_VOXEL_LAYER__MEASUREMENT_BUFFER_HPP_
