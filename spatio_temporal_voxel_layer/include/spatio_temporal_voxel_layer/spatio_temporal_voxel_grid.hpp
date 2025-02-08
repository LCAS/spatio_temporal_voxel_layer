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
 * Purpose: Implement OpenVDB's voxel library with ray tracing for our
 *          internal voxel grid layer.
 *********************************************************************/

#ifndef SPATIO_TEMPORAL_VOXEL_LAYER__SPATIO_TEMPORAL_VOXEL_GRID_HPP_
#define SPATIO_TEMPORAL_VOXEL_LAYER__SPATIO_TEMPORAL_VOXEL_GRID_HPP_

// STL
#include <math.h>
#include <unordered_map>
#include <unordered_set>
#include <ctime>
#include <iostream>
#include <utility>
#include <vector>
#include <memory>
#include <string>
// PCL
#include "pcl/common/transforms.h"
#include "pcl/PCLPointCloud2.h"
// ROS
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
// msgs
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point32.hpp"
// OpenVDB
#include "openvdb/openvdb.h"
#include "openvdb/tools/GridTransformer.h"
#include "openvdb/tools/RayIntersector.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.hpp>

// measurement struct and buffer
#include "spatio_temporal_voxel_layer/measurement_buffer.hpp"
#include "spatio_temporal_voxel_layer/frustum_models/depth_camera_frustum.hpp"
#include "spatio_temporal_voxel_layer/frustum_models/three_dimensional_lidar_frustum.hpp"
// Mutex and locks
#include "boost/thread.hpp"
#include "boost/thread/recursive_mutex.hpp"

#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <iostream>
#include <random>
#include <nav_msgs/msg/odometry.hpp>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <tuple>
#include <visualization_msgs/msg/marker.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <type_traits>
#include <thread>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <dynamicEDT3D/dynamicEDTOctomap.h>
#include <octomap/octomap.h>

#include <decomp_basis/data_type.h>
// #include <octomap_msgs/conversions.h>

#include <spatio_temporal_voxel_layer/freespace_segmentation/free_space_segment.hpp>
#include <any>

namespace spatio_temporal_voxel_layer
{

using namespace std;
using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

// voxel hashing
template <typename T>
struct matrix_hash : std::unary_function<T, size_t> {
  std::size_t operator()(T const& matrix) const {
    size_t seed = 0;
    for (size_t i = 0; i < matrix.size(); ++i) {
      auto elem = *(matrix.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};


enum GlobalDecayModel
{
  LINEAR = 0,
  EXPONENTIAL = 1,
  PERSISTENT = 2
};

// Structure for an occupied cell for map
struct occupany_cell
{
  occupany_cell(const double & _x, const double & _y)
  : x(_x), y(_y)
  {
  }

  bool operator==(const occupany_cell & other) const
  {
    return x == other.x && y == other.y;
  }

  double x, y;
};

// Structure for wrapping frustum model and necessary metadata
struct frustum_model
{
  frustum_model(Frustum * _frustum, const double & _factor)
  : frustum(_frustum), accel_factor(_factor)
  {
  }
  ~frustum_model()
  {
    if (frustum) {
      delete frustum;
    }
  }
  Frustum * frustum;
  const double accel_factor;
};

// Core voxel grid structure and interface
class SpatioTemporalVoxelGrid
{
public:
  // conveniences for line lengths
  typedef openvdb::math::Ray<openvdb::Real> GridRay;
  typedef openvdb::math::Ray<openvdb::Real>::Vec3T Vec3Type;

  SpatioTemporalVoxelGrid(
    rclcpp::Clock::SharedPtr clock,
    const float & voxel_size, const double & background_value,
    const int & decay_model, const double & voxel_decay,
    const bool & pub_voxels, const int& inf_step, const std::string global_frame);
  ~SpatioTemporalVoxelGrid(void);

  // Core making and clearing functions
  void Mark(const std::vector<MeasurementReading> & marking_observations);
  void operator()(const MeasurementReading & obs);
  void setValue() const;
  void ClearFrustums(
    const std::vector<MeasurementReading> & clearing_observations,
    std::unordered_set<occupany_cell> & cleared_cells);

  // Get the pointcloud of the underlying occupancy grid
  void GetOccupancyPointCloud(std::unique_ptr<sensor_msgs::msg::PointCloud2> & pc2);
  std::unordered_map<occupany_cell, uint> * GetFlattenedCostmap();

  // Clear the grid
  bool ResetGrid(void);
  void ResetGridArea(const occupany_cell & start, const occupany_cell & end, bool invert_area=false);

  // Save the file to file with size information
  bool SaveGrid(const std::string & file_name, double & map_size_bytes);

  // occupancy map management
  void resetBuffer();
  void resetBuffer(Eigen::Vector3d min, Eigen::Vector3d max);
  
  void getConvexSegmentsFreeSpace(const std::vector<geometry_msgs::msg::PoseStamped>& poses, 
        vec_E<Polyhedron<3>>& poly_whole, std::vector<LinearConstraint3D>& l_constraints_whole);

  // void initEDTMap(std::string mapper_name_,  MappingParameters map_params);


  void publishMapInflate(bool all_info = false);
  void processPointCloud();

  void publishUnknown();

   // get depth image and camera pose
  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& odom);
  
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_inf_pub_;

  // typedef std::shared_ptr<SpatioTemporalVoxelGrid> Ptr;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
protected:
  // Initialize grid metadata and library
  void InitializeGrid(void);

  // grid accessor methods
  bool MarkGridPoint(const openvdb::Coord & pt, const double & value) const;
  bool ClearGridPoint(const openvdb::Coord & pt) const;

  // Check occupancy status of the grid
  bool IsGridEmpty(void) const;

  // Get time information for clearing
  double GetTemporalClearingDuration(const double & time_delta);
  double GetFrustumAcceleration(
    const double & time_delta, const double & acceleration_factor);
  void TemporalClearAndGenerateCostmap(
    std::vector<frustum_model> & frustums,
    std::unordered_set<occupany_cell> & cleared_cells);

  // Populate the costmap ROS api and pointcloud with a marked point
  void PopulateCostmapAndPointcloud(const openvdb::Coord & pt);

  // Utilities for tranformation
  openvdb::Vec3d WorldToIndex(const openvdb::Vec3d & coord) const;
  openvdb::Vec3d IndexToWorld(const openvdb::Coord & coord) const;

  rclcpp::Clock::SharedPtr _clock;

  mutable openvdb::DoubleGrid::Ptr _grid;
  int _decay_model;
  double _background_value, _voxel_size, _voxel_decay;
  bool _pub_voxels;
  std::unique_ptr<std::vector<geometry_msgs::msg::Point32>> _grid_points;
  std::unordered_map<occupany_cell, uint> * _cost_map;
  boost::mutex _grid_lock;

  std::string mapper_name_;
  // Clock
  rclcpp::Clock::SharedPtr clock_;
  // Logger
  rclcpp::Logger logger_{rclcpp::get_logger("edtmapping")};
  std::shared_ptr<std::thread> pointcloud_processor;
 
  uniform_real_distribution<double> rand_noise_;
  normal_distribution<double> rand_noise2_;
  default_random_engine eng_;
  int max_queue_size_ = 1;

  FreeSpaceSegmentManager::Ptr free_seg_manager;
  vec_E<Polyhedron<3>> polyhedra_;
  std::vector<LinearConstraint3D> l_constraints_whole_; 
  rclcpp::Publisher<decomp_ros_msgs::msg::PolyhedronArray>::SharedPtr poly_whole_pub_;

  std::shared_ptr<vec_Vec3f> vec_obs_;
  vec_Vec3f processed_obs_; 
  int inf_step_ = 1;
  std::string global_frame_;


};

}  // namespace volume_grid

// hash function for unordered_map of occupancy_cells
namespace std
{
template<>
struct hash<spatio_temporal_voxel_layer::occupany_cell>
{
  std::size_t operator()(const spatio_temporal_voxel_layer::occupany_cell & k) const
  {
    return (std::hash<double>()(k.x) ^ (std::hash<double>()(k.y) << 1)) >> 1;
  }
};

}  // namespace std

#endif  // SPATIO_TEMPORAL_VOXEL_LAYER__SPATIO_TEMPORAL_VOXEL_GRID_HPP_
