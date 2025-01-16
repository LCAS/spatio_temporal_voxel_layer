/*********************************************************************
 *
 * Software License Agreement
 *
 *  Copyright (c) 2018, Simbe Robotics, Inc.
 *  Copyright (c) 2021, Samsung Research America
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
 *                         stevenmacenski@gmail.com
 * Purpose: Replace the ROS voxel grid / obstacle layers using VoxelGrid
 *          with OpenVDB's more efficient and capacble voxel library with
 *          ray tracing and knn.
 *********************************************************************/

#ifndef SPATIO_TEMPORAL_VOXEL_LAYER__SPATIO_TEMPORAL_VOXEL_LAYER_HPP_
#define SPATIO_TEMPORAL_VOXEL_LAYER__SPATIO_TEMPORAL_VOXEL_LAYER_HPP_

// STL
#include <time.h>
#include <vector>
#include <string>
#include <iostream>
#include <memory>
#include <unordered_set>
// voxel grid
#include "spatio_temporal_voxel_layer/spatio_temporal_voxel_grid.hpp"
// ROS
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
// costmap
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/footprint.hpp"
// openVDB
#include "openvdb/openvdb.h"
// msgs
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "spatio_temporal_voxel_layer/srv/save_grid.hpp"
#include "std_srvs/srv/set_bool.hpp"
// projector
#include "laser_geometry/laser_geometry.hpp"
// tf
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/buffer_core.h"
#include "tools/parameters_handler.hpp"
#include "elevation_mapping/input_sources/InputSourceManager.hpp"

#include "spatio_temporal_voxel_layer/elevation_mapping/ElevationMapping.hpp"

namespace spatio_temporal_voxel_layer
{

// conveniences for line lengths
typedef std::vector<
  std::shared_ptr<message_filters::SubscriberBase<rclcpp_lifecycle::LifecycleNode>>
  >::iterator observation_subscribers_iter;
typedef std::vector<std::shared_ptr<MeasurementBuffer>>::iterator observation_buffers_iter;

// Core ROS voxel layer class
class SpatioTemporalVoxelLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  SpatioTemporalVoxelLayer(void);
  virtual ~SpatioTemporalVoxelLayer(void);

  // Core Functions
  virtual void onInitialize(void);
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y);
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j);

  // Functions to interact with other layers
  virtual void matchSize(void);

  // Functions for layer high level operations
  virtual void reset(void);
  virtual void activate(void);
  virtual void deactivate(void);
  virtual void clearArea(int start_x, int start_y, int end_x, int end_y, bool invert_area=false) override;

  virtual bool isClearable() {return true;}

  // Functions for sensor feeds
  bool GetMarkingObservations(std::vector<MeasurementReading> & marking_observations) const;
  bool GetClearingObservations(std::vector<MeasurementReading> & marking_observations) const;
  void ObservationsResetAfterReading() const;


  // Functions to interact with maps
  void UpdateROSCostmap(
    double * min_x, double * min_y, double * max_x, double * max_y,
    std::unordered_set<occupany_cell> & cleared_cells);
  bool updateFootprint(
    double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y, double * max_x, double * max_y);
  void ResetGrid(void);

  // Saving grids callback for openVDB
  void SaveGridCallback(
    const std::shared_ptr<rmw_request_id_t>/*header*/,
    std::shared_ptr<spatio_temporal_voxel_layer::srv::SaveGrid::Request> req,
    std::shared_ptr<spatio_temporal_voxel_layer::srv::SaveGrid::Response> resp);

  std::string getLayerName(){
    return std::string ("grid_mapping_for_elevation");
  }

  inline int toAddress(const Eigen::Vector3i& id) {
    // return id(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) + id(1) * mp_.map_voxel_num_(2) + id(2);
    return getIndex(id(0), id(1));
  }

  
  inline int toAddress(int& x, int& y, int& z) {
    // return x * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) + y * mp_.map_voxel_num_(2) + z;
    return getIndex(x, y);
  }

  
  inline void boundIndex(Eigen::Vector3i& id) {
    Eigen::Vector3i id1;
    const int x_ = (int)id.x();
    const int y_ = (int)id.y();
    id1(0) = std::max(std::min(x_, (int)size_x_ - 1), 0);
    id1(1) = std::max(std::min(y_, (int)size_y_ - 1), 0);
    id1(2) = 1;
    id = id1;
  }

  inline bool isKnownOccupied(const Eigen::Vector3i& id) {
    Eigen::Vector3i id1 = id;
    boundIndex(id1);
    int adr = toAddress(id1);
    return getCost(adr) >= 1;
  }

  
  inline void setOccupied(Eigen::Vector3d pos) {
    if (!isInMap(pos)) return;

    Eigen::Vector3i id;
    posToIndex(pos, id);
    setCost(id(0), id(1), nav2_costmap_2d::LETHAL_OBSTACLE);
    // md_.occupancy_buffer_inflate_[id(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) +
    //                               id(1) * mp_.map_voxel_num_(2) + id(2)] = 1;

  }

  
  inline int getInflateOccupancy(Eigen::Vector3d pos) {
    if (!isInMap(pos)) return -1;
   
    Eigen::Vector3i id;
    posToIndex(pos, id);
    id[2] = 1;
    int adr = toAddress(id);
    // std::cout<< "actul local" << id.transpose() << std::endl;
    int cost = (int)getCost(adr);
    // return int(md_.occupancy_buffer_inflate_[toAddress(id)]);
    if(cost == nav2_costmap_2d::LETHAL_OBSTACLE){
      cost =  1;
    }else{
      cost =  0;
    }
    return cost;
  }

  inline int getInflateOccupancy(const Eigen::Vector3i& idx) {
    if (!isInMap(idx)) return -1;
    int adr = toAddress(idx);
    // std::cout<< "actul local" << id.transpose() << std::endl;
    int cost = (int)getCost(adr);
    // return int(md_.occupancy_buffer_inflate_[toAddress(id)]);
    if(cost == nav2_costmap_2d::LETHAL_OBSTACLE){
      cost =  1;
    }else{
      cost =  0;
    }
    return cost;
    // return int(md_.occupancy_buffer_inflate_[toAddress(idx)]);
  }

  
  inline bool isInMap(const Eigen::Vector3d& pos) {
    unsigned int mx, my;
    bool found_pose = worldToMap(pos(0), pos(1), mx, my);
    if (!found_pose) return false;
    // if (pos(0) < mp_.map_min_boundary_(0) + 1e-4 || pos(1) < mp_.map_min_boundary_(1) + 1e-4 ||
    //     pos(2) < mp_.map_min_boundary_(2) + 1e-4) {
    //   // cout << "less than min range!" << endl;
    //   // RCLCPP_INFO_STREAM(logger_, "pos " << pos.transpose()<< " mp_.map_min_boundary_" << mp_.map_min_boundary_);
    //   return false;
    // }
    // if (pos(0) > mp_.map_max_boundary_(0) - 1e-4 || pos(1) > mp_.map_max_boundary_(1) - 1e-4 ||
    //     pos(2) > mp_.map_max_boundary_(2) - 1e-4) {
    //       // RCLCPP_INFO_STREAM(logger_, "pos " << pos.transpose()<< " mp_.map_max_boundary_" << mp_.map_max_boundary_);
    //   return false;
    // }
    return true;
  }

  
  inline bool isInMap(const Eigen::Vector3i& idx) {
    if (idx(0) < 0 || idx(1) < 0 || idx(2) < 0) {
      // RCLCPP_INFO_STREAM(logger_, "int pos " << idx.transpose()<< " mp_.map_min_boundary_" << mp_.map_min_boundary_);
      return false;
    }
    // if (idx(0) > mp_.map_voxel_num_(0) - 1 || idx(1) > mp_.map_voxel_num_(1) - 1 ||
    //     idx(2) > mp_.map_voxel_num_(2) - 1) {
    //       // RCLCPP_INFO_STREAM(logger_, "int pos " << idx.transpose()<< " mp_.map_voxel_num_" << mp_.map_voxel_num_);
    //   return false;
    // }
    if (idx(0) > size_x_ - 1 || idx(1) > size_y_ - 1) {
          // RCLCPP_INFO_STREAM(logger_, "int pos " << idx.transpose()<< " mp_.map_voxel_num_" << mp_.map_voxel_num_);
      return false;
    }
    return true;
  }

  
  inline void posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id) {
    // for (int i = 0; i < 3; ++i) id(i) = floor((pos(i) - mp_.map_origin_(i)) * mp_.resolution_inv_);
    // id[0] = std::floor((pos(0) -getOriginX()) * getInverseResolution());
    // id[1] = std::floor((pos(1) -getOriginY()) * getInverseResolution());
    // id[2] = 1; 
    unsigned int mx, my;
    bool found_pose = worldToMap(pos(0), pos(1), mx, my);
    id[0] = mx;
    id[1] = my;
    id[2] = 1;
  }

  
  inline void indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos) {
    // for (int i = 0; i < 3; ++i) pos(i) = (id(i) + 0.5) * mp_.resolution_ + mp_.map_origin_(i);
    // pos[0] = (id[0] + 0.5) * getResolution() + getOriginX();
    // pos[1] = (id[1] + 0.5) * getResolution() + getOriginY();
    // pos[2] = 1; 
    double wx, wy;
    unsigned int mx = static_cast<unsigned int>(id(0));
    unsigned int my = static_cast<unsigned int>(id(1));

    mapToWorld(mx, my, wx, wy);
    pos[0] = wx;
    pos[1] = wy;
    pos[2] = 1;

  }

  
  inline void inflatePoint(const Eigen::Vector3i& pt, int step, vector<Eigen::Vector3i>& pts) {
    int num = 0;
    /* ---------- all inflate ---------- */
    for (int x = -step; x <= step; ++x)
      for (int y = -step; y <= step; ++y)
        for (int z = -step; z <= step; ++z) {
          pts[num++] = Eigen::Vector3i(pt(0) + x, pt(1) + y, pt(2) + z);
        }
  }

  double getFreeDistance(Eigen::Vector3d x){
    // TODO 
    return 20.0;
  }

  std::vector<Eigen::Vector3d> getMapCurrentRange(){
    std::vector<Eigen::Vector3d> map_range; //TODO FIX this 
    return map_range;
  }

  std::unique_ptr<SpatioTemporalVoxelGrid> _voxel_grid;

private:
  // Sensor callbacks
  void LaserScanCallback(
    sensor_msgs::msg::LaserScan::ConstSharedPtr message,
    const std::shared_ptr<MeasurementBuffer> & buffer);
  void LaserScanValidInfCallback(
    sensor_msgs::msg::LaserScan::ConstSharedPtr raw_message,
    const std::shared_ptr<MeasurementBuffer> & buffer);
  void PointCloud2Callback(
    sensor_msgs::msg::PointCloud2::ConstSharedPtr message,
    const std::shared_ptr<MeasurementBuffer> & buffer);

  // Functions for adding static obstacle zones
  bool AddStaticObservations(const MeasurementReading & obs);
  bool RemoveStaticObservations(void);

  // Enable/Disable callback
  void BufferEnablerCallback(const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response,
    const std::shared_ptr<MeasurementBuffer> buffer,
    const std::shared_ptr<message_filters::SubscriberBase<rclcpp_lifecycle::LifecycleNode>>
      & subcriber
    );

  /**
   * @brief Callback executed when a paramter change is detected
   * @param parameters list of changed parameters
   */
  rcl_interfaces::msg::SetParametersResult
    dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  laser_geometry::LaserProjection _laser_projector;
  std::vector<std::shared_ptr<message_filters::SubscriberBase<rclcpp_lifecycle::LifecycleNode>>>
    _observation_subscribers;
  std::vector<std::shared_ptr<tf2_ros::MessageFilterBase>> _observation_notifiers;
  std::vector<std::shared_ptr<MeasurementBuffer>> _observation_buffers;
  std::vector<std::shared_ptr<MeasurementBuffer>> _marking_buffers;
  std::vector<std::shared_ptr<MeasurementBuffer>> _clearing_buffers;
  std::vector<rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr> _buffer_enabler_servers;

  bool _publish_voxels, _mapping_mode, was_reset_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _voxel_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_inf_pub_;
  rclcpp::Service<spatio_temporal_voxel_layer::srv::SaveGrid>::SharedPtr _grid_saver;
  std::unique_ptr<rclcpp::Duration> _map_save_duration;
  rclcpp::Time _last_map_save_time;
  std::string _global_frame;
  double _voxel_size, _voxel_decay;
  int _combination_method, _mark_threshold;
  GlobalDecayModel _decay_model;
  bool _update_footprint_enabled, _enabled;
  std::vector<geometry_msgs::msg::Point> _transformed_footprint;
  std::vector<MeasurementReading> _static_observations;
  boost::recursive_mutex _voxel_grid_lock;
  int point_cloud_counter_ = 0;

  // MappingParameters mp_;
  std::string _topics_string;
  std::vector<std::string> extra_mapping_type_;

  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler;
  std::unique_ptr<ParametersHandler> parameters_handler_;
  elevation_mapping::InputSourceManager inputSources_;
  std::shared_ptr<elevation_mapping::ElevationMapping> elevation_mapping_;

  std::shared_ptr<message_filters::Cache<nav_msgs::msg::Odometry>> robotOdometryPoseCache_;
  bool only_edtmapping = false;
  int inf_step_ = 1;
};

}  // namespace spatio_temporal_voxel_layer
#endif  // SPATIO_TEMPORAL_VOXEL_LAYER__SPATIO_TEMPORAL_VOXEL_LAYER_HPP_
