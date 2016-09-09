/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Kentaro Wada.
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
 *   * Neither the name of the Willow Garage nor the names of its
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
 *********************************************************************/

#ifndef OCTOMAP_SERVER_LABELOCTOMAPSERVER_H
#define OCTOMAP_SERVER_LABELOCTOMAPSERVER_H

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/ColorRGBA.h>

#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <dynamic_reconfigure/server.h>
#include <octomap_server/OctomapServerConfig.h>

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/BoundingBoxQuery.h>
#include <octomap_msgs/conversions.h>

#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <octomap/LabelOccupancyOcTree.h>
#include <octomap/OcTreeKey.h>

#include <algorithm>
#include <string>

namespace octomap_server
{

class LabelOctomapServer
{
public:
  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::PointCloud2, sensor_msgs::Image> ApproximateSyncPolicy;

  LabelOctomapServer();
  virtual ~LabelOctomapServer();
  bool clearBBXSrv(octomap_msgs::BoundingBoxQuery::Request& req, octomap_msgs::BoundingBoxQuery::Response& res);
  bool resetSrv(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  virtual void insertCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud, const sensor_msgs::Image::ConstPtr& imgmsg);
  virtual bool openFile(const std::string& filename);

protected:
  inline static void updateMinKey(const octomap::OcTreeKey& in, octomap::OcTreeKey& min)
  {
    for (unsigned i = 0; i < 3; ++i)
    {
      min[i] = std::min(in[i], min[i]);
    }
  };

  inline static void updateMaxKey(const octomap::OcTreeKey& in, octomap::OcTreeKey& max)
  {
    for (unsigned i = 0; i < 3; ++i)
    {
      max[i] = std::max(in[i], max[i]);
    }
  };

  /// Test if key is within update area of map (2D, ignores height)
  inline bool isInUpdateBBX(const octomap::LabelOccupancyOcTree::iterator& it) const
  {
    // 2^(tree_depth-depth) voxels wide:
    unsigned voxel_width = (1 << (max_tree_depth_ - it.getDepth()));
    octomap::OcTreeKey key = it.getIndexKey(); // lower corner of voxel
    return (key[0] + voxel_width >= update_bbx_min_[0] &&
            key[1] + voxel_width >= update_bbx_min_[1] &&
            key[0] <= update_bbx_max_[0] &&
            key[1] <= update_bbx_max_[1]);
  }

  void periodicalPublishCallback(const ros::TimerEvent& event);
  void reconfigureCallback(octomap_server::OctomapServerConfig& config, uint32_t level);
  virtual void publishAll(const ros::Time& rostime = ros::Time::now());

  /**
    * @brief update occupancy map with a scan labeled as ground and nonground.
    * The scans should be in the global map frame.
    *
    * @param ground scan endpoints on the ground plane (only clear space)
    * @param nonground all other endpoints (clear up to occupied endpoint)
    */
  virtual void insertScan(const sensor_msgs::PointCloud2::ConstPtr& cloud,
                          const sensor_msgs::Image::ConstPtr& imgmsg);

  /**
    * @brief Find speckle nodes (single occupied voxels with no neighbors). Only works on lowest resolution!
    * @param key
    * @return
    */
  bool isSpeckleNode(const octomap::OcTreeKey& key) const;

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher pub_marker_;
  ros::Publisher pub_fmarker_;
  ros::Publisher pub_point_cloud_;
  message_filters::Subscriber<sensor_msgs::PointCloud2>* sub_point_cloud_;
  message_filters::Subscriber<sensor_msgs::Image>* sub_obj_proba_img_;
  ros::ServiceServer srv_clear_bbx_;
  ros::ServiceServer srv_reset_;
  tf::TransformListener tf_listener_;
  boost::recursive_mutex config_mutex_;
  dynamic_reconfigure::Server<OctomapServerConfig> reconfigure_server_;

  octomap::LabelOccupancyOcTree* octree_;
  octomap::KeyRay key_ray_;  // temp storage for ray casting
  octomap::OcTreeKey update_bbx_min_;
  octomap::OcTreeKey update_bbx_max_;

  double max_range_;
  std::string world_frame_id_;  // the map frame
  std_msgs::ColorRGBA color_;
  std_msgs::ColorRGBA color_free_;

  bool latched_topics_;
  double publish_rate_;

  ros::Timer timer_periodical_publish_;

  double resolution_;
  int n_label_;
  unsigned tree_depth_;
  unsigned max_tree_depth_;

  double occupancy_min_z_;
  double occupancy_max_z_;
  double min_size_x_;
  double min_size_y_;
  bool filter_speckles_;

  message_filters::Synchronizer<ApproximateSyncPolicy>* async_;

  bool compress_map_;

  bool init_config_;
};

}  // namespace octomap_server

#endif  // OCTOMAP_SERVER_LABELOCTOMAPSERVER_H
