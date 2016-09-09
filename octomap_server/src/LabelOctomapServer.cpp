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

#include <algorithm>
#include <limits>
#include <string>

#include <cv_bridge/rgb_colors.h>

#include <octomap_server/LabelOctomapServer.h>

namespace octomap_server
{

namespace label_octomap_server
{
  bool is_equal(double a, double b, double epsilon = 1.0e-7)
  {
    return std::abs(a - b) < epsilon;
  }
}  // namespace label_octomap_server

LabelOctomapServer::LabelOctomapServer() :
  nh_(),
  pnh_(ros::NodeHandle("~")),
  sub_point_cloud_(NULL),
  sub_obj_proba_img_(NULL),
  reconfigure_server_(config_mutex_),
  octree_(NULL),
  max_range_(-1.0),
  world_frame_id_("/map"),
  color_factor_(0.8),
  publish_rate_(0),
  latched_topics_(true),
  resolution_(0.05),
  n_label_(0),
  tree_depth_(0),
  max_tree_depth_(0),
  occupancy_min_z_(-std::numeric_limits<double>::max()),
  occupancy_max_z_(std::numeric_limits<double>::max()),
  min_size_x_(0.0),
  min_size_y_(0.0),
  filter_speckles_(false),
  compress_map_(true),
  init_config_(true)
{
  pnh_.param("frame_id", world_frame_id_, world_frame_id_);
  pnh_.param("color_factor", color_factor_, color_factor_);

  pnh_.param("occupancy_min_z", occupancy_min_z_, occupancy_min_z_);
  pnh_.param("occupancy_max_z", occupancy_max_z_, occupancy_max_z_);
  pnh_.param("min_x_size", min_size_x_, min_size_x_);
  pnh_.param("min_y_size", min_size_y_, min_size_y_);
  pnh_.param("filter_speckles", filter_speckles_, filter_speckles_);
  pnh_.param("resolution", resolution_, resolution_);
  if (!pnh_.hasParam("n_label"))
  {
    ROS_FATAL("Rosparam '~n_label' must be set.");
    exit(1);
  }
  pnh_.getParam("n_label", n_label_);

  double prob_hit;
  double prob_miss;
  double threshold_min;
  double threshold_max;
  pnh_.param("sensor_model/hit", prob_hit, 0.7);
  pnh_.param("sensor_model/miss", prob_miss, 0.4);
  pnh_.param("sensor_model/min", threshold_min, 0.12);
  pnh_.param("sensor_model/max", threshold_max, 0.97);
  pnh_.param("sensor_model/max_range", max_range_, max_range_);

  pnh_.param("compress_map", compress_map_, compress_map_);

  // initialize octomap object & params
  octree_ = new octomap::LabelOccupancyOcTree(resolution_, n_label_);
  octree_->setProbHit(prob_hit);
  octree_->setProbMiss(prob_miss);
  octree_->setClampingThresMin(threshold_min);
  octree_->setClampingThresMax(threshold_max);
  tree_depth_ = octree_->getTreeDepth();
  max_tree_depth_ = tree_depth_;

  double r, g, b, a;
  pnh_.param("color/r", r, 0.0);
  pnh_.param("color/g", g, 0.0);
  pnh_.param("color/b", b, 1.0);
  pnh_.param("color/a", a, 1.0);
  color_.r = r;
  color_.g = g;
  color_.b = b;
  color_.a = a;

  pnh_.param("color_free/r", r, 0.0);
  pnh_.param("color_free/g", g, 1.0);
  pnh_.param("color_free/b", b, 0.0);
  pnh_.param("color_free/a", a, 1.0);
  color_free_.r = r;
  color_free_.g = g;
  color_free_.b = b;
  color_free_.a = a;

  pnh_.param("latch", latched_topics_, latched_topics_);
  if (latched_topics_)
  {
    ROS_INFO("Publishing latched (single publish will take longer, all topics are prepared)");
  }
  else
  {
    ROS_INFO("Publishing non-latched (topics are only prepared as needed, will only be re-published on map change");
  }

  pnh_.param("publish_rate", publish_rate_, publish_rate_);
  if (publish_rate_ > 0)
  {
    timer_periodical_publish_ = pnh_.createTimer(
        ros::Duration(1.0 / publish_rate_),
        &LabelOctomapServer::periodicalPublishCallback,
        this,
        /*oneshot=*/false);
  }

  pub_marker_ = nh_.advertise<visualization_msgs::MarkerArray>("occupied_cells_vis_array", 1, latched_topics_);
  pub_fmarker_ = nh_.advertise<visualization_msgs::MarkerArray>("free_cells_vis_array", 1, latched_topics_);
  pub_point_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("octomap_point_cloud_centers", 1, latched_topics_);

  sub_point_cloud_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, "cloud_in", 5);
  sub_obj_proba_img_ = new message_filters::Subscriber<sensor_msgs::Image>(nh_, "proba_image_in", 5);
  async_ = new message_filters::Synchronizer<ApproximateSyncPolicy>(100);
  async_->connectInput(*sub_point_cloud_, *sub_obj_proba_img_);
  async_->registerCallback(boost::bind(&LabelOctomapServer::insertCallback, this, _1, _2));

  srv_clear_bbx_ = pnh_.advertiseService("clear_bbx", &LabelOctomapServer::clearBBXSrv, this);
  srv_reset_ = pnh_.advertiseService("reset", &LabelOctomapServer::resetSrv, this);

  reconfigure_server_.setCallback(boost::bind(&LabelOctomapServer::reconfigureCallback, this, _1, _2));
}

LabelOctomapServer::~LabelOctomapServer()
{
  if (sub_obj_proba_img_)
  {
    delete sub_obj_proba_img_;
    sub_obj_proba_img_ = NULL;
  }

  if (sub_point_cloud_)
  {
    delete sub_point_cloud_;
    sub_point_cloud_ = NULL;
  }

  if (octree_)
  {
    delete octree_;
    octree_ = NULL;
  }
}

bool LabelOctomapServer::openFile(const std::string& filename)
{
  if (filename.length() <= 3)
  {
    return false;
  }

  std::string suffix = filename.substr(filename.length()-3, 3);
  if (suffix== ".bt")
  {
    if (!octree_->readBinary(filename))
    {
      return false;
    }
  }
  else if (suffix == ".ot")
  {
    octomap::AbstractOcTree* tree = octomap::AbstractOcTree::read(filename);
    if (!tree)
    {
      return false;
    }
    if (octree_)
    {
      delete octree_;
      octree_ = NULL;
    }
    octree_ = dynamic_cast<octomap::LabelOccupancyOcTree*>(tree);
    if (!octree_)
    {
      ROS_ERROR("Could not read OcTree in file, currently there are no other types supported in .ot");
      return false;
    }
  }
  else
  {
    return false;
  }

  ROS_INFO("Octomap file %s loaded (%zu nodes).", filename.c_str(), octree_->size());

  tree_depth_ = octree_->getTreeDepth();
  max_tree_depth_ = tree_depth_;
  resolution_ = octree_->getResolution();
  double min_x, min_y, min_z;
  double max_x, max_y, max_z;
  octree_->getMetricMin(min_x, min_y, min_z);
  octree_->getMetricMax(max_x, max_y, max_z);

  update_bbx_min_[0] = octree_->coordToKey(min_x);
  update_bbx_min_[1] = octree_->coordToKey(min_y);
  update_bbx_min_[2] = octree_->coordToKey(min_z);

  update_bbx_max_[0] = octree_->coordToKey(max_x);
  update_bbx_max_[1] = octree_->coordToKey(max_y);
  update_bbx_max_[2] = octree_->coordToKey(max_z);

  publishAll();

  return true;
}

void LabelOctomapServer::insertCallback(
    const sensor_msgs::PointCloud2::ConstPtr& cloud,
    const sensor_msgs::Image::ConstPtr& imgmsg)
{
  insertScan(cloud, imgmsg);
  publishAll(cloud->header.stamp);
}

void LabelOctomapServer::insertScan(
    const sensor_msgs::PointCloud2::ConstPtr& cloud,
    const sensor_msgs::Image::ConstPtr& imgmsg)
{
  if (!(cloud->height == imgmsg->height && cloud->width == imgmsg->width)) {
    ROS_ERROR("Input point cloud and image has different size. cloud: (%d, %d), image: (%d, %d)",
              cloud->width, cloud->height, imgmsg->width, imgmsg->height);
    return;
  }

  // Get transform of sensor to the world
  tf::StampedTransform sensor_to_world_tf;
  try
  {
    tf_listener_.lookupTransform(world_frame_id_, cloud->header.frame_id, cloud->header.stamp, sensor_to_world_tf);
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting callback");
    return;
  }

  // Get sensor origin
  tf::Point sensor_origin_tf = sensor_to_world_tf.getOrigin();
  octomap::point3d sensor_origin = octomap::pointTfToOctomap(sensor_origin_tf);

  pcl::PointCloud<pcl::PointXYZ> pc;
  pcl::fromROSMsg(*cloud, pc);

  // transform clouds to world frame for insertion
  Eigen::Matrix4f sensor_to_world;
  pcl_ros::transformAsMatrix(sensor_to_world_tf, sensor_to_world);
  pcl::transformPointCloud(pc, pc, sensor_to_world);

  cv::Mat proba_img = cv_bridge::toCvCopy(imgmsg, imgmsg->encoding)->image;

  if (n_label_ != proba_img.channels())
  {
    ROS_ERROR("Number of labels and channels must be same. label: %d, probability image channel: %d",
              n_label_, proba_img.channels());
    return;
  }


  if (!octree_->coordToKeyChecked(sensor_origin, update_bbx_min_) ||
      !octree_->coordToKeyChecked(sensor_origin, update_bbx_max_))
  {
    ROS_ERROR_STREAM("Could not generate Key for origin " << sensor_origin);
  }

  // instead of direct scan insertion, compute update to filter ground:
  octomap::KeySet free_cells;
  octomap::KeySet occupied_cells;

  // all other points: free on ray, occupied on endpoint:
  for (size_t index = 0; index < pc.points.size(); index++)
  {
    int width_index = index % cloud->width;
    int height_index = index / cloud->width;
    if (isnan(pc.points[index].x) || isnan(pc.points[index].y) || isnan(pc.points[index].z))
    {
      // Skip NaN points
      continue;
    }
    octomap::point3d point(pc.points[index].x, pc.points[index].y, pc.points[index].z);
    // maxrange check
    if ((max_range_ < 0.0) || ((point - sensor_origin).norm() <= max_range_))
    {
      // free cells
      if (octree_->computeRayKeys(sensor_origin, point, key_ray_))
      {
        free_cells.insert(key_ray_.begin(), key_ray_.end());
      }
      // occupied endpoint
      octomap::OcTreeKey key;
      if (octree_->coordToKeyChecked(point, key))
      {
        occupied_cells.insert(key);

        // update log_odds for the voxel
        std::valarray<float> log_odds(proba_img.channels());
        for (int channel_index=0; channel_index < proba_img.channels(); channel_index++)
        {
          // float label_probability = proba_img.at<float>(height_index, width_index);
          float label_probability = proba_img.ptr<float>(height_index)[proba_img.channels() * width_index
                                                                       + channel_index];
          log_odds[channel_index] = octomap::logodds(static_cast<double>(label_probability));
        }
        octree_->updateNode(key, log_odds);

        updateMinKey(key, update_bbx_min_);
        updateMaxKey(key, update_bbx_max_);
      }
    }
    else
    {
      // ray longer than maxrange:
      octomap::point3d new_end = sensor_origin + (point - sensor_origin).normalized() * max_range_;
      if (octree_->computeRayKeys(sensor_origin, new_end, key_ray_))
      {
        free_cells.insert(key_ray_.begin(), key_ray_.end());

        octomap::OcTreeKey end_key;
        if (octree_->coordToKeyChecked(new_end, end_key))
        {
          updateMinKey(end_key, update_bbx_min_);
          updateMaxKey(end_key, update_bbx_max_);
        }
        else
        {
          ROS_ERROR_STREAM("Could not generate Key for endpoint " << new_end);
        }
      }
    }
  }

  for (octomap::KeySet::iterator it = free_cells.begin(), end=free_cells.end(); it!= end; ++it)
  {
    if (occupied_cells.find(*it) == occupied_cells.end())
    {
      octree_->updateNode(*it, false);
    }
  }

  // TODO: eval lazy+updateInner vs. proper insertion
  // non-lazy by default (updateInnerOccupancy() too slow for large maps)
  // octree_->updateInnerOccupancy();
  octomap::point3d min_point, max_point;
  ROS_DEBUG_STREAM(
    "Bounding box keys (before): " << update_bbx_min_[0] << " " <<update_bbx_min_[1] << " " <<
    update_bbx_min_[2] << " / " <<update_bbx_max_[0] << " "<< update_bbx_max_[1] << " " << update_bbx_max_[2]);

  // TODO: we could also limit the bbx to be within the map bounds here (see publishing check)
  min_point = octree_->keyToCoord(update_bbx_min_);
  max_point = octree_->keyToCoord(update_bbx_max_);
  ROS_DEBUG_STREAM("Updated area bounding box: " << min_point << " - " << max_point);
  ROS_DEBUG_STREAM(
    "Bounding box keys (after): " << update_bbx_min_[0] << " " <<update_bbx_min_[1] << " " <<
    update_bbx_min_[2] << " / " <<update_bbx_max_[0] << " " << update_bbx_max_[1] << " " << update_bbx_max_[2]);

  if (compress_map_)
  {
    octree_->prune();
  }
}

void LabelOctomapServer::periodicalPublishCallback(const ros::TimerEvent& event)
{
  publishAll(event.current_real);
}

// TODO(wkentro): Implementation
void LabelOctomapServer::publishAll(const ros::Time& rostime)
{
  ros::WallTime start_time = ros::WallTime::now();

  // TODO: estimate num occ. voxels for size of arrays (reserve)
  if (octree_->size() <= 1)
  {
    ROS_WARN("Nothing to publish, octree is empty");
    return;
  }

  bool publish_marker_array = (latched_topics_ || pub_marker_.getNumSubscribers() > 0);
  bool publish_free_marker_array = (latched_topics_ || pub_fmarker_.getNumSubscribers() > 0);
  bool publish_point_cloud = (latched_topics_ || pub_point_cloud_.getNumSubscribers() > 0);

  // init markers for free space:
  visualization_msgs::MarkerArray free_nodes_vis;
  // each array stores all cubes of a different size, one for each depth level:
  free_nodes_vis.markers.resize(tree_depth_+1);

  // init markers:
  visualization_msgs::MarkerArray occupied_nodes_vis;
  // each array stores all cubes of a different size, one for each depth level:
  occupied_nodes_vis.markers.resize(tree_depth_+1);

  // init pointcloud:
  pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;

  // now, traverse all leafs in the tree:
  for (octomap::LabelOccupancyOcTree::iterator it = octree_->begin(max_tree_depth_), end = octree_->end();
       it != end; ++it)
  {
    double z = it.getZ();
    // filter with z value
    if (z <= occupancy_min_z_ || z >= occupancy_max_z_)
    {
      continue;
    }

    // create markers and point cloud to visualize
    if (octree_->isNodeOccupied(*it))
    {
      double size = it.getSize();
      double x = it.getX();
      double y = it.getY();

      // Ignore speckles in the map:
      if (filter_speckles_ && (it.getDepth() == tree_depth_ + 1) && isSpeckleNode(it.getKey()))
      {
        ROS_DEBUG("Ignoring single speckle at (%f, %f, %f)", x, y, z);
        continue;
      } // else: current octree node is no speckle, send it out

      // convert occupancy to color
      std_msgs::ColorRGBA color;
      if (publish_marker_array || publish_point_cloud)
      {
        std::valarray<float> occupancy = (*it).getOccupancy();
        int label;
        float max_probability = 0.0;
        for (int i=0; i < occupancy.size(); i++)
        {
          if (occupancy[i] > max_probability)
          {
            max_probability = occupancy[i];
            label = i;
          }
        }
        cv::Vec3d rgb = cv_bridge::rgb_colors::getRGBColor(label);
        color.r = rgb[0];
        color.g = rgb[1];
        color.b = rgb[2];
        color.a = max_probability;
      }

      // create marker
      if (publish_marker_array)
      {
        unsigned idx = it.getDepth();
        assert(idx < occupied_nodes_vis.markers.size());

        geometry_msgs::Point cube_center;
        cube_center.x = x;
        cube_center.y = y;
        cube_center.z = z;

        occupied_nodes_vis.markers[idx].points.push_back(cube_center);
        occupied_nodes_vis.markers[idx].colors.push_back(color);
      }

      // insert into pointcloud:
      if (publish_point_cloud)
      {
        pcl::PointXYZRGB point = pcl::PointXYZRGB();
        point.x = x;
        point.y = y;
        point.z = z;
        point.r = color.r;
        point.g = color.g;
        point.b = color.b;
        pcl_cloud.push_back(point);
      }
    }
    else
    {
      // create marker for free space:
      if (publish_free_marker_array)
      {
        double x = it.getX();
        double y = it.getY();

        unsigned idx = it.getDepth();
        assert(idx < free_nodes_vis.markers.size());

        geometry_msgs::Point cube_center;
        cube_center.x = x;
        cube_center.y = y;
        cube_center.z = z;

        free_nodes_vis.markers[idx].points.push_back(cube_center);
      }
    }
  }

  // finish MarkerArray:
  if (publish_marker_array)
  {
    for (unsigned i= 0; i < occupied_nodes_vis.markers.size(); ++i)
    {
      double size = octree_->getNodeSize(i);

      occupied_nodes_vis.markers[i].header.frame_id = world_frame_id_;
      occupied_nodes_vis.markers[i].header.stamp = rostime;
      occupied_nodes_vis.markers[i].ns = "map";
      occupied_nodes_vis.markers[i].id = i;
      occupied_nodes_vis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
      occupied_nodes_vis.markers[i].scale.x = size;
      occupied_nodes_vis.markers[i].scale.y = size;
      occupied_nodes_vis.markers[i].scale.z = size;

      if (occupied_nodes_vis.markers[i].points.size() > 0)
      {
        occupied_nodes_vis.markers[i].action = visualization_msgs::Marker::ADD;
      }
      else
      {
        occupied_nodes_vis.markers[i].action = visualization_msgs::Marker::DELETE;
      }
    }

    pub_marker_.publish(occupied_nodes_vis);
  }


  // finish FreeMarkerArray
  if (publish_free_marker_array)
  {
    for (unsigned i= 0; i < free_nodes_vis.markers.size(); ++i)
    {
      double size = octree_->getNodeSize(i);

      free_nodes_vis.markers[i].header.frame_id = world_frame_id_;
      free_nodes_vis.markers[i].header.stamp = rostime;
      free_nodes_vis.markers[i].ns = "map";
      free_nodes_vis.markers[i].id = i;
      free_nodes_vis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
      free_nodes_vis.markers[i].scale.x = size;
      free_nodes_vis.markers[i].scale.y = size;
      free_nodes_vis.markers[i].scale.z = size;
      free_nodes_vis.markers[i].color = color_free_;

      if (free_nodes_vis.markers[i].points.size() > 0)
      {
        free_nodes_vis.markers[i].action = visualization_msgs::Marker::ADD;
      }
      else
      {
        free_nodes_vis.markers[i].action = visualization_msgs::Marker::DELETE;
      }
    }

    pub_fmarker_.publish(free_nodes_vis);
  }

  // finish pointcloud:
  if (publish_point_cloud)
  {
    sensor_msgs::PointCloud2 cloud;
    pcl::toROSMsg(pcl_cloud, cloud);
    cloud.header.frame_id = world_frame_id_;
    cloud.header.stamp = rostime;
    cloud.is_dense = false;
    pub_point_cloud_.publish(cloud);
  }

  double total_elapsed = (ros::WallTime::now() - start_time).toSec();
  ROS_DEBUG("Map publishing in LabelOctomapServer took %f sec", total_elapsed);
}

bool LabelOctomapServer::clearBBXSrv(
    octomap_msgs::BoundingBoxQuery::Request& req, octomap_msgs::BoundingBoxQuery::Response& res)
{
  octomap::point3d min = octomap::pointMsgToOctomap(req.min);
  octomap::point3d max = octomap::pointMsgToOctomap(req.max);

  double threshold_min = octree_->getClampingThresMin();
  std::valarray<float> log_odds(octomap::logodds(threshold_min), n_label_);
  for (octomap::LabelOccupancyOcTree::leaf_bbx_iterator it = octree_->begin_leafs_bbx(min, max),
      end = octree_->end_leafs_bbx(); it != end; ++it)
  {
    it->setLogOdds(log_odds);
    // octree_->updateNode(it.getKey(), -6.0f);
  }
  // TODO: eval which is faster (setLogOdds+updateInner or updateNode)
  octree_->updateInnerOccupancy();

  publishAll(ros::Time::now());

  return true;
}

bool LabelOctomapServer::resetSrv(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  visualization_msgs::MarkerArray occupied_nodes_vis;
  occupied_nodes_vis.markers.resize(tree_depth_ +1);
  ros::Time rostime = ros::Time::now();
  octree_->clear();

  ROS_INFO("Cleared octomap");
  publishAll(rostime);

  for (unsigned i= 0; i < occupied_nodes_vis.markers.size(); ++i)
  {
    occupied_nodes_vis.markers[i].header.frame_id = world_frame_id_;
    occupied_nodes_vis.markers[i].header.stamp = rostime;
    occupied_nodes_vis.markers[i].ns = "map";
    occupied_nodes_vis.markers[i].id = i;
    occupied_nodes_vis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
    occupied_nodes_vis.markers[i].action = visualization_msgs::Marker::DELETE;
  }

  pub_marker_.publish(occupied_nodes_vis);

  visualization_msgs::MarkerArray free_nodes_vis;
  free_nodes_vis.markers.resize(tree_depth_ +1);

  for (unsigned i= 0; i < free_nodes_vis.markers.size(); ++i)
  {
    free_nodes_vis.markers[i].header.frame_id = world_frame_id_;
    free_nodes_vis.markers[i].header.stamp = rostime;
    free_nodes_vis.markers[i].ns = "map";
    free_nodes_vis.markers[i].id = i;
    free_nodes_vis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
    free_nodes_vis.markers[i].action = visualization_msgs::Marker::DELETE;
  }
  pub_fmarker_.publish(free_nodes_vis);

  return true;
}

bool LabelOctomapServer::isSpeckleNode(const octomap::OcTreeKey& nKey) const
{
  octomap::OcTreeKey key;
  bool neighborFound = false;
  for (key[2] = nKey[2] - 1; !neighborFound && key[2] <= nKey[2] + 1; ++key[2])
  {
    for (key[1] = nKey[1] - 1; !neighborFound && key[1] <= nKey[1] + 1; ++key[1])
    {
      for (key[0] = nKey[0] - 1; !neighborFound && key[0] <= nKey[0] + 1; ++key[0])
      {
        if (key != nKey)
        {
          octomap::LabelOccupancyOcTree::NodeType* node = octree_->search(key);
          if (node && octree_->isNodeOccupied(node))
          {
            // we have a neighbor => break!
            neighborFound = true;
          }
        }
      }
    }
  }

  return neighborFound;
}

void LabelOctomapServer::reconfigureCallback(octomap_server::OctomapServerConfig& config, uint32_t level)
{
  if (max_tree_depth_ != unsigned(config.max_depth))
  {
    max_tree_depth_ = unsigned(config.max_depth);
  }
  else
  {
    occupancy_min_z_             = config.occupancy_min_z;
    occupancy_max_z_             = config.occupancy_max_z;
    filter_speckles_            = config.filter_speckles;
    compress_map_               = config.compress_map;

    // Parameters with a namespace require an special treatment at the beginning, as dynamic reconfigure
    // will overwrite them because the server is not able to match parameters' names.
    if (init_config_)
    {
      // If parameters do not have the default value, dynamic reconfigure server should be updated.
      if (!label_octomap_server::is_equal(max_range_, -1.0))
      {
        config.sensor_model_max_range = max_range_;
      }
      if (!label_octomap_server::is_equal(octree_->getProbHit(), 0.7))
      {
        config.sensor_model_hit = octree_->getProbHit();
      }
      if (!label_octomap_server::is_equal(octree_->getProbMiss(), 0.4))
      {
        config.sensor_model_miss = octree_->getProbMiss();
      }
      if (!label_octomap_server::is_equal(octree_->getClampingThresMin(), 0.12))
      {
        config.sensor_model_min = octree_->getClampingThresMin();
      }
      if (!label_octomap_server::is_equal(octree_->getClampingThresMax(), 0.97))
      {
        config.sensor_model_max = octree_->getClampingThresMax();
      }
      init_config_ = false;

      boost::recursive_mutex::scoped_lock reconf_lock(config_mutex_);
      reconfigure_server_.updateConfig(config);
    }
    else
    {
      max_range_ = config.sensor_model_max_range;
      octree_->setClampingThresMin(config.sensor_model_min);
      octree_->setClampingThresMax(config.sensor_model_max);

      // Checking values that might create unexpected behaviors.
      if (label_octomap_server::is_equal(config.sensor_model_hit, 1.0))
      {
        config.sensor_model_hit -= 1.0e-6;
      }
      octree_->setProbHit(config.sensor_model_hit);
      if (label_octomap_server::is_equal(config.sensor_model_miss, 0.0))
      {
        config.sensor_model_miss += 1.0e-6;
      }
      octree_->setProbMiss(config.sensor_model_miss);
    }
  }
  publishAll();
}

}  // namespace octomap_server
