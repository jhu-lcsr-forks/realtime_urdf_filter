/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage nor the names of its
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

/* Author: Ioan Sucan, Suat Gedikli */
/* Author: Jonathan Bohren */

#include <realtime_urdf_filter/moveit_octomap_updater.h>
#include <moveit/occupancy_map_monitor/occupancy_map_monitor.h>
#include <geometric_shapes/shape_operations.h>
#include <sensor_msgs/image_encodings.h>
#include <XmlRpcException.h>
#include <stdint.h>

namespace occupancy_map_monitor
{

RealtimeURDFFilterOctomapUpdater::RealtimeURDFFilterOctomapUpdater() :
  OccupancyMapUpdater("RealtimeURDFFilterUpdater"),
  nh_("~"),
  argc_(1),
  argv_(new char*[argc_]),
  input_depth_transport_(nh_),
  model_depth_transport_(nh_),
  filtered_depth_transport_(nh_),
  filtered_label_transport_(nh_),
  image_topic_("depth"),
  queue_size_(5),
  near_clipping_plane_distance_(0.3),
  far_clipping_plane_distance_(5.0),
  shadow_threshold_(0.04),
  padding_scale_(0.0),
  padding_offset_(0.02),
  skip_vertical_pixels_(4),
  skip_horizontal_pixels_(6),
  image_callback_count_(0),
  average_callback_dt_(0.0),
  good_tf_(5), // start optimistically, so we do not output warnings right from the beginning
  failed_tf_(0),
  K0_(0.0), K2_(0.0), K4_(0.0), K5_(0.0)
{
  argv_[1] = "RealtimeURDFFilterUpdater";
  argv_[0] = "";

  filter_.reset(new realtime_urdf_filter::RealtimeURDFFilter(nh_, argc_, argv_));
}

RealtimeURDFFilterOctomapUpdater::~RealtimeURDFFilterOctomapUpdater()
{
  filter_->stop();

#if 0 // no need to delete these since they were initialized from static const char*
  if(argv_) {
    for(unsigned int i=0; i<argc_; i++) {
      delete argv_[i];
    }
    delete[] argv_;
    argc_ = 0;
  }
#endif
}

bool RealtimeURDFFilterOctomapUpdater::setParams(XmlRpc::XmlRpcValue &params)
{
  try
  {
    filter_->setParams(params);
  }
  catch (XmlRpc::XmlRpcException &ex)
  {
    ROS_ERROR("XmlRpc Exception: %s", ex.getMessage().c_str());
    return false;
  }

  return true;
}

bool RealtimeURDFFilterOctomapUpdater::initialize()
{
  tf_ = monitor_->getTFClient();
  free_space_updater_.reset(new LazyFreeSpaceUpdater(tree_));

  return true;
}

void RealtimeURDFFilterOctomapUpdater::start()
{
  filter_->advertise();
  filter_->subscribe(boost::bind(&RealtimeURDFFilterOctomapUpdater::depthImageCallback, this, _1, _2));
}

void RealtimeURDFFilterOctomapUpdater::stop()
{
}

mesh_filter::MeshHandle RealtimeURDFFilterOctomapUpdater::excludeShape(const shapes::ShapeConstPtr &shape)
{
  mesh_filter::MeshHandle h = 0;

  return h;
}

void RealtimeURDFFilterOctomapUpdater::forgetShape(mesh_filter::MeshHandle handle)
{
}


void RealtimeURDFFilterOctomapUpdater::depthImageCallback(const sensor_msgs::ImageConstPtr& depth_msg, const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  ROS_DEBUG("Received a new depth image message (frame = '%s', encoding='%s')", depth_msg->header.frame_id.c_str(), depth_msg->encoding.c_str());

  // hold wall time for performance monitoring
  ros::WallTime start = ros::WallTime::now();

#if 1 // measure the frequency at which we receive updates
  if (image_callback_count_ < 1000)
  {
    if (image_callback_count_ > 0)
    {
      const double dt_start = (start - last_depth_callback_start_).toSec();
      if (image_callback_count_ < 2)
        average_callback_dt_ = dt_start;
      else
        average_callback_dt_ = ((image_callback_count_ - 1) * average_callback_dt_ + dt_start) / (double)image_callback_count_;
    }
  }
  else
    // every 1000 updates we reset the counter almost to the beginning (use 2 so we don't have so much of a ripple in the measured average)
    image_callback_count_ = 2;
  last_depth_callback_start_ = start;
  ++image_callback_count_;
#endif

  // use the depth frame if there's no frame to transform into
  if (monitor_->getMapFrame().empty())
    monitor_->setMapFrame(depth_msg->header.frame_id);

  // compute the transform from the sensor to the map frame
  tf::StampedTransform map_H_sensor;
  if (monitor_->getMapFrame() == depth_msg->header.frame_id) {
    map_H_sensor.setIdentity();
  } else {
    // get transform for cloud into map frame
    if (tf_)
    {
      // wait at most 50ms
      static const double TEST_DT = 0.005;
      const int nt = (int)(0.5 + average_callback_dt_ / TEST_DT) * std::max(1, ((int)queue_size_ / 2));
      bool found = false;
      std::string err;
      for (int t = 0 ; t < nt ; ++t) {
        try
        {
          tf_->lookupTransform(monitor_->getMapFrame(), depth_msg->header.frame_id, depth_msg->header.stamp, map_H_sensor);
          found = true;
          break;
        }
        catch (tf::TransformException &ex)
        {
          static const ros::Duration d(TEST_DT);
          err = ex.what();
          d.sleep();
        }
      }
      static const unsigned int MAX_TF_COUNTER = 1000; // so we avoid int overflow
      if (found)
      {
        good_tf_++;
        if (good_tf_ > MAX_TF_COUNTER)
        {
          const unsigned int div = MAX_TF_COUNTER/10;
          good_tf_ /= div;
          failed_tf_ /= div;
        }
      }
      else
      {
        failed_tf_++;
        if (failed_tf_ > good_tf_)
          ROS_WARN_THROTTLE(1, "More than half of the image messages discared due to TF being unavailable (%u%%). Transform error of sensor data: %s; quitting callback. Map frame: %s, Depth frame: %s",
                            (100 * failed_tf_) / (good_tf_ + failed_tf_), err.c_str(), monitor_->getMapFrame().c_str(), depth_msg->header.frame_id.c_str());
        else
          ROS_DEBUG_THROTTLE(1, "Transform error of sensor data: %s; quitting callback", err.c_str());
        if (failed_tf_ > MAX_TF_COUNTER)
        {
          const unsigned int div = MAX_TF_COUNTER/10;
          good_tf_ /= div;
          failed_tf_ /= div;
        }
        return;
      }
    }
    else
      return;
  }

  if (!updateTransformCache(depth_msg->header.frame_id, depth_msg->header.stamp))
  {
    ROS_ERROR_THROTTLE(1, "Transform cache was not updated. Self-filtering may fail.");
    return;
  }

# if 0 // check endianness
  if (depth_msg->is_bigendian && !HOST_IS_BIG_ENDIAN)
    ROS_ERROR_THROTTLE(1, "endian problem: received image data does not match host");
#endif

  const int w = depth_msg->width;
  const int h = depth_msg->height;

#if 0 // call the mesh filter (old)
  mesh_filter::StereoCameraModel::Parameters& params = mesh_filter_->parameters();
  params.setCameraParameters (info_msg->K[0], info_msg->K[4], info_msg->K[2], info_msg->K[5]);
  params.setImageSize(w, h);
#endif

  const bool is_u_short = depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1;
#if 0 // call the mesh filter (old)
  if (is_u_short)
    mesh_filter_->filter(&depth_msg->data[0], GL_UNSIGNED_SHORT);
  else
  {
    if (depth_msg->encoding != sensor_msgs::image_encodings::TYPE_32FC1)
    {
      ROS_ERROR_THROTTLE(1, "Unexpected encoding type: '%s'. Ignoring input.", depth_msg->encoding.c_str());
      return;
    }
    mesh_filter_->filter(&depth_msg->data[0], GL_FLOAT);
  }

  // the mesh filter runs in background; compute extra things in the meantime
#endif

  // filter the depth image
  filter_->filter_callback(depth_msg, info_msg);

  // Use correct principal point from calibration
  const double px = info_msg->K[2];
  const double py = info_msg->K[5];

  // if the camera parameters have changed at all, recompute the cache we had
  if (w >= x_cache_.size() || h >= y_cache_.size() || K2_ != px || K5_ != py || K0_ != info_msg->K[0] || K4_ != info_msg->K[4])
  {
    K2_ = px;
    K5_ = py;
    K0_ = info_msg->K[0];
    K4_ = info_msg->K[4];

    inv_fx_ = 1.0 / K0_;
    inv_fy_ = 1.0 / K4_;

    // if there are any NaNs, discard data
    if (!(px == px && py == py && inv_fx_ == inv_fx_ && inv_fy_ == inv_fy_))
      return;

    // Pre-compute some constants
    if (x_cache_.size() < w)
      x_cache_.resize(w);
    if (y_cache_.size() < h)
      y_cache_.resize(h);

    for (int x = 0; x < w; ++x)
      x_cache_[x] = (x - px) * inv_fx_;

    for (int y = 0; y < h; ++y)
      y_cache_[y] = (y - py) * inv_fy_;
  }

  const octomap::point3d sensor_origin(map_H_sensor.getOrigin().getX(), map_H_sensor.getOrigin().getY(), map_H_sensor.getOrigin().getZ());

  octomap::KeySet *occupied_cells_ptr = new octomap::KeySet();
  octomap::KeySet *model_cells_ptr = new octomap::KeySet();
  octomap::KeySet &occupied_cells = *occupied_cells_ptr;
  octomap::KeySet &model_cells = *model_cells_ptr;

  // allocate memory if needed
  std::size_t img_size = h * w;
  if (filtered_labels_.size() < img_size)
    filtered_labels_.resize(img_size);

#if 0
  // get filtered labels

  // get the labels of the filtered data
  const unsigned int* labels_row = &filtered_labels_ [0];
  mesh_filter_->getFilteredLabels(&filtered_labels_ [0]);
#else
  // get filtered labels
  filter_->getLables(filtered_labels_);

  // get the labels of the filtered data
  const unsigned int* labels_row = &filtered_labels_ [0];
#endif

#if 0 // publish debug information if needed
  if (debug_info_)
  {
    sensor_msgs::Image debug_msg;
    debug_msg.header = depth_msg->header;
    debug_msg.height = depth_msg->height;
    debug_msg.width = depth_msg->width;
    debug_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    debug_msg.is_bigendian = depth_msg->is_bigendian;
    debug_msg.step = depth_msg->step;
    debug_msg.data.resize(img_size * sizeof(float));
    mesh_filter_->getModelDepth(reinterpret_cast<float*>(&debug_msg.data[0]));
    pub_model_depth_image_.publish(debug_msg, *info_msg);

    sensor_msgs::Image filtered_depth_msg;
    filtered_depth_msg.header = depth_msg->header;
    filtered_depth_msg.height = depth_msg->height;
    filtered_depth_msg.width = depth_msg->width;
    filtered_depth_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    filtered_depth_msg.is_bigendian = depth_msg->is_bigendian;
    filtered_depth_msg.step = depth_msg->step;
    filtered_depth_msg.data.resize(img_size * sizeof(float));

    mesh_filter_->getFilteredDepth(reinterpret_cast<float*>(&filtered_depth_msg.data[0]));
    pub_filtered_depth_image_.publish(filtered_depth_msg, *info_msg);

    sensor_msgs::Image label_msg;
    label_msg.header = depth_msg->header;
    label_msg.height = depth_msg->height;
    label_msg.width = depth_msg->width;
    label_msg.encoding = sensor_msgs::image_encodings::RGBA8;
    label_msg.is_bigendian = depth_msg->is_bigendian;
    label_msg.step = w * sizeof(unsigned int);
    label_msg.data.resize(img_size * sizeof(unsigned int));
    mesh_filter_->getFilteredLabels(reinterpret_cast<unsigned int*>(&label_msg.data[0]));

    pub_filtered_label_image_.publish(label_msg, *info_msg);
  }
#endif

#if 0 // publish filtered cloud ( cloud without robot )
  if(!filtered_cloud_topic_.empty())
  {
    static std::vector<float> filtered_data;
    sensor_msgs::Image filtered_msg;
    filtered_msg.header = depth_msg->header;
    filtered_msg.height = depth_msg->height;
    filtered_msg.width = depth_msg->width;
    filtered_msg.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
    filtered_msg.is_bigendian = depth_msg->is_bigendian;
    filtered_msg.step = depth_msg->step;
    filtered_msg.data.resize(img_size * sizeof(unsigned short));
    if(filtered_data.size() < img_size)
      filtered_data.resize(img_size);
    mesh_filter_->getFilteredDepth(reinterpret_cast<float*>(&filtered_data[0]));
    unsigned short* tmp_ptr = (unsigned short*) &filtered_msg.data[0];
    for(std::size_t i=0; i < img_size; ++i)
    {
      tmp_ptr[i] = (unsigned short) (filtered_data[i] * 1000 + 0.5);
    }
    pub_filtered_depth_image_.publish(filtered_msg, *info_msg);
  }
#endif

  // figure out occupied cells and model cells
  tree_->lockRead();

  try
  {
    const int h_bound = h - skip_vertical_pixels_;
    const int w_bound = w - skip_horizontal_pixels_;
    const double scale = (is_u_short) ? (1e-3) : (1.0);

    if (is_u_short)
    {
      const uint16_t *input_row = reinterpret_cast<const uint16_t*>(&depth_msg->data[0]);

      for (int y = skip_vertical_pixels_ ; y < h_bound ; ++y, labels_row += w, input_row += w)
        for (int x = skip_horizontal_pixels_ ; x < w_bound ; ++x)
        {
          // not filtered
          if (labels_row [x] == mesh_filter::MeshFilterBase::Background)
          {
            float zz = (float)input_row[x] * scale; // scale from mm to m
            float yy = y_cache_[y] * zz;
            float xx = x_cache_[x] * zz;
            /* transform to map frame */
            tf::Vector3 point_tf = map_H_sensor * tf::Vector3(xx, yy, zz);
            occupied_cells.insert(tree_->coordToKey(point_tf.getX(), point_tf.getY(), point_tf.getZ()));
          }
          // on far plane or a model point -> remove
          else if (labels_row [x] >= mesh_filter::MeshFilterBase::FarClip)
          {
            float zz = input_row[x] * scale;
            float yy = y_cache_[y] * zz;
            float xx = x_cache_[x] * zz;
            /* transform to map frame */
            tf::Vector3 point_tf = map_H_sensor * tf::Vector3(xx, yy, zz);
            // add to the list of model cells
            model_cells.insert(tree_->coordToKey(point_tf.getX(), point_tf.getY(), point_tf.getZ()));
          }
        }
    }
    else
    {
      const float *input_row = reinterpret_cast<const float*>(&depth_msg->data[0]);

      for (int y = skip_vertical_pixels_ ; y < h_bound ; ++y, labels_row += w, input_row += w)
        for (int x = skip_horizontal_pixels_ ; x < w_bound ; ++x)
        {
          if (labels_row [x] == mesh_filter::MeshFilterBase::Background)
          {
            float zz = input_row[x] * scale;
            float yy = y_cache_[y] * zz;
            float xx = x_cache_[x] * zz;
            /* transform to map frame */
            tf::Vector3 point_tf = map_H_sensor * tf::Vector3(xx, yy, zz);
            occupied_cells.insert(tree_->coordToKey(point_tf.getX(), point_tf.getY(), point_tf.getZ()));
          }
          else if (labels_row [x] >= mesh_filter::MeshFilterBase::FarClip)
          {
            float zz = input_row[x] * scale;
            float yy = y_cache_[y] * zz;
            float xx = x_cache_[x] * zz;
            /* transform to map frame */
            tf::Vector3 point_tf = map_H_sensor * tf::Vector3(xx, yy, zz);
            // add to the list of model cells
            model_cells.insert(tree_->coordToKey(point_tf.getX(), point_tf.getY(), point_tf.getZ()));
          }
        }
    }

  }
  catch (...)
  {
    tree_->unlockRead();
    return;
  }
  tree_->unlockRead();

  /* cells that overlap with the model are not occupied */
  for (octomap::KeySet::iterator it = model_cells.begin(), end = model_cells.end(); it != end; ++it)
    occupied_cells.erase(*it);

  // mark occupied cells
  tree_->lockRead();
  tree_->lockWrite();
  try
  {
    /* now mark all occupied cells */
    for (octomap::KeySet::iterator it = occupied_cells.begin(), end = occupied_cells.end(); it != end; ++it)
      tree_->updateNode(*it, true);
  }
  catch (...)
  {
    ROS_ERROR("Internal error while updating octree");
  }
  tree_->unlockWrite();
  tree_->unlockRead();
  tree_->triggerUpdateCallback();

  // at this point we still have not freed the space
  free_space_updater_->pushLazyUpdate(occupied_cells_ptr, model_cells_ptr, sensor_origin);

  ROS_DEBUG("Processed depth image in %lf ms", (ros::WallTime::now() - start).toSec() * 1000.0);
}

}
