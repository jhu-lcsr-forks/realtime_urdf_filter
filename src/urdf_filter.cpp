/*
 * Copyright (c) 2011, Nico Blodow <blodow@cs.tum.edu>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Intelligent Autonomous Systems Group/
 *       Technische Universitaet Muenchen nor the names of its contributors
 *       may be used to endorse or promote products derived from this software
 *       without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "realtime_urdf_filter/urdf_filter.h"

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <algorithm>
#include <image_transport/camera_common.h>


//#define USE_OWN_CALIBRATION

using namespace realtime_urdf_filter;

// constructor. sets up ros and reads in parameters
RealtimeURDFFilter::RealtimeURDFFilter (ros::NodeHandle &nh, int argc, char **argv)
  : nh_(nh)
  , image_transport_(nh_)
  , fbo_initialized_(false)
  , depth_image_pbo_ (GL_INVALID_VALUE)
  , depth_texture_(GL_INVALID_VALUE)
  , depth_input_image_topic_("input_depth")
  , depth_input_camera_info_topic_("camera_info")
  , depth_filtered_topic_("output_depth")
  , depth_mask_topic_("output_mask")
  , width_(0)
  , height_(0)
  , camera_tx_(0)
  , camera_ty_(0)
  , far_plane_ (8)
  , near_plane_ (0.1)
  , argc_ (argc), argv_(argv)
{

}

void RealtimeURDFFilter::start ()
{
  XmlRpc::XmlRpcValue params;
  nh_.getParam("/", params);
  this->setParams(params);

  std::string orig_camera_info_topic = image_transport::getCameraInfoTopic(depth_input_image_topic_);

  std::map<std::string, std::string> it_remappings;
  it_remappings[orig_camera_info_topic] = depth_input_camera_info_topic_;
  ros::NodeHandle it_nh(nh_, nh_.getNamespace(), it_remappings);
  image_transport_ = image_transport::ImageTransport(it_nh);

  this->advertise();
  this->subscribe();
}

void RealtimeURDFFilter::stop ()
{
  depth_sub_.shutdown();
}

static std::string getXmlParam(XmlRpc::XmlRpcValue &params, std::string name)
{
  XmlRpc::XmlRpcValue v;

  if(params.hasMember(name)) {
    v = params[name];
    ROS_ASSERT (v.getType() == XmlRpc::XmlRpcValue::TypeString && "paramter!");
    ROS_INFO_STREAM ("using "<<name<<" "<< ((std::string)v));
    return (std::string)v;
  }

  return "";
}

void RealtimeURDFFilter::setParams (XmlRpc::XmlRpcValue &params)
{
  XmlRpc::XmlRpcValue v;

  // get fixed frame name
  ROS_ASSERT (params.hasMember("fixed_frame"));
  v = params["fixed_frame"];
  ROS_ASSERT (v.getType() == XmlRpc::XmlRpcValue::TypeString && "fixed_frame paramter!");
  fixed_frame_ = (std::string)v;
  ROS_INFO ("using fixed frame %s", fixed_frame_.c_str ());

  if(params.hasMember("depth_input_image_topic")) {
    v = params["depth_input_image_topic"];
    ROS_ASSERT (v.getType() == XmlRpc::XmlRpcValue::TypeString && "depth_input_image_topic paramter!");
    depth_input_image_topic_ = (std::string)v;
    ROS_INFO ("using image topic %s", depth_input_image_topic_.c_str ());
  }

  depth_input_camera_info_topic_ = getXmlParam(params, "depth_input_camera_info_topic");
  depth_filtered_topic_ = getXmlParam(params, "depth_filtered_topic");
  depth_mask_topic_ = getXmlParam(params, "depth_mask_topic");

  // get camera frame name
  // we do not read this from ROS message, for being able to run this within openni (self filtered tracker..)
  ROS_ASSERT (params.hasMember("camera_frame"));
  v = params["camera_frame"];
  ROS_ASSERT (v.getType() == XmlRpc::XmlRpcValue::TypeString && "need a camera_frame paramter!");
  cam_frame_ = (std::string)v;
  ROS_INFO ("using camera frame %s", cam_frame_.c_str ());

  // read additional camera offset (TODO: make optional)
  ROS_ASSERT (params.hasMember("camera_offset"));
  v = params["camera_offset"];
  ROS_ASSERT (v.getType() == XmlRpc::XmlRpcValue::TypeStruct && "need a camera_offset paramter!");
  ROS_ASSERT (v.hasMember ("translation") && v.hasMember ("rotation") && "camera offset needs a translation and rotation parameter!");

  // translation
  XmlRpc::XmlRpcValue vec = v["translation"];
  ROS_ASSERT (vec.getType() == XmlRpc::XmlRpcValue::TypeArray && vec.size() == 3 && "camera_offset.translation parameter must be a 3-value array!");
  ROS_INFO ("using camera translational offset: %f %f %f",
      (double)(vec[0]),
      (double)(vec[1]),
      (double)(vec[2])
      );
  camera_offset_t_ = tf::Vector3((double)vec[0], (double)vec[1], (double)vec[2]);

  // rotation
  vec = v["rotation"];
  ROS_ASSERT (vec.getType() == XmlRpc::XmlRpcValue::TypeArray && vec.size() == 4 && "camera_offset.rotation parameter must be a 4-value array [x y z w]!");
  ROS_INFO ("using camera rotational offset: %f %f %f %f", (double)vec[0], (double)vec[1], (double)vec[2], (double)vec[3]);
  camera_offset_q_ = tf::Quaternion((double)vec[0], (double)vec[1], (double)vec[2], (double)vec[3]);

  // depth distance threshold (how far from the model are points still deleted?)
  ROS_ASSERT (params.hasMember("depth_distance_threshold"));
  v = params["depth_distance_threshold"];
  ROS_ASSERT (v.getType() == XmlRpc::XmlRpcValue::TypeDouble && "need a depth_distance_threshold paramter!");
  depth_distance_threshold_ = (double)v;
  ROS_INFO ("using depth distance threshold %f", depth_distance_threshold_);

  // depth distance threshold (how far from the model are points still deleted?)
  ROS_ASSERT (params.hasMember("show_gui"));
  v = params["show_gui"];
  ROS_ASSERT (v.getType() == XmlRpc::XmlRpcValue::TypeBoolean && "need a show_gui paramter!");
  show_gui_ = (bool)v;
  ROS_INFO ("showing gui / visualization: %s", (show_gui_?"ON":"OFF"));

  // fitler replace value
  ROS_ASSERT (params.hasMember("filter_replace_value"));
  v = params["filter_replace_value"];
  ROS_ASSERT (v.getType() == XmlRpc::XmlRpcValue::TypeDouble && "need a filter_replace_value paramter!");
  filter_replace_value_ = (double)v;
  ROS_INFO ("using filter replace value %f", filter_replace_value_);

  // read models
  ROS_ASSERT (params.hasMember("models"));
  v = params["models"];
  ROS_ASSERT (v.getType() == XmlRpc::XmlRpcValue::TypeArray && "models param must be an array!");
  models_ = v;

  // determine if visual or collision geometry should be used
  ROS_ASSERT (params.hasMember("geometry_type"));
  v = params["geometry_type"];
  ROS_ASSERT (v.getType() == XmlRpc::XmlRpcValue::TypeString && "need a geometry_type paramter!");
  geometry_type_ = ((std::string)v == "collision" ? URDFRenderer::COLLISION : URDFRenderer::VISUAL);

  ROS_ASSERT (params.hasMember("inflation"));
  v = params["inflation"];
  ROS_ASSERT (v.getType() == XmlRpc::XmlRpcValue::TypeDouble && "need an inflation paramter!");
  inflation_ = (double)v;
}

void RealtimeURDFFilter::subscribe()
{
  // setup subscribers
  depth_sub_ = image_transport_.subscribeCamera(
      depth_input_image_topic_, 10,
      &RealtimeURDFFilter::filter_callback, this);
}

void RealtimeURDFFilter::subscribe( CallbackType callback)
{
  // setup subscribers
  this->subscribe(depth_input_image_topic_, 10, callback);
}

void RealtimeURDFFilter::subscribe(
    const std::string image_topic,
    const size_t queue,
    CallbackType callback)
{
  // setup subscribers
  depth_sub_ = image_transport_.subscribeCamera(
      image_topic, queue, callback);
}

void RealtimeURDFFilter::advertise ()
{
  // setup publishers
  mask_pub_ = image_transport_.advertiseCamera(depth_mask_topic_, 10);
  depth_pub_ = image_transport_.advertise(depth_filtered_topic_, 10);
  //depth_pub_raw_ = image_transport_.advertiseCamera("output_depth_raw", 10);
}

RealtimeURDFFilter::~RealtimeURDFFilter ()
{
  delete masked_depth_;
  delete mask_;
}

// loads URDF models
void RealtimeURDFFilter::loadModels ()
{
  for (int i = 0; i < models_.size(); ++i)
  {
    XmlRpc::XmlRpcValue elem = models_[i];
    ROS_ASSERT (elem.getType()  == XmlRpc::XmlRpcValue::TypeStruct);

    std::string description_param = elem["model"];
    std::string tf_prefix = elem["tf_prefix"];

    // read URDF model
    std::string content;

    if (!nh_.getParam(description_param, content))
    {
      std::string loc;
      if (nh_.searchParam(description_param, loc))
      {
        nh_.getParam(loc, content);
      }
      else
      {
        ROS_ERROR ("Parameter [%s] does not exist, and was not found by searchParam()",
                   description_param.c_str());
        continue;
      }
    }

    if (content.empty())
    {
      ROS_ERROR ("URDF is empty");
      continue;
    }

    // finally, set the model description so we can later parse it.
    ROS_INFO ("Loading URDF model: %s", description_param.c_str ());
    renderers_.push_back (
        new URDFRenderer (
            content, tf_prefix, cam_frame_, fixed_frame_, tf_,
            geometry_type_,
            inflation_));
  }
}

// helper function to get current time
double RealtimeURDFFilter::getTime ()
{
  timeval current_time;
  gettimeofday (&current_time, NULL);
  return (current_time.tv_sec + 1e-6 * current_time.tv_usec);
}

void RealtimeURDFFilter::filter (
    unsigned char* buffer, double* projection_matrix, int width, int height, const ros::Time ros_now)
{
  static std::vector<double> timings;
  double begin = getTime();
  if (width_ != width || height_ != height) {
    if(width_ !=0 || height_!=0) {
      ROS_ERROR ("image size has changed (%ix%i) -> (%ix%i)", width_, height_, width, height);
    }
    width_ = width;
    height_ = height;
    this->initGL();
  }

  // Load models / construct renderers
  if(renderers_.empty()) {
    return;
  }

  if (mask_pub_.getNumSubscribers() > 0) {
    need_mask_ = true;
  } else {
    need_mask_ = false;
  }

  // get depth_image into OpenGL texture buffer
  int size_in_bytes = width_ * height_ * sizeof(float);
  textureBufferFromDepthBuffer(buffer, size_in_bytes);

  // render everything
  this->render(projection_matrix, ros_now);

  // Timing
  static unsigned count = 0;
  static double last = getTime ();

  double now = getTime ();
  timings.push_back ((now - begin) * 1000.0);

  if (++count == 30 || (now - last) > 5) {
    double sum = 0.0;
    double max = 0.0;
    double min = FLT_MAX;
    for (std::vector<double>::iterator it = timings.begin(); it!=timings.end(); ++it)
    {
      min = std::min (min, *it);
      max = std::max (max, *it);
      sum += *it;
    }

    ROS_DEBUG_STREAM("Average framerate: "
      << std::setprecision(3) << double(count)/double(now - last) << " Hz "
      << " (min: "<< min
      << ", max: " << max
      << ", avg: " << sum / timings.size()
      << " ms)");
    count = 0;
    last = now;
    timings.clear();
  }
}

// callback function that gets ROS images and does everything
void RealtimeURDFFilter::filter_callback
     (const sensor_msgs::ImageConstPtr& ros_depth_image,
      const sensor_msgs::CameraInfo::ConstPtr& camera_info)
{
  //ROS_DEBUG_STREAM("Received image with camera info: "<<*camera_info);

  unsigned char *buffer = NULL;

  // convert to OpenCV cv::Mat
  cv_bridge::CvImageConstPtr orig_depth_img;
  try {
    orig_depth_img = cv_bridge::toCvShare( ros_depth_image);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge Exception: %s", e.what());
    return;
  }

  double depth_factor = 1.0;

  if(orig_depth_img->image.type() == CV_16UC1) {
    depth_factor = 1.0 / 1000.0;
  }

  // Convert the depth image into a char buffer
  cv::Mat depth_image_float;
  orig_depth_img->image.convertTo(depth_image_float, CV_32FC1, depth_factor);
  cv::Mat1f depth_image = depth_image_float;
  buffer = bufferFromDepthImage(depth_image);

  // Compute the projection matrix from the camera_info
  tfScalar projection_matrix[16];
  getProjectionMatrix (camera_info, projection_matrix);

  // Filter the image
  this->filter(buffer, projection_matrix, depth_image.cols, depth_image.rows, ros_depth_image->header.stamp);

  // publish processed depth image and image mask
  if (depth_pub_.getNumSubscribers() > 0)
  {
    cv::Mat masked_depth_image (height_, width_, CV_32FC1, masked_depth_);
    cv_bridge::CvImage out_masked_depth;
    //out_masked_depth.header.frame_id = cam_frame_;
    //out_masked_depth.header.stamp = ros_depth_image->header.stamp;
    out_masked_depth.header = ros_depth_image->header;
    out_masked_depth.encoding = "32FC1";
    out_masked_depth.image = masked_depth_image;
    depth_pub_.publish (out_masked_depth.toImageMsg ());
  }

  if (mask_pub_.getNumSubscribers() > 0)
  {
    cv::Mat mask_image (height_, width_, CV_8UC1, mask_);

    cv_bridge::CvImage out_mask;
    out_mask.header.frame_id = cam_frame_;
    out_mask.header.stamp = ros_depth_image->header.stamp;
    out_mask.encoding = "mono8";
    out_mask.image = mask_image;
    mask_pub_.publish (out_mask.toImageMsg (), camera_info);
  }
}

void RealtimeURDFFilter::getLables(
    std::vector<unsigned int> &lables)
{
  lables.assign(labeled_depth_, labeled_depth_+(width_*height_));
}

void RealtimeURDFFilter::textureBufferFromDepthBuffer(unsigned char* buffer, int size_in_bytes)
{
  ROS_DEBUG_NAMED("buffers","Texture buffer from depth buffer...");
  // check if we already have a PBO
  if (depth_image_pbo_ == GL_INVALID_VALUE) {
    ROS_DEBUG("Generating Pixel Buffer Object...");
    glGenBuffers(1, &depth_image_pbo_);
  }
  // upload buffer data to GPU
  glBindBuffer(GL_ARRAY_BUFFER, depth_image_pbo_);
  glBufferData(GL_ARRAY_BUFFER, size_in_bytes, buffer, GL_DYNAMIC_DRAW);
  glBindBuffer(GL_ARRAY_BUFFER, 0);

  // Check if we already have a texture buffer
  if (depth_texture_ == GL_INVALID_VALUE) {
    ROS_DEBUG("Generating Texture Object...");
    glGenTextures(1, &depth_texture_);
  }
  // assign PBO to Texture Buffer
  glBindTexture(GL_TEXTURE_BUFFER, depth_texture_);
  glTexBuffer(GL_TEXTURE_BUFFER, GL_R32F, depth_image_pbo_);
}

unsigned char* RealtimeURDFFilter::bufferFromDepthImage (cv::Mat1f depth_image)
{
  // Host buffer to hold depth pixel data
  static std::vector<unsigned char> buffer;

  // Try to get the pixel data from cv::Mat as one continuous buffer
  if (depth_image.isContinuous()) {
    return depth_image.data;
  } else {
    // Get the size of each row in bytes
    int row_size = depth_image.cols * depth_image.elemSize();

    // Allocate the buffer
    buffer.resize(row_size * depth_image.rows);

    // Copy the image row by row
    for (int i = 0; i < depth_image.rows; i++) {
      memcpy(
          (void*)(&buffer[0] + i * row_size),
          (void*) &depth_image.data[i],
          row_size);
    }
  }

  return &buffer[0];
}

// set up OpenGL stuff
void RealtimeURDFFilter::initGL ()
{
  static bool gl_initialized = false;

  ROS_INFO("Initializing OpenGL subsystem...");

  if (!gl_initialized) {
    // Initialize GLUT
    glutInit (&argc_, argv_);

    //TODO: change this to use an offscreen pbuffer, so no window is necessary,
    //for now, we can just hide it (see below)

    // The debug window shows a 3x2 grid of images
    glutInitWindowSize (960, 480);
    glutInitDisplayMode ( GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH | GLUT_STENCIL);
    glutCreateWindow ("Realtime URDF Filter Debug Window");

    // Hide the GLUT window
    if (!show_gui_) {
      glutHideWindow();
    }

    gl_initialized = true;
  }

  // initialize OpenGL Extension Wrangler library
  GLenum err = glewInit();
  if (GLEW_OK != err) {
    throw std::runtime_error("ERROR: could not initialize GLEW!");
  }

  // Set up FBO
  // FIXME: Replace this with more robust / specialized FBO
  this->initFrameBufferObject();

  // Load URDF models + meshes onto GPU
  this->loadModels();

  // Make sure we loaded something!
  if(renderers_.empty()) {
    throw std::runtime_error("Could not load any models for filtering!");
  } else {
    ROS_INFO_STREAM("Loaded "<<renderers_.size()<<" models for filtering.");
  }

  // Alocate buffer for the masked depth image (float)
  masked_depth_ = new GLfloat[width_ * height_];
  // Allocate buffer for labels
  labeled_depth_ = new GLint[width_ * height_];
  // Alocate buffer for the mask (uchar)
  mask_ = new GLubyte[width_ * height_];
}

// set up FBO
void RealtimeURDFFilter::initFrameBufferObject ()
{

  fbo_ = new FramebufferObject("rgba=5x32t depth=24t stencil=8t");
  fbo_->initialize(width_, height_);

  fbo_initialized_ = true;

  GLenum err = glGetError();
  if(err != GL_NO_ERROR) {
    ROS_ERROR("OpenGL ERROR after FBO initialization: %s", gluErrorString(err));
  }

  GLuint status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
  if(status != GL_FRAMEBUFFER_COMPLETE) {
    ROS_ERROR("OpenGL FrameBuffer ERROR after FBO initialization: %i", status);
  }
}

// compute Projection matrix from CameraInfo message
void RealtimeURDFFilter::getProjectionMatrix (
    const sensor_msgs::CameraInfo::ConstPtr& info, tfScalar* glTf)
{
  tf::Vector3 position;
  tf::Quaternion orientation;

#ifdef USE_OWN_CALIBRATION
  float P[12];
  P[0] = 585.260; P[1] = 0.0;     P[2]  = 317.387; P[3]  = 0.0;
  P[4] = 0.0;     P[5] = 585.028; P[6]  = 239.264; P[7]  = 0.0;
  P[8] = 0.0;     P[9] = 0.0;     P[10] = 1.0;     P[11] = 0.0;

  double fx = P[0];
  double fy = P[5];
  double cx = P[2];
  double cy = P[6];
#else
  double fx = info->P[0];// * 0.5;
  double fy = info->P[5];// * 0.5;
  double cx = info->P[2];// * 0.5;
  double cy = (info->P[6]);// * 0.5 - 48;

  // TODO: check if this does the right thing with respect to registered depth / camera info
  // Add the camera's translation relative to the left camera (from P[3]);
  camera_tx_ = -1 * (info->P[3] / fx);
  camera_ty_ = -1 * (info->P[7] / fy);

#endif

  for (unsigned int i = 0; i < 16; ++i) {
    glTf[i] = 0.0;
  }

  // calculate the projection matrix
  // NOTE: this minus is there to flip the x-axis of the image.
  glTf[0]= -2.0 * fx / width_;
  glTf[5]= 2.0 * fy / height_;

  glTf[8]= 2.0 * (0.5 - cx / width_);
  glTf[9]= 2.0 * (cy / height_ - 0.5);

  glTf[10]= - (far_plane_ + near_plane_) / (far_plane_ - near_plane_);
  glTf[14]= -2.0 * far_plane_ * near_plane_ / (far_plane_ - near_plane_);

  glTf[11]= -1;
}

void RealtimeURDFFilter::render (const double* camera_projection_matrix, const ros::Time now)
{
  static const GLenum buffers[] = {
    GL_COLOR_ATTACHMENT0,
    GL_COLOR_ATTACHMENT1,
    GL_COLOR_ATTACHMENT2,
    GL_COLOR_ATTACHMENT3,
    GL_COLOR_ATTACHMENT4
  };

  GLenum err;

  // Check if the framebuffer object has been initialized
  if (!fbo_initialized_) {
    return;
  }

  // get transformation from camera to "fixed frame"
  tf::StampedTransform camera_transform;
  try {
    tf_.lookupTransform (cam_frame_, fixed_frame_, ros::Time (), camera_transform);
    ROS_DEBUG_STREAM_NAMED("tf","Camera to world translation "<<
        cam_frame_<<" -> "<<fixed_frame_<<
        " ["<<
        " "<<camera_transform.getOrigin().x()<<
        " "<<camera_transform.getOrigin().y()<<
        " "<<camera_transform.getOrigin().z()<<
        "]"
        );
  } catch (tf::TransformException ex) {
    ROS_ERROR("Could not look up camera to world transform: %s",ex.what());
    return;
  }

  err = glGetError();
  if(err != GL_NO_ERROR) {
    ROS_ERROR("OpenGL ERROR at beginning of rendering: %s", gluErrorString(err));
    return;
  }

  glPushAttrib(GL_ALL_ATTRIB_BITS);
  glEnable(GL_NORMALIZE);

  // Render into FBO
  fbo_->beginCapture();

  // Create shader programs
  static ShaderWrapper shader = ShaderWrapper::fromFiles(
      "package://realtime_urdf_filter/include/shaders/urdf_filter.vert",
      "package://realtime_urdf_filter/include/shaders/urdf_filter.frag");

  err = glGetError();
  if(err != GL_NO_ERROR) {
    ROS_ERROR("OpenGL ERROR compiling shaders: %s", gluErrorString(err));
    return;
  }

  // Enable shader for this frame
  shader();

  // Specify the list of color buffers to draw into
  glDrawBuffers(sizeof(buffers) / sizeof(GLenum), buffers);

  //Cclear the buffers
  glClearColor(0.0, 0.0, 0.0, 1.0);
  glClearStencil(0x0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

  glEnable(GL_DEPTH_TEST);
  glDisable(GL_TEXTURE_2D);

  fbo_->disableTextureTarget();

  // Setup camera projection
  glMatrixMode (GL_PROJECTION);
  glLoadIdentity();

  // Load camera projection matrix into OpenGL camera matrix
  glMultMatrixd(camera_projection_matrix);

  // Setup camera position
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  // Kinect has x right, y down, z into image
  gluLookAt (0,0,0, 0,0,1, 0,1,0);

  // Draw background quad behind everything (just before the far plane)
  // Otherwise, the shader only sees kinect points where he rendered stuff
  glBegin(GL_QUADS);
    glVertex3f(-100.0, -100.0, far_plane_*0.99);
    glVertex3f( 100.0, -100.0, far_plane_*0.99);
    glVertex3f( 100.0,  100.0, far_plane_*0.99);
    glVertex3f(-100.0,  100.0, far_plane_*0.99);
  glEnd();

  // Transformation matrix
  double glTf[16];

  // Apply user-defined camera offset transformation (launch file)
  tf::Transform transform (camera_offset_q_, camera_offset_t_);
  transform.inverse().getOpenGLMatrix(glTf);
  glMultMatrixd((GLdouble*)glTf);

  // Apply camera to "fixed frame" transform (world coordinates)
  tf::Vector3 right = tf::Transform(camera_transform.getRotation()) * tf::Vector3 (1,0,0);
  camera_transform.setOrigin(camera_transform.getOrigin() + (right * camera_tx_));
  tf::Vector3 down = tf::Transform(camera_transform.getRotation()) * tf::Vector3 (0,1,0);
  camera_transform.setOrigin(camera_transform.getOrigin() + (down * camera_ty_));


  camera_transform.getOpenGLMatrix(glTf);
  glMultMatrixd((GLdouble*)glTf);

  // Set up stencil buffer etc.
  // The background quad is not in the stencil buffer
  glEnable(GL_STENCIL_TEST);
  glStencilFunc(GL_ALWAYS, 0x1, 0x1);
  glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);

  // make texture with depth image available in shader
  glActiveTexture (GL_TEXTURE0);
  GLuint depth_texture_id = 0;
  shader.SetUniformVal1i (std::string("depth_texture"), depth_texture_id);
  shader.SetUniformVal1i (std::string("width"), int(width_));
  shader.SetUniformVal1i (std::string("height"), int(height_));
  shader.SetUniformVal1f (std::string("z_far"), far_plane_);
  shader.SetUniformVal1f (std::string("z_near"), near_plane_);
  shader.SetUniformVal1f (std::string("max_diff"), float(depth_distance_threshold_));
  shader.SetUniformVal1f (std::string("replace_value"), float(filter_replace_value_));
  glBindTexture (GL_TEXTURE_BUFFER, depth_texture_);

  // render every renderable / urdf model
  std::vector<URDFRenderer*>::const_iterator r;
  for (r = renderers_.begin (); r != renderers_.end (); r++) {
    (*r)->render (now);
  }

  // Disable shader
  glUseProgram((GLuint)NULL);

  fbo_->endCapture();
  glPopAttrib();

  // Use stencil buffer to draw a red / blue mask into color attachment 3
#if 0
  if (need_mask_ || show_gui_) {
    glPushAttrib(GL_ALL_ATTRIB_BITS);

    fbo_->beginCapture();
      glDrawBuffer(GL_COLOR_ATTACHMENT3);

      glEnable(GL_STENCIL_TEST);
      glStencilFunc(GL_EQUAL, 0x1, 0x1);
      glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);

      glDisable(GL_DEPTH_TEST);
      glDisable(GL_TEXTURE_2D);
      fbo_->disableTextureTarget();

      glMatrixMode(GL_PROJECTION);
      glPushMatrix();
        glLoadIdentity();
        gluOrtho2D(0.0, 1.0, 0.0, 1.0);

        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();
          glLoadIdentity();

          glColor3f(1.0, 0.0, 0.0);

          glBegin(GL_QUADS);
            glVertex2f(0.0, 0.0);
            glVertex2f(1.0, 0.0);
            glVertex2f(1.0, 1.0);
            glVertex2f(0.0, 1.0);
          glEnd();

        glPopMatrix();
        glMatrixMode(GL_PROJECTION);
      glPopMatrix();

      glStencilFunc(GL_EQUAL, 0x0, 0x1);
      glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);

      glDisable(GL_DEPTH_TEST);
      glDisable(GL_TEXTURE_2D);
      fbo_->disableTextureTarget();

      glMatrixMode(GL_PROJECTION);
      glPushMatrix();
        glLoadIdentity();
        gluOrtho2D(0.0, 1.0, 0.0, 1.0);

        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();
          glLoadIdentity();

          glColor3f(0.0, 0.0, 1.0);

          glBegin(GL_QUADS);
            glVertex2f(0.0, 0.0);
            glVertex2f(1.0, 0.0);
            glVertex2f(1.0, 1.0);
            glVertex2f(0.0, 1.0);
          glEnd();

        glPopMatrix();
        glMatrixMode(GL_PROJECTION);
      glPopMatrix();
    fbo_->endCapture();

    glPopAttrib();
  }
#endif

  // Render all color buffer attachments into window
  if (show_gui_) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
      glLoadIdentity();
      gluOrtho2D(0.0, 1.0, 0.0, 1.0);

      glMatrixMode(GL_MODELVIEW);
      glPushMatrix();
        glLoadIdentity();

        // draw color buffer 0
        fbo_->bind(0);
        glBegin(GL_QUADS);
          glTexCoord2f(0.0, fbo_->getHeight());
          glVertex2f(0.0, 0.5);
          glTexCoord2f(fbo_->getWidth(), fbo_->getHeight());
          glVertex2f(0.333, 0.5);
          glTexCoord2f(fbo_->getWidth(), 0.0);
          glVertex2f(0.333, 1.0);
          glTexCoord2f(0.0, 0.0);
          glVertex2f(0.0, 1.0);
        glEnd();

        // draw color buffer 1
        fbo_->bind(1);
        glBegin(GL_QUADS);
          glTexCoord2f(0.0, fbo_->getHeight());
          glVertex2f(0.0, 0.0);
          glTexCoord2f(fbo_->getWidth(), fbo_->getHeight());
          glVertex2f(0.333, 0.0);
          glTexCoord2f(fbo_->getWidth(), 0.0);
          glVertex2f(0.333, 0.5);
          glTexCoord2f(0.0, 0.0);
          glVertex2f(0.0, 0.5);
        glEnd();

        // draw color buffer 2
        fbo_->bind(2);
        glBegin(GL_QUADS);
          glTexCoord2f(0.0, fbo_->getHeight());
          glVertex2f(0.333, 0.5);
          glTexCoord2f(fbo_->getWidth(), fbo_->getHeight());
          glVertex2f(0.666, 0.5);
          glTexCoord2f(fbo_->getWidth(), 0.0);
          glVertex2f(0.666, 1.0);
          glTexCoord2f(0.0, 0.0);
          glVertex2f(0.333, 1.0);
        glEnd();

        // draw color buffer 3
        fbo_->bind(3);
        glBegin(GL_QUADS);
          glTexCoord2f(0.0, fbo_->getHeight());
          glVertex2f(0.333, 0.0);
          glTexCoord2f(fbo_->getWidth(), fbo_->getHeight());
          glVertex2f(0.666, 0.0);
          glTexCoord2f(fbo_->getWidth(), 0.0);
          glVertex2f(0.666, 0.5);
          glTexCoord2f(0.0, 0.0);
          glVertex2f(0.333, 0.5);
        glEnd();

        // draw color buffer 4 (labels)
        fbo_->bind(4);
        glBegin(GL_QUADS);
          glTexCoord2f(0.0, fbo_->getHeight());
          glVertex2f(0.666, 0.0);
          glTexCoord2f(fbo_->getWidth(), fbo_->getHeight());
          glVertex2f(1.0, 0.0);
          glTexCoord2f(fbo_->getWidth(), 0.0);
          glVertex2f(1.0, 0.5);
          glTexCoord2f(0.0, 0.0);
          glVertex2f(0.666, 0.5);
        glEnd();

        // draw depth buffer
        fbo_->bindDepth();
        glBegin(GL_QUADS);
          glTexCoord2f(0.0, fbo_->getHeight());
          glVertex2f(0.666, 0.5);
          glTexCoord2f(fbo_->getWidth(), fbo_->getHeight());
          glVertex2f(1.0, 0.5);
          glTexCoord2f(fbo_->getWidth(), 0.0);
          glVertex2f(1.0, 1.0);
          glTexCoord2f(0.0, 0.0);
          glVertex2f(0.666, 1.0);
        glEnd();

      glPopMatrix();
      glMatrixMode(GL_PROJECTION);
    glPopMatrix();
  }

  fbo_->bind(1);
  glGetTexImage (fbo_->getTextureTarget(), 0, GL_RED, GL_FLOAT, masked_depth_);
  if (need_mask_)
  {
    fbo_->bind(3);
    glGetTexImage (fbo_->getTextureTarget(), 0, GL_RED, GL_UNSIGNED_BYTE, mask_);
  }

  fbo_->bind(4);
  glGetTexImage (fbo_->getTextureTarget(), 0, GL_RED, GL_INT, labeled_depth_);

  // Ok, finished with all OpenGL, let's swap!
  if (show_gui_) {
    glutSwapBuffers ();
    glutPostRedisplay();
    glutMainLoopEvent ();
  }
  // TODO: this necessary? glFlush ();
}

