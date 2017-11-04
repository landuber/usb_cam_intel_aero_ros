/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Robert Bosch LLC.
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
*   * Neither the name of the Robert Bosch nor the names of its
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
*********************************************************************/

#include <string>
#include <pthread.h>
#include <poll.h>
#include <signal.h>
#include <ros/ros.h>
#include <camera.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/fill_image.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sstream>
#include <std_srvs/Empty.h>
#include <linux/videodev2.h>
#include <opencv2/opencv.hpp>



#define POLL_ERROR_EVENTS (POLLERR | POLLHUP | POLLNVAL)

#define CAMERA_MSEC_TIMEOUT 10000
#define DEFAULT_PIXEL_FORMAT V4L2_PIX_FMT_YUV420
#define UNUSED __attribute__((__unused__))

using namespace cv;

typedef unsigned char uchar;

static volatile bool _should_run;



namespace usb_cam {


class UsbCamNode
{
public:
  // private ROS node handle
  ros::NodeHandle node_;

  // shared image message
  sensor_msgs::Image img_;
  image_transport::CameraPublisher image_pub_;

  // parameters
  std::string video_device_name_, io_method_name_, pixel_format_name_, camera_name_, camera_info_url_;
  //std::string start_service_name_, start_service_name_;
  bool streaming_status_;
  int device_id, image_width_, image_height_, framerate_, exposure_, brightness_, contrast_, saturation_, sharpness_, focus_,
      white_balance_, gain_;
  bool autofocus_, autoexposure_, auto_white_balance_;
  boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;

  Camera * camera_;
  pthread_mutex_t node_lock_;

  ros::ServiceServer service_start_, service_stop_;
  UsbCamNode();
  ~UsbCamNode();

  bool service_start_cap(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res );
  bool service_stop_cap( std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res );
  void camera_callback(const void *img, UNUSED size_t len, const struct timeval *timestamp);
  void *camera_thread();
  void _signal_handlers_setup(void);
  bool send_image();
  void init_camera();
  void run();
  bool spin();
};

static void camera_callback_(const void *img, size_t len, const struct timeval *timestamp, void *data)
{
    UsbCamNode *node = (UsbCamNode *)data;
    node->camera_callback(img, len, timestamp);
}

static void *thread_callback_(void *data)
{
    UsbCamNode *node = (UsbCamNode *) data;
    return node->camera_thread();
}

static void exit_signal_handler(UNUSED int signum)
{
    _should_run = false;
}


UsbCamNode::UsbCamNode() : node_("~")
{
    // advertise the main image topic
    image_transport::ImageTransport it(node_);
    image_pub_ = it.advertiseCamera("image_raw", 1);

    // grab the parameters
    node_.param("video_device", video_device_name_, std::string("/dev/video0"));
    node_.param("device_id", device_id, 0); 
    node_.param("brightness", brightness_, -1); //0-255, -1 "leave alone"
    node_.param("contrast", contrast_, -1); //0-255, -1 "leave alone"
    node_.param("saturation", saturation_, -1); //0-255, -1 "leave alone"
    node_.param("sharpness", sharpness_, -1); //0-255, -1 "leave alone"
    // possible values: mmap, read, userptr
    node_.param("io_method", io_method_name_, std::string("mmap"));
    node_.param("image_width", image_width_, 640);
    node_.param("image_height", image_height_, 480);
    node_.param("framerate", framerate_, 30);
    // possible values: yuyv, uyvy, mjpeg, yuvmono10, rgb24
    node_.param("pixel_format", pixel_format_name_, std::string("mjpeg"));
    // enable/disable autofocus
    node_.param("autofocus", autofocus_, false);
    node_.param("focus", focus_, -1); //0-255, -1 "leave alone"
    // enable/disable autoexposure
    node_.param("autoexposure", autoexposure_, true);
    node_.param("exposure", exposure_, 100);
    node_.param("gain", gain_, -1); //0-100?, -1 "leave alone"
    // enable/disable auto white balance temperature
    node_.param("auto_white_balance", auto_white_balance_, true);
    node_.param("white_balance", white_balance_, 4000);

    // load the camera info
    node_.param("camera_frame_id", img_.header.frame_id, std::string("head_camera"));
    node_.param("camera_name", camera_name_, std::string("head_camera"));
    node_.param("camera_info_url", camera_info_url_, std::string(""));
    cinfo_.reset(new camera_info_manager::CameraInfoManager(node_, camera_name_, camera_info_url_));

    // create Services
    service_start_ = node_.advertiseService("start_capture", &UsbCamNode::service_start_cap, this);
    service_stop_ = node_.advertiseService("stop_capture", &UsbCamNode::service_stop_cap, this);

    // check for default camera info
    if (!cinfo_->isCalibrated())
    {
      cinfo_->setCameraName(video_device_name_);
      sensor_msgs::CameraInfo camera_info;
      camera_info.header.frame_id = img_.header.frame_id;
      camera_info.width = image_width_;
      camera_info.height = image_height_;
      cinfo_->setCameraInfo(camera_info);
    }


    ROS_INFO("Starting '%s' (%s) at %dx%d via %s (%s) at %i FPS", camera_name_.c_str(), video_device_name_.c_str(),
        image_width_, image_height_, io_method_name_.c_str(), pixel_format_name_.c_str(), framerate_);




}

void UsbCamNode::init_camera()
{
    // New camera
    camera_ = new Camera(video_device_name_.c_str());
    if(!camera_) {
	ROS_FATAL("No memory to allocate Camera");
        node_.shutdown();
        return;
    }

    if(camera_->init(device_id, image_width_, image_height_, DEFAULT_PIXEL_FORMAT)){
        ROS_FATAL("Unable to initialize camera");
        node_.shutdown();
        return;
    }
}

void UsbCamNode::run()
{
    camera_->callback_set(camera_callback_, this);

    if(camera_->start()) {
        ROS_FATAL("Unable to start camera streaming");
        node_.shutdown();
        return;
    }

    _signal_handlers_setup();
    _should_run = true;
    // synchronization
    pthread_mutex_init(&node_lock_, NULL);
    pthread_t thread;
    if(pthread_create(&thread, NULL, thread_callback_, this)) {
	ROS_FATAL("Unable to create a thread");
        node_.shutdown();
	return;
    }
}

void * UsbCamNode::camera_thread()
{
	Pollable *pollables[] = { camera_ };
	const uint8_t len = sizeof(pollables) / sizeof(Pollable *);
	struct pollfd desc[len];

	for (uint8_t i = 0; i < len; i++) {
		desc[i].fd = pollables[i]->_fd;
		desc[i].events = POLLIN | POLLPRI | POLL_ERROR_EVENTS;
		desc[i].revents = 0;
	}

	while (_should_run) {
		//int ret = poll(desc, sizeof(desc) / sizeof(struct pollfd), CAMERA_MSEC_TIMEOUT);
		int ret = poll(desc, sizeof(desc) / sizeof(struct pollfd), -1);
		if (ret == 0) {
			ROS_INFO("Camera timeout, restarting...");
 			//camera_->handle_read();
			camera_->restart();
			continue;
		}
		if (ret < 1) {
			continue;
		}

		for (unsigned i = 0; ret && i < (sizeof(desc) / sizeof(struct pollfd)); i++, ret--) {
			for (uint8_t j = 0; j < len; j++) {
				if (desc[i].fd == pollables[j]->_fd) {
					if (desc[i].revents & (POLLIN | POLLPRI)) {
						pollables[j]->handle_read();
					}
					if (desc[i].revents & POLLOUT) {
						pollables[j]->handle_canwrite();
					}
					if (desc[i].revents & POLL_ERROR_EVENTS) {
						ROS_FATAL("Poll error event on camera: %u", desc[i].revents);
					}
					break;
				}
			}
		}
	}

	return NULL;
}

void UsbCamNode::_signal_handlers_setup(void)
{
    struct sigaction sa = { };

    sa.sa_flags = SA_NOCLDSTOP;
    sa.sa_handler = exit_signal_handler;
    sigaction(SIGTERM, &sa, NULL);
    sigaction(SIGINT, &sa, NULL);

    sa.sa_handler = SIG_IGN;
    sigaction(SIGPIPE, &sa, NULL);
}


bool UsbCamNode::service_start_cap(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res )
{
   camera_->restart();
   return true;
}


bool UsbCamNode::service_stop_cap( std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res )
{
    camera_->stop();
    return true;
}


  void UsbCamNode::camera_callback(const void *img, UNUSED size_t len, const struct timeval *timestamp)
  {
    uchar *image = (uchar*)img;
    // todo: make this flexible for the color case
    pthread_mutex_lock(&node_lock_);
    fillImage(img_, "mono8", image_height_, image_width_, image_width_, image);
    pthread_mutex_unlock(&node_lock_);

  }

  bool UsbCamNode::send_image()
  {
    // grab the camera info
    sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));
    pthread_mutex_lock(&node_lock_);
    ci->header.frame_id = img_.header.frame_id;
    ci->header.stamp = img_.header.stamp;

    // publish the image
    image_pub_.publish(img_, *ci);
    pthread_mutex_unlock(&node_lock_);

    return true;
  }

  UsbCamNode::~UsbCamNode()
  {
    camera_->shutdown();
  }

  bool UsbCamNode::spin()
  {
    ros::Rate loop_rate(this->framerate_);
    while (node_.ok())
    {
      if (!send_image()) ROS_WARN("USB camera did not respond in time.");
      ros::spinOnce();
      loop_rate.sleep();

    }
    return true;
  }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "usb_cam");
  usb_cam::UsbCamNode a;
  a.init_camera();
  a.run();
  a.spin();
  return EXIT_SUCCESS;
}
