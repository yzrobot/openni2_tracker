/**
 * BSD 3-Clause License
 *
 * Copyright (c) 2013, Marcus Liebhardt, Yujin Robot.
 * Copyright (c) 2016, Kei Okada
 * Copyright (c) 2020, Zhi Yan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **/

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <NiTE.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <image_transport/image_transport.h>
#include <OpenNI.h>
#include <boost/thread/mutex.hpp>

namespace openni2_tracker {
  class OpenNI2TrackerNodelet : public nodelet::Nodelet {

  private:
    std::vector<bool> g_visibleUsers;
    std::vector<nite::SkeletonState> g_skeletonStates;

    boost::shared_ptr<ros::NodeHandle> nh_;
    boost::shared_ptr<ros::NodeHandle> pnh_;
    boost::shared_ptr<image_transport::ImageTransport> it_;
    image_transport::Subscriber depth_img_sub_;
    //image_transport::CameraSubscriber depth_img_sub_;
    ros::Timer publish_timer_;
    tf2_ros::TransformBroadcaster broadcaster_;
    
    boost::mutex mutex_;
    boost::shared_ptr<nite::UserTrackerFrameRef> userTrackerFrame_;
    boost::shared_ptr<nite::UserTracker> userTracker_;
    nite::Status niteRc_;
    openni::Device devDevice_;

    std::string camera_frame_id_;
    std::string device_id_;
    int max_users_;

    const std::map<nite::JointType, std::string> joints = {
      {nite::JOINT_HEAD,           "head"},
      {nite::JOINT_NECK,           "neck"}, 
      {nite::JOINT_LEFT_HAND,      "left_hand"},
      {nite::JOINT_LEFT_ELBOW,     "left_elbow"},
      {nite::JOINT_LEFT_SHOULDER,  "left_shoulder"},
      {nite::JOINT_RIGHT_HAND,     "right_hand"},
      {nite::JOINT_RIGHT_ELBOW,    "right_elbow"},
      {nite::JOINT_RIGHT_SHOULDER, "right_shoulder"},
      {nite::JOINT_RIGHT_FOOT,     "right_foot"},
      {nite::JOINT_RIGHT_KNEE,     "right_knee"},
      {nite::JOINT_RIGHT_HIP,      "right_hip"},
      {nite::JOINT_LEFT_FOOT,      "left_foot"},
      {nite::JOINT_LEFT_KNEE,      "left_knee"},
      {nite::JOINT_LEFT_HIP,       "left_hip"},
      {nite::JOINT_TORSO,          "torso"},
    };
    
  public:
    OpenNI2TrackerNodelet() : max_users_(10) {}
    
    ~OpenNI2TrackerNodelet() {
      NODELET_INFO("shutdown: device %s", device_id_.c_str());
      userTrackerFrame_->release();
      userTracker_->destroy();
      devDevice_.close();
      nite::NiTE::shutdown();
    }

    void updateUserState(const nite::UserData& user, unsigned long long ts) {
      if(user.isNew()) {
	printf("[%08llu] User #%d:\t%s\n", ts, user.getId(), "New");
      }
      else if (user.isVisible() && !g_visibleUsers[user.getId()]) {
	printf("[%08llu] User #%d:\t%s\n", ts, user.getId(), "Visible");
      }
      else if (!user.isVisible() && g_visibleUsers[user.getId()]) {
	printf("[%08llu] User #%d:\t%s\n", ts, user.getId(), "Out of Scene");
      }
      else if (user.isLost()) {
	printf("[%08llu] User #%d:\t%s\n", ts, user.getId(), "Lost");
      }
  
      g_visibleUsers[user.getId()] = user.isVisible();
  
      if(g_skeletonStates[user.getId()] != user.getSkeleton().getState()) {
	switch(g_skeletonStates[user.getId()] = user.getSkeleton().getState()) {
	case nite::SKELETON_NONE:
	  printf("[%08llu] User #%d:\t%s\n", ts, user.getId(), "Stopped tracking");
	  break;
	case nite::SKELETON_CALIBRATING:
	  printf("[%08llu] User #%d:\t%s\n", ts, user.getId(), "Calibrating");
	  break;
	case nite::SKELETON_TRACKED:
	  printf("[%08llu] User #%d:\t%s\n", ts, user.getId(), "Tracking");
	  break;
	case nite::SKELETON_CALIBRATION_ERROR_NOT_IN_POSE:
	case nite::SKELETON_CALIBRATION_ERROR_HANDS:
	case nite::SKELETON_CALIBRATION_ERROR_LEGS:
	case nite::SKELETON_CALIBRATION_ERROR_HEAD:
	case nite::SKELETON_CALIBRATION_ERROR_TORSO:
	  printf("[%08llu] User #%d:\t%s\n", ts, user.getId(), "Calibration Failed");
	  break;
	}
      }
    }

    void device_initialization() {
      boost::mutex::scoped_lock lock(mutex_);
      
      if(openni::OpenNI::initialize() != openni::STATUS_OK) {
    	NODELET_FATAL("OpenNI initial error");
    	return;
      }
      
      openni::Array<openni::DeviceInfo> deviceInfoList;
      openni::OpenNI::enumerateDevices(&deviceInfoList);
      
      if(device_id_.size() == 0 || device_id_ == "#1") {
	device_id_ = deviceInfoList[0].getUri();
      }
      
      if(devDevice_.open(device_id_.c_str()) != openni::STATUS_OK) {
      	NODELET_FATAL("Couldn't open device: %s", device_id_.c_str());
      	return;
      }
      
      userTrackerFrame_.reset(new nite::UserTrackerFrameRef);
      userTracker_.reset(new nite::UserTracker);
      nite::NiTE::initialize();
      
      niteRc_ = userTracker_->create(&devDevice_);
      if(niteRc_ != nite::STATUS_OK) {
      	NODELET_FATAL("Couldn't create user tracker");
      	return;
      }
      
      NODELET_INFO("OpenNI userTracker initialized");
    }
    
    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
      depth_img_sub_.shutdown();     
      device_initialization();
    }
    
    void timeCallback(const ros::TimerEvent) {
      boost::mutex::scoped_lock lock(mutex_);
      
      if (!devDevice_.isValid()) {
    	return;
      }

      niteRc_ = userTracker_->readFrame(&(*userTrackerFrame_));
      if (niteRc_ != nite::STATUS_OK) {
    	NODELET_WARN("Get next frame failed.");
    	return;
      }
      
      const nite::Array<nite::UserData> &users = userTrackerFrame_->getUsers();
      for (int i = 0; i < users.getSize(); ++i) {
    	const nite::UserData &user = users[i];
    	updateUserState(user, userTrackerFrame_->getTimestamp());
    	if(user.isNew()) {
	  NODELET_INFO("Found a new user.");
    	  userTracker_->startSkeletonTracking(user.getId());
    	}
	else if(user.getSkeleton().getState() == nite::SKELETON_TRACKED) {
    	  NODELET_INFO("Now tracking user %d", user.getId());

	  geometry_msgs::TransformStamped transform;
	  transform.header.stamp = ros::Time::now();
	  transform.header.frame_id = camera_frame_id_;
	  
	  for(const auto jointNamePair : joints) {
	    const nite::SkeletonJoint& joint = user.getSkeleton().getJoint(jointNamePair.first);
	    
	    nite::Point3f position = joint.getPosition();
	    nite::Quaternion orientation = joint.getOrientation();
	    
	    // If the orientation isn't a unit quaternion, replace it with the identy quaternion
	    // (Should only occur for hands and feet)
	    if(orientation.x * orientation.x + 
	       orientation.y * orientation.y +
	       orientation.z * orientation.z < 0.001) {
	      orientation = nite::Quaternion(1,0,0,0);
	    }
	    
	    std::stringstream frame;
	    frame << jointNamePair.second << "_" << user.getId();
	    
	    transform.child_frame_id = frame.str();
	    transform.transform.translation.x = position.x / 1000.0;
	    transform.transform.translation.y = position.y / 1000.0;
	    transform.transform.translation.z = position.z / 1000.0;

	    transform.transform.rotation.x = orientation.x;
	    transform.transform.rotation.y = orientation.y;
	    transform.transform.rotation.z = orientation.z;
	    transform.transform.rotation.w = orientation.w;

	    broadcaster_.sendTransform(transform);
	  }
    	}
      }
    }
    
    virtual void onInit() {
      for(int i = 0; i < max_users_; ++i) {
      	g_visibleUsers.push_back(false);
      	g_skeletonStates.push_back(nite::SKELETON_NONE);
      }
      
      nh_.reset(new ros::NodeHandle(getMTNodeHandle()));
      pnh_.reset(new ros::NodeHandle(getMTPrivateNodeHandle()));
      it_.reset(new image_transport::ImageTransport(*nh_));
      
      pnh_->param<std::string>("camera_frame_id", camera_frame_id_, "camera_depth_frame");
      NODELET_INFO("camera_frame_id: %s", camera_frame_id_.c_str());
      pnh_->param<std::string>("device_id", device_id_, "#1");
      NODELET_INFO("device_id: %s", device_id_.c_str());
      
      double publish_period;
      nh_->param<double>("publish_period", publish_period, 33);
      publish_timer_ = nh_->createTimer(ros::Duration(publish_period / 1000.0), boost::bind(&OpenNI2TrackerNodelet::timeCallback, this, _1));

      bool is_standalone;
      pnh_->param<bool>("is_standalone", is_standalone, "false");
      if(!is_standalone) {
      	NODELET_WARN("Waiting for %s to initialize OpenNI", nh_->resolveName("image").c_str());
      	depth_img_sub_ = it_->subscribe("image", 1, &OpenNI2TrackerNodelet::imageCallback, this);
      } else {
      	device_initialization();
      }
    }
  };
  
  PLUGINLIB_EXPORT_CLASS(openni2_tracker::OpenNI2TrackerNodelet, nodelet::Nodelet)
}
