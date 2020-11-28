/**
 * BSD 3-Clause License
 *
 * Copyright (c) 2013, Marcus Liebhardt, Yujin Robot.
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

#define MAX_USERS 10
bool g_visibleUsers[MAX_USERS] = {false};
nite::SkeletonState g_skeletonStates[MAX_USERS] = {nite::SKELETON_NONE};

void updateUserState(const nite::UserData &user, unsigned long long ts) {
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

int main(int argc, char **argv) {
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
  
  ros::init(argc, argv, "openni2_tracker");
  ros::NodeHandle private_nh("~");
  tf2_ros::TransformBroadcaster br;
  
  std::string camera_frame_id;
  private_nh.param<std::string>("camera_frame_id", camera_frame_id, "camera_depth_frame");
  double frequency;
  private_nh.param<double>("frequency", frequency, 30);
  double confidence;
  private_nh.param<double>("confidence", confidence, 0.5);
  
  nite::UserTracker userTracker;
  nite::Status niteRc;
  
  nite::NiTE::initialize();
  
  niteRc = userTracker.create();
  if(niteRc != nite::STATUS_OK) {
    ROS_ERROR_STREAM("Couldn't create user tracker");
    return 3;
  }
  
  ROS_WARN_STREAM("Start moving around to get detected... (PSI pose may be required for skeleton calibration, depending on the configuration)");
  
  nite::UserTrackerFrameRef userTrackerFrame;

  ros::Rate loop_rate(frequency);
  while(ros::ok()) {
    niteRc = userTracker.readFrame(&userTrackerFrame);
    if(niteRc == nite::STATUS_OK) {
      const nite::Array<nite::UserData>& users = userTrackerFrame.getUsers();
      for(int i = 0; i < users.getSize(); ++i) {
	const nite::UserData& user = users[i];
	updateUserState(user, userTrackerFrame.getTimestamp());
	if(user.isNew()) {
	  ROS_INFO_STREAM("Found a new user.");
	  userTracker.startSkeletonTracking(user.getId());
	}
	else if(user.getSkeleton().getState() == nite::SKELETON_TRACKED) {
	  ROS_INFO_STREAM("Now tracking user " << user.getId());
	  const nite::Skeleton skeleton = user.getSkeleton();
	  
	  geometry_msgs::TransformStamped transform;
	  transform.header.stamp = ros::Time::now();
	  transform.header.frame_id = camera_frame_id;
	  
	  for(const auto jointNamePair : joints) {
	    const nite::SkeletonJoint& joint = skeleton.getJoint(jointNamePair.first);

	    if(joint.getPositionConfidence() > confidence) {
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

	      br.sendTransform(transform);
	    }
	  }
	}
      }
    }
    else {
      ROS_WARN_STREAM("Get next frame failed.");
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  nite::NiTE::shutdown();
  return EXIT_SUCCESS;
}
