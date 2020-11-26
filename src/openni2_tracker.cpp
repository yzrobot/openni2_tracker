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
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <kdl/frames.hpp>
#include <NiTE.h>

using std::string;

#define MAX_USERS 10
bool g_visibleUsers[MAX_USERS] = {false};
nite::SkeletonState g_skeletonStates[MAX_USERS] = {nite::SKELETON_NONE};

void updateUserState(const nite::UserData &user, unsigned long long ts) {
  if(user.isNew()) {
    printf("[%08llu] User #%d:\t%s\n", ts, user.getId(), "New");
  } else if (user.isVisible() && !g_visibleUsers[user.getId()]) {
    printf("[%08llu] User #%d:\t%s\n", ts, user.getId(), "Visible");
  } else if (!user.isVisible() && g_visibleUsers[user.getId()]) {
    printf("[%08llu] User #%d:\t%s\n", ts, user.getId(), "Out of Scene");
  } else if (user.isLost()) {
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

/* publish skeleton to /tf */
void publishTransform(const nite::SkeletonJoint &joint, double confidence, tf::TransformBroadcaster &br, std::string &camera_frame_id, std::string joint_frame_id) {
  if(joint.getPositionConfidence() > confidence) {
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(joint.getPosition().x / 1000.0,
				    joint.getPosition().y / 1000.0,
				    joint.getPosition().z / 1000.0));
    transform.setRotation(tf::Quaternion(0, 0, 0, 1));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), camera_frame_id, joint_frame_id));
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "openni2_tracker");
  ros::NodeHandle private_nh("~");
  tf::TransformBroadcaster br;
  
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
	} else if(user.getSkeleton().getState() == nite::SKELETON_TRACKED) {
	  ROS_INFO_STREAM("Now tracking user " << user.getId());
	  publishTransform(user.getSkeleton().getJoint(nite::JOINT_HEAD), confidence, br, camera_frame_id, "/user_"+std::to_string(user.getId())+"/head");
	  publishTransform(user.getSkeleton().getJoint(nite::JOINT_NECK), confidence, br, camera_frame_id, "/user_"+std::to_string(user.getId())+"/neck");
	  publishTransform(user.getSkeleton().getJoint(nite::JOINT_TORSO), confidence, br, camera_frame_id, "/user_"+std::to_string(user.getId())+"/torso");
	  
	  publishTransform(user.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER), confidence, br, camera_frame_id, "/user_"+std::to_string(user.getId())+"/left_shoulder");
	  publishTransform(user.getSkeleton().getJoint(nite::JOINT_LEFT_ELBOW), confidence, br, camera_frame_id, "/user_"+std::to_string(user.getId())+"/left_elbow");
	  publishTransform(user.getSkeleton().getJoint(nite::JOINT_LEFT_HAND), confidence, br, camera_frame_id, "/user_"+std::to_string(user.getId())+"/left_hand");

	  publishTransform(user.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER), confidence, br, camera_frame_id, "/user_"+std::to_string(user.getId())+"/right_shoulder");
	  publishTransform(user.getSkeleton().getJoint(nite::JOINT_RIGHT_ELBOW), confidence, br, camera_frame_id, "/user_"+std::to_string(user.getId())+"/right_elbow");
	  publishTransform(user.getSkeleton().getJoint(nite::JOINT_RIGHT_HAND), confidence, br, camera_frame_id, "/user_"+std::to_string(user.getId())+"/right_hand");

	  publishTransform(user.getSkeleton().getJoint(nite::JOINT_LEFT_HIP), confidence, br, camera_frame_id, "/user_"+std::to_string(user.getId())+"/left_hip");
	  publishTransform(user.getSkeleton().getJoint(nite::JOINT_LEFT_KNEE), confidence, br, camera_frame_id, "/user_"+std::to_string(user.getId())+"/left_knee");
	  publishTransform(user.getSkeleton().getJoint(nite::JOINT_LEFT_FOOT), confidence, br, camera_frame_id, "/user_"+std::to_string(user.getId())+"/left_foot");

	  publishTransform(user.getSkeleton().getJoint(nite::JOINT_RIGHT_HIP), confidence, br, camera_frame_id, "/user_"+std::to_string(user.getId())+"/right_hip");
	  publishTransform(user.getSkeleton().getJoint(nite::JOINT_RIGHT_KNEE), confidence, br, camera_frame_id, "/user_"+std::to_string(user.getId())+"/right_knee");
	  publishTransform(user.getSkeleton().getJoint(nite::JOINT_RIGHT_FOOT), confidence, br, camera_frame_id, "/user_"+std::to_string(user.getId())+"/right_foot");
	}
      }
    } else {
      ROS_WARN_STREAM("Get next frame failed.");
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  nite::NiTE::shutdown();
  return EXIT_SUCCESS;
}
