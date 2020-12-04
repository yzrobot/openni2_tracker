/**
 * BSD 3-Clause License
 *
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

int main(int argc, char **argv) {
  ros::init(argc, argv, "openni2_hand_tracker");
  ros::NodeHandle private_nh("~");
  tf2_ros::TransformBroadcaster br;
  
  std::string camera_frame_id;
  private_nh.param<std::string>("camera_frame_id", camera_frame_id, "camera_depth_frame");
  double frequency;
  private_nh.param<double>("frequency", frequency, 30);
  
  nite::HandTracker handTracker;
  nite::Status niteRc;
  
  nite::NiTE::initialize();
  if(niteRc != nite::STATUS_OK) {
    ROS_ERROR_STREAM("NiTE initialization failed\n");
    return 1;
  }
  
  niteRc = handTracker.create();
  if(niteRc != nite::STATUS_OK) {
    ROS_ERROR_STREAM("Couldn't create hand tracker");
    return 3;
  }

  handTracker.startGestureDetection(nite::GESTURE_WAVE);
  handTracker.startGestureDetection(nite::GESTURE_CLICK);
  //handTracker.startGestureDetection(nite::GESTURE_HAND_RAISE);
  ROS_WARN_STREAM("Wave or click or hand-raise to start tracking your hand...");
  
  nite::HandTrackerFrameRef handTrackerFrame;

  ros::Rate loop_rate(frequency);
  while(ros::ok()) {
    niteRc = handTracker.readFrame(&handTrackerFrame);
    if(niteRc == nite::STATUS_OK) {
      const nite::Array<nite::GestureData>& gestures = handTrackerFrame.getGestures();
      for(int i = 0; i < gestures.getSize(); ++i) {
	if(gestures[i].isComplete()) {
	  nite::HandId newId;
	  handTracker.startHandTracking(gestures[i].getCurrentPosition(), &newId);
	}
      }
      
      const nite::Array<nite::HandData>& hands = handTrackerFrame.getHands();
      for(int i = 0; i < hands.getSize(); ++i) {
	const nite::HandData& hand = hands[i];
	if(hand.isTracking()) {
	  geometry_msgs::TransformStamped transform;
	  transform.header.stamp = ros::Time::now();
	  transform.header.frame_id = camera_frame_id;
	  
	  nite::Point3f position = hand.getPosition();
	  std::stringstream frame;
	  frame << "hand_" << hand.getId();
	  
	  transform.child_frame_id = frame.str();
	  transform.transform.translation.x = position.x / 1000.0;
	  transform.transform.translation.y = position.y / 1000.0;
	  transform.transform.translation.z = position.z / 1000.0;
	  
	  transform.transform.rotation.x = 0;
	  transform.transform.rotation.y = 0;
	  transform.transform.rotation.z = 0;
	  transform.transform.rotation.w = 1;
	  
	  br.sendTransform(transform);
	  //printf("%d. (%5.2f, %5.2f, %5.2f)\n", hand.getId(), hand.getPosition().x, hand.getPosition().y, hand.getPosition().z);
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
