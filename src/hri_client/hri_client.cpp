#include <ros/ros.h>

#include <hri_client/hri_client.h>

namespace hri_client {

  HriClient::HriClient(ros::NodeHandle nh, ros::NodeHandle nh_private)
   : nh_(nh)
   , nh_private_(nh_private)
   , max_rotational_speed_(30.0*M_PI/180)
   , visca_camera_path_("/dev/ttyUSB0")
   , visca_feedback_rate_(5.0)
   , orientation_control_rate_(5.0)
   , gain_rotational_speed_(1.0)
   , orientation_(0.0)
   , pan_tilt_(0, 0) 
   , sound_file_("")
  {
    // Get ROS params
    if (!nh_private_.getParam("visca_camera_path", visca_camera_path_)) {
      ROS_WARN_STREAM("HriClient: no visca_camera_path parameter provided, using default value of " << visca_camera_path_);
    }
    if (!nh_private_.getParam("visca_feedback_rate", visca_feedback_rate_)) {
      ROS_WARN_STREAM("HriClient: no visca_feedback_rate parameter provided, using default value of " << visca_feedback_rate_);
    }
    if (!nh_private_.getParam("orientation_control_rate", orientation_control_rate_)) {
      ROS_WARN_STREAM("HriClient: no orientation_control_rate parameter provided, using default value of " << orientation_control_rate_);
    }
    if (!nh_private_.getParam("max_rotational_speed", max_rotational_speed_)) {
      ROS_WARN_STREAM("HriClient: no max_rotational_speed parameter provided, using default value of " << max_rotational_speed_);
    }
    if (!nh_private_.getParam("gain_rotational_speed", gain_rotational_speed_)) {
      ROS_WARN_STREAM("HriClient: no gain_rotational_speed parameter provided, using default value of " << gain_rotational_speed_);
    }

    // sound_play stuff
    try {
      sound_client_.reset( new sound_play::SoundClient() );
    }
    catch (...) {
      ROS_ERROR("HriClient: failed to create SoundClient");
      throw;
    }
    ros::Duration(1.0).sleep();  // soundplay_node is just slow

    // VISCA camera stuff
    try {
      visca_interface_.reset( new visca_interface::ViscaInterface(visca_camera_path_) );
    }
    catch (...) {
      ROS_ERROR("HriClient: failed to create ViscaInterface");
      throw;
    }

    // Register ROS publishers / subscribers to topics
    input_actionsequence_subscriber_ = nh_.subscribe("action_sequence", 100, &HriClient::actionSequenceCallBack, this);
    pose_subscriber_ = nh_.subscribe("RosAria/pose", 100, &HriClient::poseCallBack, this);
    command_velocity_publisher_ = nh_.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 1);

    // Start ROS timers
    visca_feedback_timer_ = nh_.createTimer(ros::Duration(visca_feedback_rate_), &HriClient::viscaFeedbackTimerCallBack, this);
    orientation_control_timer_ = nh_.createTimer(ros::Duration(orientation_control_rate_), &HriClient::orientationControlTimerCallBack, this);
  }

  HriClient::~HriClient()
  {
    // Publish a last control message for 0 velocity to stop robot if already moving
    stopRobot();
    // Rest of destruction sequence handled by smart ptrs
  }

  void HriClient::stopRobot()
  {
    geometry_msgs::Twist command_velocity_msg; 
    // 0 velocity to stop robot
    command_velocity_msg.linear.x = 0;
    command_velocity_msg.linear.y = 0;
    command_velocity_msg.linear.z = 0;
    command_velocity_msg.angular.x = 0;
    command_velocity_msg.angular.y = 0;
    command_velocity_msg.angular.z = 0;
    
    // publish the message for RosAria node to handle
    command_velocity_publisher_.publish( command_velocity_msg );
  }

  void HriClient::controlRobot(double orientation_error)
  {
    geometry_msgs::Twist command_velocity_msg;
    // want to rotate in-place, so linear velocities should be 0, and all rotational velocities also 0  
    command_velocity_msg.linear.x = 0;
    command_velocity_msg.linear.y = 0;
    command_velocity_msg.linear.z = 0;
    command_velocity_msg.angular.x = 0;
    command_velocity_msg.angular.y = 0;
    // handle rotation by giving a positive (counterclockwise) or negative (clockwise) rotational velocity depending on whether the error is positive or negative
    // A: compute P-error
    command_velocity_msg.angular.z = gain_rotational_speed_ * orientation_error;
    // B: constrain within +-max_rotational_speed_
    command_velocity_msg.angular.z = std::min(command_velocity_msg.angular.z,max_rotational_speed_);
    command_velocity_msg.angular.z = std::max(command_velocity_msg.angular.z,-max_rotational_speed_);
    
    // publish the message for RosAria node to handle
    command_velocity_publisher_.publish( command_velocity_msg );
  }

  void HriClient::poseCallBack(const nav_msgs::Odometry::ConstPtr& pose_msg)
  {
   geometry_msgs::Quaternion pose_orientation = pose_msg->pose.pose.orientation;
   
   tf::Quaternion q(pose_orientation.x, pose_orientation.y, pose_orientation.z, pose_orientation.w);
   tf::Matrix3x3 m(q);
   double roll, pitch, yaw;
   m.getRPY(roll, pitch, yaw);

   orientation_ = yaw;
  }
  
  void HriClient::actionSequenceCallBack(const hri_client::ActionSequence::ConstPtr& action_sequence_msg)
  {
    action_sequence_cache_ = std::deque<hri_client::Action>(action_sequence_msg-> action_sequence.begin(), action_sequence_msg-> action_sequence.end());
  }

  void HriClient::viscaFeedbackTimerCallBack(const ros::TimerEvent& timer_event)
  {
    pan_tilt_ = visca_interface_->getPanTilt();
  }

  void HriClient::orientationControlTimerCallBack(const ros::TimerEvent& timer_event)
  {
    // retrieve action to perform from cache
    if (!action_sequence_cache_.empty()) {
      auto curr_action = action_sequence_cache_.front();
      
      // Robot orientation error
      auto orientation_error = curr_action.robot_orientation * (M_PI/180.0) - orientation_;  // received orientation in action message is in degrees
      sound_file_ = curr_action.sound_toplay;
      // Camera pose error
      visca_interface::PanTilt pan_tilt_reference(static_cast<int16_t>(curr_action.cam_pan),
                                                  static_cast<int16_t>(curr_action.cam_tilt));
      visca_interface::PanTiltSpeed pan_tilt_speed_reference(static_cast<int16_t>(curr_action.cam_panspeed),
                                                             static_cast<int16_t>(curr_action.cam_tiltspeed));
      visca_interface::PanTilt pan_tilt_error(pan_tilt_reference.pan - pan_tilt_.pan,
                                              pan_tilt_reference.tilt - pan_tilt_.tilt);

      // When orientation error is large enough, handle moving the robot to the desired orientation
      if ( std::abs(orientation_error) > 2.5*(M_PI/180) ) {
        // A: move robot using P-control
        controlRobot(orientation_error);
      }
      // When robot orientation is near the desired reference value, control the camera to go to desired pan-tilt pose
      else { 
        // A: stop robot motion (publish zero velocity reference to ensure it doesn't roll away over time)
        stopRobot();

        // When camera orientation error is large enough, handle moving the camera
        if ( std::abs(pan_tilt_error.pan) > 1 || std::abs(pan_tilt_error.tilt) > 1) {
          // B: send command to camera
          visca_interface_->setPanTilt(pan_tilt_reference, pan_tilt_speed_reference);
        }
        // When camera orientation error is near the desired reference value, playback the sound file (if one exists)
        else {
          // C: playback sound file
          if (!sound_file_.empty()){
            sound_client_->playWave(sound_file_);
            sound_file_.clear();

            ros::Duration(3.0).sleep();  // all sounds in experiment about 3 sec long
          }
          
          // D: action complete, pop to resume to next on next timer callback
          action_sequence_cache_.pop_front();
        }
      }
    }
  }

} //namespace hri_client