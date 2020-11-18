#include <ros/ros.h>

// VISCA interface 
#include <visca_interface/visca_interface.h>
// sound_play
#include <sound_play/sound_play.h>

// ROS messages and types
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

// Custom messages
#include <hri_client/Action.h>
#include <hri_client/ActionSequence.h>

namespace hri_client {

  class HriClient {

    public: 
      HriClient(ros::NodeHandle nh, ros::NodeHandle nh_private);
      ~HriClient();

      HriClient(const HriClient&) = delete;
      void operator=(const HriClient&) = delete;
      
      void poseCallBack(const nav_msgs::Odometry::ConstPtr& pose_msg);
      void actionSequenceCallBack(const hri_client::ActionSequence::ConstPtr& action_sequence_msg);
      void viscaFeedbackTimerCallBack(const ros::TimerEvent& timer_event);
      void orientationControlTimerCallBack(const ros::TimerEvent& timer_event);

      // Helpers
      void stopRobot();
      void controlRobot(double orientation_error);

    private:
      // NodeHandles for ROS access
      ros::NodeHandle nh_;
      ros::NodeHandle nh_private_;

      // ROS subscribers / publishers
      ros::Subscriber input_actionsequence_subscriber_;
      ros::Subscriber pose_subscriber_;
      ros::Publisher command_velocity_publisher_;

      // ROS Timers
      ros::Timer visca_feedback_timer_;
      ros::Timer orientation_control_timer_;

      // Sound Play stuff
      std::unique_ptr<sound_play::SoundClient> sound_client_;

      // VISCA camera stuff
      std::unique_ptr<visca_interface::ViscaInterface> visca_interface_;

      // Action sequence received
      std::deque<hri_client::Action> action_sequence_cache_;

      // parameters
      std::string visca_camera_path_;  // path
      double visca_feedback_rate_;  // camera read rate Hz - determines timer callbak period
      double orientation_control_rate_;  // robot control rate Hz - determines timer callbak period
      double max_rotational_speed_;  // rad/s
      double gain_rotational_speed_; // proportional error gain

      // state
      double orientation_;
      visca_interface::PanTilt pan_tilt_;
      std::string sound_file_;
  };

} //namespace hri_client;