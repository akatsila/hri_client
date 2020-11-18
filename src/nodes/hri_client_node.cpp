#include <ros/ros.h>

#include <hri_client/hri_client.h>

int main(int argc, char **argv)
{
  // initialize the ros node
  ros::init(argc, argv, "hri_client_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  hri_client::HriClient hri(nh, nh_private);

  ros::spin();

  return 0;
}
