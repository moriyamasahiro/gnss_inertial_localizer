#include "ekf_publisher.hpp"

int main(int argc, char **argv){

  ros::init(argc, argv, "ekf");
  
  ekf_publisher *ekf_pub;
  ekf_pub = new ekf_publisher();

  //ros::spin();
  ekf_pub->loop();

  return 0;
}
