#include "ekf_offline.hpp"

int main(int argc, char **argv){
  ros::init(argc, argv, "ekf_tester");
  ros::NodeHandle n;

  ekf_offline ekf_offline;
  ekf_offline.loop();
  return 0;
}
