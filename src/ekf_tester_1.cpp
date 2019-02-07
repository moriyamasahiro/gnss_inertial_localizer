#include "ekf.hpp"

int main(int argc, char **argv){
  ros::init(argc, argv, "ekf_tester");
  ros::NodeHandle n;

  EKF ekf_tester;
  ekf_tester.initialize();

  ros::Rate r(100);
  ekf_tester.set_time();
  int count = 0;
  while(ros::ok()){

    ekf_tester.set_time();
    ekf_tester.predict_state();
    if(count == 10){
      ekf_tester.update_state();
      count = -1;
    }
    ekf_tester.debug_msg();
    count += 1;
    r.sleep();

  }
  return 0;
}
