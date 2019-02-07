#include "ekf.hpp"
using namespace std;

// get jaccobian of h. observation values are postion and velocity from gnss receiver.
Eigen::MatrixXf ekf::get_jaccobian_h(string frame_id){
  if(anntena_pos.find(frame_id) == anntena_pos.end()){
    ROS_INFO_STREAM("anntena postition of " << frame_id << " was not initialized. Did you set it?");
    exit;
  }
  Eigen::MatrixXf jaccobian_h(10, 6);

  Eigen::MatrixXf s = ekf::state.state;
  Eigen::MatrixXf u = ekf::imu_output;
  float anntena_x = anntena_pos[frame_id][0];
  float anntena_y =  anntena_pos[frame_id][1];
  float anntena_z = anntena_pos[frame_id][2];

  ROS_INFO_STREAM("anntena position: " << anntena_x<< ", " << anntena_y << ", " << anntena_z);

  jaccobian_h <<1, 0, 0, 0, 0, 0, 2*anntena_y*s(9,0) - 2*anntena_z*s(8,0), 2*anntena_y*s(8,0) + 2*anntena_z*s(9,0), -4*anntena_x*s(8,0) + 2*anntena_y*s(7,0) - 2*anntena_z*s(6,0), -4*anntena_x*s(9,0) + 2*anntena_y*s(6,0) + 2*anntena_z*s(7,0),
                0, 1, 0, 0, 0, 0, -2*anntena_x*s(9,0) + 2*anntena_z*s(7,0), 2*anntena_x*s(8,0) - 4*anntena_y*s(7,0) + 2*anntena_z*s(6,0), 2*anntena_x*s(7,0) + 2*anntena_z*s(9,0), -2*anntena_x*s(6,0) - 4*anntena_y*s(9,0) + 2*anntena_z*s(8,0),
                0, 0, 1, 0, 0, 0, 2*anntena_x*s(8,0) - 2*anntena_y*s(7,0), 2*anntena_x*s(9,0) - 2*anntena_y*s(6,0) - 4*anntena_z*s(7,0), 2*anntena_x*s(6,0) + 2*anntena_y*s(9,0) - 4*anntena_z*s(8,0), 2*anntena_x*s(7,0) + 2*anntena_y*s(8,0),
                0, 0, 0, 1, 0, 0, -2*s(8,0)*(-anntena_x*u(4,0) + anntena_y*u(3,0)) + 2*s(9,0)*(anntena_x*u(5,0) - anntena_z*u(3,0)), 2*s(8,0)*(anntena_x*u(5,0) - anntena_z*u(3,0)) + 2*s(9,0)*(-anntena_x*u(4,0) + anntena_y*u(3,0)), -2*s(6,0)*(-anntena_x*u(4,0) + anntena_y*u(3,0)) + 2*s(7,0)*(anntena_x*u(5,0) - anntena_z*u(3,0)) - 4*s(8,0)*(-anntena_y*u(5,0) + anntena_z*u(4,0)), 2*s(6,0)*(anntena_x*u(5,0) - anntena_z*u(3,0)) + 2*s(7,0)*(-anntena_x*u(4,0) + anntena_y*u(3,0)) - 4*s(9,0)*(-anntena_y*u(5,0) + anntena_z*u(4,0)),
                0, 0, 0, 0, 1, 0, 2*s(7,0)*(-anntena_x*u(4,0) + anntena_y*u(3,0)) - 2*s(9,0)*(-anntena_y*u(5,0) + anntena_z*u(4,0)), 2*s(6,0)*(-anntena_x*u(4,0) + anntena_y*u(3,0)) - 4*s(7,0)*(anntena_x*u(5,0) - anntena_z*u(3,0)) + 2*s(8,0)*(-anntena_y*u(5,0) + anntena_z*u(4,0)), 2*s(7,0)*(-anntena_y*u(5,0) + anntena_z*u(4,0)) + 2*s(9,0)*(-anntena_x*u(4,0) + anntena_y*u(3,0)), -2*s(6,0)*(-anntena_y*u(5,0) + anntena_z*u(4,0)) + 2*s(8,0)*(-anntena_x*u(4,0) + anntena_y*u(3,0)) - 4*s(9,0)*(anntena_x*u(5,0) - anntena_z*u(3,0)),
                0, 0, 0, 0, 0, 1, -2*s(7,0)*(anntena_x*u(5,0) - anntena_z*u(3,0)) + 2*s(8,0)*(-anntena_y*u(5,0) + anntena_z*u(4,0)), -2*s(6,0)*(anntena_x*u(5,0) - anntena_z*u(3,0)) - 4*s(7,0)*(-anntena_x*u(4,0) + anntena_y*u(3,0)) + 2*s(9,0)*(-anntena_y*u(5,0) + anntena_z*u(4,0)), 2*s(6,0)*(-anntena_y*u(5,0) + anntena_z*u(4,0)) - 4*s(8,0)*(-anntena_x*u(4,0) + anntena_y*u(3,0)) + 2*s(9,0)*(anntena_x*u(5,0) - anntena_z*u(3,0)), 2*s(7,0)*(-anntena_y*u(5,0) + anntena_z*u(4,0)) + 2*s(8,0)*(anntena_x*u(5,0) - anntena_z*u(3,0));

  return jaccobian_h;
}
