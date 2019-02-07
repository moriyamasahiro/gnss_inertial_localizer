#include "ekf.hpp"

void ekf::get_jaccobian_h_with_lgnss(){
  Eigen::MatrixXf jaccobian_h(6, 10);

  Eigen::MatrixXf s = ekf::state.state;

  jaccobian_h << 1, 0, 0, 0, 0, 0, 0.12*s(8,0), 0.12*s(9,0), 0.12*s(6,0) + 0.92*s(8,0), 0.12*s(7,0) + 0.92*s(9,0),
                 0, 1, 0, 0, 0, 0, -0.12*s(7,0) - 0.46*s(9,0), -0.12*s(6,0) - 0.46*s(8,0), -0.46*s(7,0) + 0.12*s(9,0), -0.46*s(6,0) + 0.12*s(8,0),
                 0, 0, 1, 0, 0, 0, 0.46*s(8,0), -0.24*s(7,0) - 0.46*s(9,0), 0.46*s(6,0) - 0.24*s(8,0), -0.46*s(7,0),
                 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
                 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
                 0, 0, 0, 0, 0, 1, 0, 0, 0, 0;

  ekf::jaccobian_h = jaccobian_h;
}

void ekf::get_jaccobian_h_with_rgnss(){
  Eigen::MatrixXf jaccobian_h(6, 10);

  Eigen::MatrixXf s = ekf::state.state;

  jaccobian_h << 1, 0, 0, 0, 0, 0, 0.12*s(8,0), 0.12*s(9,0), 0.12*s(6,0) - 0.92*s(8,0), 0.12*s(7,0) - 0.92*s(9,0),
                 0, 1, 0, 0, 0, 0, -0.12*s(7,0) + 0.46*s(9,0), -0.12*s(6,0) + 0.46*s(8,0), 0.46*s(7,0) + 0.12*s(9,0), 0.46*s(6,0) + 0.12*s(8,0),
                 0, 0, 1, 0, 0, 0, -0.46*s(8,0), -0.24*s(7,0) + 0.46*s(9,0), -0.46*s(6,0) - 0.24*s(8,0), 0.46*s(7,0),
                 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
                 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
                 0, 0, 0, 0, 0, 1, 0, 0, 0, 0;

  ekf::jaccobian_h = jaccobian_h;
}

void ekf::get_jaccobian_h_imu_init(){
  Eigen::MatrixXf jaccobian_h(10, 16);

  Eigen::MatrixXf s = ekf::state.state;
  Eigen::MatrixXf u = ekf::imu_output;

  jaccobian_h << 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -0.060, 0.26, 0, 0, 0, 0, 0.060, -0.26,
                 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0.0600000000000000, 0, 0, 0, 0, 0, -0.0600000000000000, 0, 0,
                 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, -26.0, 0, 0,
                 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -5.0, 26.0,
                 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 5.0, 0, 0,
                 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, -26.0, 0, 0,
                 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, (s(10,0) - u(0,0))/sqrt(pow(-s(10,0) + u(0,0),2) + pow(-s(11,0) + u(1,0),2) + pow(-s(12,0) + u(2,0),2)), (s(11,0) - u(1,0))/sqrt(pow(-s(10,0) + u(0,0),2) + pow(-s(11,0) + u(1,0),2) + pow(-s(12,0) + u(2,0),2)), (s(12,0) - u(2,0))/sqrt(pow(-s(10,0) + u(0,0),2) + pow(-s(11,0) + u(1,0),2) + pow(-s(12,0) + u(2,0),2)), 0, 0, 0,
                 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0,
                 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0,
                 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1;

  ekf::jaccobian_h_imu_init = jaccobian_h;
}

void ekf::get_jaccobian_h_gnss_init(){
  Eigen::MatrixXf jaccobian_h(12, 16);

  Eigen::MatrixXf s = ekf::state.state;

  jaccobian_h <<1, 0, 0, 0, 0, 0, 10.0*s(8,0) - 52.0*s(9,0), 52.0*s(8,0) + 10.0*s(9,0), 10.0*s(6,0) + 52.0*s(7,0), -52.0*s(6,0) + 10.0*s(7,0), 0, 0, 0, 0, 0, 0,
                 0, 1, 0, 0, 0, 0, -10.0*s(7,0), -10.0*s(6,0) - 104.0*s(7,0), 10.0*s(9,0), 10.0*s(8,0) - 104.0*s(9,0), 0, 0, 0, 0, 0, 0,
                 0, 0, 1, 0, 0, 0, 52.0*s(7,0), 52.0*s(6,0) - 20.0*s(7,0), -20.0*s(8,0) + 52.0*s(9,0), 52.0*s(8,0), 0, 0, 0, 0, 0, 0,
                 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -5.0, 26.0,
                 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 5.0, 0, 0,
                 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, -26.0, 0, 0,
                 1, 0, 0, 0, 0, 0, 10.0*s(8,0) - 52.0*s(9,0), 52.0*s(8,0) + 10.0*s(9,0), 10.0*s(6,0) + 52.0*s(7,0), -52.0*s(6,0) + 10.0*s(7,0), 0, 0, 0, 0, 0, 0,
                 0, 1, 0, 0, 0, 0, -10.0*s(7,0), -10.0*s(6,0) - 104.0*s(7,0), 10.0*s(9,0), 10.0*s(8,0) - 104.0*s(9,0), 0, 0, 0, 0, 0, 0,
                 0, 0, 1, 0, 0, 0, 52.0*s(7,0), 52.0*s(6,0) - 20.0*s(7,0), -20.0*s(8,0) + 52.0*s(9,0), 52.0*s(8,0), 0, 0, 0, 0, 0, 0,
                 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -5.0, 26.0,
                 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 5.0, 0, 0,
                 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, -26.0, 0, 0;

  ekf::jaccobian_h_gnss_init = jaccobian_h;
}

void ekf::get_jaccobian_h(string frame_id){
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
  ekf::jaccobian_h = jaccobian_h;
}
