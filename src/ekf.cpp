#include "ekf.hpp"


ekf::ekf(){
  //ekf::longitude0 = 139.541194;
  //ekf::latitude0 = 35.738108;

  ekf::longitude0 = 0.;
  ekf::latitude0 = 0.;
  //initialize state
  struct STATE state;

  vector<float> left_pos{0., 0.23, 0.05};
  vector<float> right_pos{0., -0.23, 0.05};
  anntena_pos["/left_gnss"] = left_pos;
  anntena_pos["/right_gnss"] = right_pos;

  state.state = Eigen::MatrixXf::Zero(10,1);
  state.covariance = Eigen::MatrixXf::Identity(10,10)*1e10;
  state.covariance_with_imu = Eigen::MatrixXf::Identity(16,16)*1e10;

  state.state(0,0) = 20.3185;
  state.state(1,0) = -165.338;
  state.state(2,0) = -7.3177;
  state.state(3,0) = 0.;
  state.state(4,0) = 0.;
  state.state(5,0) = 0.;
  state.state(6,0) = 1.;
  state.state(7,0) = 0.;
  state.state(8,0) = 0.;
  state.state(9,0) = 0.;

  state.covariance_with_imu(10,10) = 0.1;
  state.covariance_with_imu(11,11) = 0.1;
  state.covariance_with_imu(12,12) = 0.1;
  state.covariance_with_imu(13,13) = 0.01;
  state.covariance_with_imu(14,14) = 0.01;
  state.covariance_with_imu(15,15) = 0.01;

  ekf::state = state;
  ekf::state.covariance_with_imu.block(0,0,10,10) = ekf::state.covariance;
  ekf::bias = Eigen::MatrixXf::Zero(6,1);
  ekf::bias << 0.,0.,0.,0.,0.,0.;

  //initialize observation
  struct OBSERVATION observation;

  Eigen::MatrixXf ob(6,1);
  Eigen::MatrixXf cov(6,6);
  ob << 0., 0., 0., 0., 0., 0.;
  cov << 1., 0., 0., 0., 0., 0.,
         0., 1., 0., 0., 0., 0.,
         0., 0., 1., 0., 0., 0.,
         0., 0., 0., 1., 0., 0.,
         0., 0., 0., 0., 1., 0.,
         0., 0., 0., 0., 0., 1.;

  observation.observation = ob;
  observation.covariance =cov;

  ekf::observation = observation;

  struct OBSERVATION init_observation_imu;
  Eigen::MatrixXf init_ob_imu(10,1);
  Eigen::MatrixXf init_cov_imu(10,10);

  init_ob_imu << 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.;
  init_cov_imu << 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
                  0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
                  0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
                  0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
                  0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
                  0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
                  0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
                  0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
                  0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
                  0., 0., 0., 0., 0., 0., 0., 0., 0., 0.;

  init_observation_imu.observation = init_ob_imu;
  init_observation_imu.covariance = init_cov_imu;

  ekf::observation_imu_init = init_observation_imu;

  struct OBSERVATION init_observation_gnss;
  Eigen::MatrixXf init_ob_gnss(12,1);
  Eigen::MatrixXf init_cov_gnss(12,12);

  init_ob_gnss << 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.;
  init_cov_gnss << 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
                   0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
                   0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
                   0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
                   0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
                   0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
                   0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
                   0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
                   0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
                   0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
                   0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
                   0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.;

  init_observation_gnss.observation = init_ob_gnss;
  init_observation_gnss.covariance = init_cov_gnss;

  ekf::observation_gnss_init = init_observation_gnss;

  //initialize imu output
  Eigen::MatrixXf imu_output(6,1);
  imu_output <<0., 0., -9.798, 0., 0., 0.;
  ekf::imu_output = imu_output;

  //Eigen::MatrixXf bias(6,1);
  //bias << 0.2210001, 0.2210001, 0.2210001, 2.4575407927235e-5, 0.0001276324, 9.7502189843605e-5;

  //ekf::bias = bias;

  //initialize Time
  ekf::past_time = ros::Time::now();
  ekf::current_time = ros::Time::now();
  is_initialized = false;
  is_started = false;
}

void ekf::set_time(){
  ekf::past_time = ekf::current_time;
  ekf::current_time = ros::Time::now();
  //std::cout << ekf::past_time.sec << "." << ekf::past_time.nsec << std::endl;
}

void ekf::set_time(ros::Time t){

  ekf::past_time = ekf::current_time;
  ekf::current_time = t;
  //std::cout << ekf::past_time.sec << "." << ekf::past_time.nsec << std::endl;
}

float ekf::get_delta(){
  //ros::Time current_time = ekf::current_time;
  //ros::Time past_time = ekf::past_time;
  float delta = (current_time - past_time).toSec();

  //ROS_INFO_STREAM(delta);

  return delta;
}

void ekf::predict_state(){
  Eigen::MatrixXf jaccobian_f = ekf::get_jaccobian_f();

  ekf::state.state = ekf::f();

  ekf::state.covariance = (jaccobian_f)*(get_delta() * state.covariance_with_imu)*(jaccobian_f.transpose()) + state.covariance;

  ekf::normalize_quaternion();
}

void ekf::predict_state(float accel_x, float accel_y, float accel_z, float gyro_x, float gyro_y, float gyro_z, ros::Time t){

  ekf::imu_output << accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z;

  ekf::past_time = ekf::current_time;
  ekf::current_time = t;

  Eigen::MatrixXf jaccobian_f = ekf::get_jaccobian_f();
  ekf::state.state = ekf::f();
  ekf::state.covariance = (jaccobian_f)*(ekf::state.covariance_with_imu)*(jaccobian_f.transpose()) + ekf::state.covariance;

  ekf::normalize_quaternion();
}

void ekf::predict_state(ros::Time t){
  if(is_started == true){
    ekf::past_time = ekf::current_time;
    ekf::current_time = t;
  }
  else {
    ekf::past_time = current_time = t;
    //is_started = true;
  }

  Eigen::MatrixXf jaccobian_f = get_jaccobian_f();
  ekf::state.state = ekf::f();
  ekf::state.covariance = (jaccobian_f)*(get_delta() * state.covariance_with_imu)*(jaccobian_f.transpose()) + state.covariance;

  ekf::normalize_quaternion();
}

Eigen::MatrixXf ekf::f(){
  float dt = ekf::get_delta();
  Eigen::MatrixXf s = ekf::state.state;
  Eigen::VectorXf u = ekf::imu_output;

  Eigen::MatrixXf p(3,1); //robot_position
  Eigen::MatrixXf p_(3,1); //robot_velocity
  Eigen::MatrixXf att(4,1); //robot_attitude
  Eigen::MatrixXf accel_bias(3,1); //bias of accelermeter
  Eigen::MatrixXf gyro_bias(3,1); //bias of gyroscope
  Eigen::MatrixXf accel(3,1); //output of accelermeter
  Eigen::MatrixXf gyro(3,1); //output of gyroscope
  Eigen::MatrixXf g(3,1); //acceleration of gravity
  Eigen::MatrixXf G(4,3);

  Eigen::MatrixXf rotation_matrix(3,3); //rotation_matrix from robot_frame to world_frame
  p << s(0,0), s(1,0), s(2,0);
  p_ << s(3,0), s(4,0), s(5,0);
  att << s(6,0), s(7,0), s(8,0), s(9,0);

  accel << u[0], u[1], u[2];
  gyro << u[3], u[4], u[5];
  g << 0, 0, -9.798;

  G << -att(1,0), -att(2,0), -att(3,0),
       att(0,0), att(3,0), -att(2,0),
       -att(3,0), att(0,0), att(1,0),
       att(2,0), -att(1,0), att(0,0);

  G << -att(1,0), -att(2,0), -att(3,0),
       att(0,0), att(3,0), -att(2,0),
       -att(3,0), att(0,0), att(1,0),
       att(2,0), -att(1,0), att(0,0);




  rotation_matrix << 1-2*(pow(att(2,0),2)+pow(att(3,0),2)), 2*(att(1,0)*att(2,0)+att(0,0)*att(3,0)), 2*(att(1,0)*att(3,0)-att(0,0)*att(2,0)),
                     2*(att(1,0)*att(2,0)-att(0,0)*att(3,0)), 1-2*(pow(att(1,0),2)+pow(att(3,0),2)), 2*(att(2,0)*att(3,0) + att(0,0)*att(1,0)),
                     2*(att(1,0)*att(3,0)+att(0,0)*att(2,0)), 2*(att(2,0)*att(3,0)-att(0,0)*att(1,0)), 1-2*(pow(att(1,0),2)+pow(att(2,0),2));

  Eigen::MatrixXf accel_inW = rotation_matrix * (accel) - g;
  Eigen::MatrixXf gyro_inW =  (gyro);

  Eigen::MatrixXf next_p = (p) + (dt * p_) + ((1.0/2.0) * pow(dt, 2) * (accel_inW));
  Eigen::MatrixXf next_p_ = (p_) + (dt) * (accel_inW);
  Eigen::MatrixXf next_att = att + (dt) * G * gyro_inW;

  s(0,0) = next_p(0,0);
  s(1,0) = next_p(1,0);
  s(2,0) = next_p(2,0);
  s(3,0) = next_p_(0,0);
  s(4,0) = next_p_(1,0);
  s(5,0) = next_p_(2,0);
  s(6,0) = next_att(0,0);
  s(7,0) = next_att(1,0);
  s(8,0) = next_att(2,0);
  s(9,0) = next_att(3,0);

  return s;
}

Eigen::MatrixXf ekf::get_KalmanGain(){
  Eigen::MatrixXf P_ = ekf::state.covariance;
  Eigen::MatrixXf C = ekf::jaccobian_h;
  Eigen::MatrixXf R = ekf::observation.covariance;

  ROS_INFO_STREAM((C * P_ * C.transpose() + R).inverse());

  Eigen::MatrixXf KG = P_ * C.transpose() * (C * P_ * C.transpose() + R).inverse();

  //ROS_INFO_STREAM(KG);

  return KG;
}

Eigen::MatrixXf ekf::get_KalmanGain_init(int flag){
  Eigen::MatrixXf P_ = ekf::state.covariance;
  Eigen::MatrixXf C;
  Eigen::MatrixXf R;
  if(flag == 0){
    C = ekf::jaccobian_h_imu_init;
    R = ekf::observation_imu_init.covariance;
  }
  else if(flag == 1){
    C = ekf::jaccobian_h_gnss_init;
    R = ekf::observation_gnss_init.covariance;
  }

  Eigen::MatrixXf KG = P_ * C.transpose() * (C * P_ * C.transpose() + R).inverse();

  return KG;
}



void ekf::update_state(string frame_id){
  ROS_INFO_STREAM("oops");
  ekf::get_jaccobian_h(frame_id);

  Eigen::MatrixXf z = ekf::observation.observation;
  //ROS_INFO_STREAM("observation: " << z.rows() << ", " << z.cols());

  //ROS_INFO_STREAM("observation: " << z(0,0) << ", " << z(1,0) << ", " << z(2,0)<< ", " << z(3,0) << ", " << z(4,0)<< ", " << z(5,0));

  Eigen::MatrixXf _x =ekf::state.state;
  Eigen::MatrixXf P_ = ekf::state.covariance;

  Eigen::MatrixXf KG = ekf::get_KalmanGain();

  //ROS_INFO_STREAM("KG: " << KG.rows()<< ", " << KG.cols());
  //ROS_INFO_STREAM("jacc: " << jaccobian_h.rows() << ", " << jaccobian_h.cols());
  //ROS_INFO_STREAM("z: " << z.rows() << ", " << z.cols());
  Eigen::MatrixXf x = _x + KG * (z - ekf::jaccobian_h*_x);
  Eigen::MatrixXf P = P_ - KG * ekf::jaccobian_h * P_;

  ekf::state.state = x;
  ekf::state.covariance = P;

  ekf::normalize_quaternion();
}

void ekf::debug_msg(){
  Eigen::MatrixXf s = ekf::state.state;
  Eigen::MatrixXf v = ekf::state.covariance;
  std::cout << "position is " << s(0,0) << ", " << s(1,0) << ", " << s(2,0) << std::endl;
  std::cout << "velocity is " << s(3,0) << ", " << s(4,0) << ", " << s(5,0) << std::endl;
  std::cout << "attitude is " << s(6,0) << ", " << s(7,0) << ", " << s(8,0) << "," << s(9,0) << std::endl;
  //std::cout << "gps_vector is " << s(16,0) << ", " << s(17,0) << ", " << s(18,0) << std::endl;
  std::cout << "covariance of position is\n " << v(0, 0)<< std::endl;
  std::cout << "  "  << std::endl;
}

//get output of imu
void ekf::set_ImuVal(sensor_msgs::Imu imu){
  //Eigen::MatrixXf imu_output(6,1);
  imu_output << -imu.linear_acceleration.x, imu.linear_acceleration.y, -imu.linear_acceleration.z,
                -imu.angular_velocity.x, imu.angular_velocity.y, -imu.angular_velocity.z;

  //ekf::imu_output = imu_output;
}
void ekf::set_ImuInitVal(sensor_msgs::Imu imu){
  //Eigen::MatrixXf imu_output(6,1);
  observation_imu_init.observation << -imu.linear_acceleration.x, imu.linear_acceleration.y, -imu.linear_acceleration.z,
                                      -imu.angular_velocity.x, imu.angular_velocity.y, -imu.angular_velocity.z,
                                      9.798,
                                      0., 0., 0.;

  observation_imu_init.covariance << imu.linear_acceleration_covariance[0], 0., 0., 0., 0., 0., 0., 0., 0., 0.,
                                     0., imu.linear_acceleration_covariance[4], 0., 0., 0., 0., 0., 0., 0., 0.,
                                     0., 0., imu.linear_acceleration_covariance[8], 0., 0., 0., 0., 0., 0., 0.,
                                     0., 0., 0., imu.angular_velocity_covariance[0], 0., 0., 0., 0., 0., 0.,
                                     0., 0., 0., 0., imu.angular_velocity_covariance[4], 0., 0., 0., 0., 0.,
                                     0., 0., 0., 0., 0., imu.angular_velocity_covariance[8], 0., 0., 0., 0.,
                                     0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
                                     0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
                                     0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
                                     0., 0., 0., 0., 0., 0., 0., 0., 0., 0.;
  ekf::set_time(imu.header.stamp);
  //ekf::imu_output = imu_output;
}

void ekf::set_LGnssVal(const sensor_msgs::NavSatFixConstPtr &gnss){
  OBSERVATION observation;
  Eigen::MatrixXf gnss_output(3,1);
  Eigen::MatrixXf gnss_covariance(3,3);

  gnss_output << gnss->latitude, gnss->longitude, gnss->altitude;
  gnss_covariance << gnss->position_covariance[0], 0., 0.,
                     0., gnss->position_covariance[4], 0.,
                     0., 0., gnss->position_covariance[8];

  observation.observation = gnss_output;
  observation.covariance = gnss_covariance;
}

void ekf::set_LGnssVal(const sensor_msgs::NavSatFixConstPtr &gnss, const geometry_msgs::TwistWithCovarianceStampedConstPtr &gnss_velocity){
  OBSERVATION observation;
  Eigen::MatrixXf gnss_output(6,1);
  Eigen::MatrixXf gnss_covariance(6,6);

  Eigen::Vector3f xyz = ekf::transform_enu2xyz(gnss->latitude, gnss->longitude, gnss->altitude);
  ROS_INFO_STREAM("Read: " << xyz[0] << ", " << xyz[1] << ", " << xyz[2]);

  gnss_output << xyz[0], xyz[1], xyz[2], gnss_velocity->twist.twist.linear.x, gnss_velocity->twist.twist.linear.y, gnss_velocity->twist.twist.linear.z;
  gnss_covariance << gnss->position_covariance[0], 0., 0., 0., 0., 0.,
                     0., gnss->position_covariance[4], 0., 0., 0., 0.,
                     0., 0., gnss->position_covariance[8], 0., 0., 0.,
                     0., 0., 0., gnss_velocity->twist.covariance[0], 0., 0.,
                     0., 0., 0., 0., gnss_velocity->twist.covariance[7], 0.,
                     0., 0., 0., 0., 0., gnss_velocity->twist.covariance[14];

  ekf::observation.observation = gnss_output;
  ekf::observation.covariance = gnss_covariance;
}

void ekf::set_RGnssVal(const sensor_msgs::NavSatFixConstPtr &gnss){
  OBSERVATION observation;
  Eigen::MatrixXf gnss_output(3,1);
  Eigen::MatrixXf gnss_covariance(3,3);

  gnss_output << gnss->latitude, gnss->longitude, gnss->altitude;
  gnss_covariance << gnss->position_covariance[0], 0., 0.,
                     0., gnss->position_covariance[4], 0.,
                     0., 0., gnss->position_covariance[8];

  observation.observation = gnss_output;
  observation.covariance = gnss_covariance;
}

void ekf::set_RGnssVal(const sensor_msgs::NavSatFixConstPtr &gnss, const geometry_msgs::TwistWithCovarianceStampedConstPtr &gnss_velocity){
  OBSERVATION observation;
  Eigen::MatrixXf gnss_output(6,1);
  Eigen::MatrixXf gnss_covariance(6,6);

  Eigen::Vector3f xyz = ekf::transform_enu2xyz(gnss->latitude, gnss->longitude, gnss->altitude);
  ROS_INFO_STREAM("Read: " << xyz[0] << ", " << xyz[1] << ", " << xyz[2]);

  gnss_output << xyz[0], xyz[1], xyz[2], gnss_velocity->twist.twist.linear.x, gnss_velocity->twist.twist.linear.y, gnss_velocity->twist.twist.linear.z;
  gnss_covariance << gnss->position_covariance[0], 0., 0., 0., 0., 0.,
                     0., gnss->position_covariance[4], 0., 0., 0., 0.,
                     0., 0., gnss->position_covariance[8], 0., 0., 0.,
                     0., 0., 0., gnss_velocity->twist.covariance[0], 0., 0.,
                     0., 0., 0., 0., gnss_velocity->twist.covariance[7], 0.,
                     0., 0., 0., 0., 0., gnss_velocity->twist.covariance[14];

  ekf::observation.observation = gnss_output;
  ekf::observation.covariance = gnss_covariance;
}

void ekf::set_GnssVal(const nav_msgs::OdometryConstPtr &gnss, const sensor_msgs::NavSatFixConstPtr &gnss_, const geometry_msgs::TwistWithCovarianceStampedConstPtr &gnss_velocity){
  if(gnss_->status.status == 1){
	OBSERVATION observation;
    Eigen::MatrixXf gnss_output(6,1);
    Eigen::MatrixXf gnss_covariance(6,6);

    gnss_output << gnss->pose.pose.position.x, gnss->pose.pose.position.y, gnss->pose.pose.position.z, gnss_velocity->twist.twist.linear.x, gnss_velocity->twist.twist.linear.y, gnss_velocity->twist.twist.linear.z;
    gnss_covariance << gnss->pose.covariance[0], 0., 0., 0., 0., 0.,
                       0., gnss->pose.covariance[7], 0., 0., 0., 0.,
                       0., 0., gnss->pose.covariance[14], 0., 0., 0.,
                       0., 0., 0., gnss_velocity->twist.covariance[0], 0., 0.,
                       0., 0., 0., 0., gnss_velocity->twist.covariance[7], 0.,
                       0., 0., 0., 0., 0., gnss_velocity->twist.covariance[14];

    //gnss_output << gnss_velocity->twist.twist.linear.x, gnss_velocity->twist.twist.linear.y, gnss_velocity->twist.twist.linear.z;
    //gnss_covariance << gnss_velocity->twist.covariance[0], 0., 0.,
    //                  0., gnss_velocity->twist.covariance[7], 0.,
    //                   0., 0., gnss_velocity->twist.covariance[14];

    //gnss_output << gnss->pose.pose.position.x, gnss->pose.pose.position.y, gnss->pose.pose.position.z;
    //gnss_covariance << gnss->pose.covariance[0], 0., 0.,
    //                   0., gnss->pose.covariance[7], 0.,
    //                   0., 0., gnss->pose.covariance[14];



    ekf::observation.observation = gnss_output;
    ekf::observation.covariance = gnss_covariance;

    is_started = true;
  }
}

void ekf::normalize_quaternion(){
  float norm = sqrt(pow(ekf::state.state(6,0),2)+pow(ekf::state.state(7,0),2)+pow(ekf::state.state(8,0),2)+pow(ekf::state.state(9,0),2));
  ekf::state.state.block(6,0,4,1) /= (norm);
  ekf::state.covariance.block(6,0,4,10) /= norm;
  ekf::state.covariance.block(0,6,10,4) /= norm;

  ekf::state.covariance_with_imu.block(0,0,10,10) = ekf::state.covariance;
}

nav_msgs::Odometry ekf::get_odometry(){
  nav_msgs::Odometry odometry;
  odometry.pose.pose.position.x = ekf::state.state(0,0);
  odometry.pose.pose.position.y = ekf::state.state(1,0);
  odometry.pose.pose.position.z = ekf::state.state(2,0);
  odometry.pose.pose.orientation.x = ekf::state.state(6,0);
  odometry.pose.pose.orientation.y = ekf::state.state(7,0);
  odometry.pose.pose.orientation.z = ekf::state.state(8,0);
  odometry.pose.pose.orientation.x = ekf::state.state(9,0);
}

Eigen::MatrixXf ekf::get_state(){
  return ekf::state.state;
}

ros::Time ekf::get_time(){
  return ekf::current_time;
}

void ekf::check_init(){
  if(is_initialized==true){
    ROS_INFO_STREAM("State has already been initialized!!");
    return;
  }
  int j = 0;
  for(int i=0;i<10;i++){
    if(state.covariance(i,i)<=threshold[i]){
      j += 1;
    }
  }
  if(j==10){
    ROS_INFO_STREAM("State has just been initialized!!");
    Eigen::MatrixXf s = state.state;
    ROS_INFO_STREAM("state: " << s(0,0) << ", " << s(1,0) << ", " << s(2,0)<< ", " << s(3,0) << ", " << s(4,0)<< ", " << s(5,0) << ", " << s(6,0)<< ", " << s(7,0) << ", " << s(8,0)<< ", " << s(9,0) << ", " << s(10,0)<< ", " << s(11,0) << ", " << s(12,0)<< ", " << s(13,0)<< ", " << s(14,0) << ", " << s(15,0));
    is_initialized = true;

  }
  else {
    ROS_INFO_STREAM("Now, state is being initialized!!");
    Eigen::MatrixXf s = state.state;
    ROS_INFO_STREAM("state: " << s(0,0) << ", " << s(1,0) << ", " << s(2,0)<< ", " << s(3,0) << ", " << s(4,0)<< ", " << s(5,0) << ", " << s(6,0)<< ", " << s(7,0) << ", " << s(8,0)<< ", " << s(9,0) << ", " << s(10,0)<< ", " << s(11,0) << ", " << s(12,0)<< ", " << s(13,0)<< ", " << s(14,0) << ", " << s(15,0));

  }
  return;
}
