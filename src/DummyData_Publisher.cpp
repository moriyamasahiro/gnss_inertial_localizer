#include "ekf.hpp"

#include <boost/thread.hpp>
#include <random>

#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"

#include "std_msgs/Bool.h"

#include <Eigen/Core>
#include <Eigen/Geometry>


using namespace std;
using namespace boost;

class DummyData_Publisher
{
private:
  ros::NodeHandle* nh;
  ros::Time t, t_begin;
  ekf* dummy_ekf;
  float x, y, z;
  float vx, vy, vz;
  float ax, ay, az;
  float wx, wy, wz;
  float theta;
  float w;
  float r;
  vector<float> bias{0.,0.,0.,0.,0.,0.};

  bool is_initialized;


public:
  DummyData_Publisher();
  void loop();
  void publish_imu();
  void publish_lgnss();
  void publish_rgnss();
  void add_noise();
  void update_inner_state();
  void init();
  void initCallback(std_msgs::Bool msg);

  template <typename T, typename U> T add_noise(T true_value, U covariance);
};

DummyData_Publisher::DummyData_Publisher(){
  nh = new ros::NodeHandle;
  dummy_ekf = new ekf();
  t = t_begin = ros::Time::now();

  w = M_PI /60.;
  r = 10.;

  theta = 0;


  is_initialized = true;

  x = r * cos(0);
  y = r * sin(0);
  z = 0.;

  vx = 0.;
  vy = 0.;
  vz = 0.;

  ax = 0.;
  ay = 0.;
  az = 0.;

  wx = 0.;
  wy = 0.;
  wz = 0.;
}

void DummyData_Publisher::loop(){
  thread thread_init(&DummyData_Publisher::init, this);
  thread thread_update(&DummyData_Publisher::update_inner_state, this);
  thread thread_imu(&DummyData_Publisher::publish_imu, this);
  thread thread_lgnss(&DummyData_Publisher::publish_lgnss, this);
  thread thread_rgnss(&DummyData_Publisher::publish_rgnss, this);

  thread_init.join();
  thread_update.join();
  thread_imu.join();
  thread_lgnss.join();
  thread_rgnss.join();
}

void DummyData_Publisher::publish_imu(){
  ros::Publisher imu_pub = nh->advertise<sensor_msgs::Imu>("imu", 1000);
  ros::Rate loop_rate(40);


  while (ros::ok())
  {
    sensor_msgs::Imu imu;

    Eigen::Vector3d a_value;
    Eigen::Vector3d a_covariance;
    Eigen::Vector3d w_value;
    Eigen::Vector3d w_covariance;

    a_value << ax + bias[0], ay + bias[1], az + 9.798 + bias[2];
    a_covariance << 0.01 + bias[3], 0.01 + bias[4], 0.01 + bias[5];
    w_value << wx, wy, wz;
    w_covariance << 0.0001, 0.0001, 0.0001;

    a_value = DummyData_Publisher::add_noise(a_value, a_covariance);
    w_value = DummyData_Publisher::add_noise(w_value, w_covariance);

    imu.header.stamp = ros::Time::now();
    imu.linear_acceleration.x = a_value[0];
    imu.linear_acceleration.y = a_value[1];
    imu.linear_acceleration.z = a_value[2];
    imu.linear_acceleration_covariance[0] = a_covariance[0];
    imu.linear_acceleration_covariance[4] = a_covariance[1];
    imu.linear_acceleration_covariance[8] = a_covariance[2];
    imu.angular_velocity.x = w_value[0];
    imu.angular_velocity.y = w_value[1];
    imu.angular_velocity.z = w_value[2];
    imu.angular_velocity_covariance[0] = w_covariance[0];
    imu.angular_velocity_covariance[4] = w_covariance[1];
    imu.angular_velocity_covariance[8] = w_covariance[2];

    imu_pub.publish(imu);
    ros::spinOnce();
    ROS_INFO_STREAM("Publish IMU data!!");
    loop_rate.sleep();
  }
}

void DummyData_Publisher::publish_lgnss(){
  ros::Publisher lgnss_fix_pub = nh->advertise<sensor_msgs::NavSatFix>("left_gnss/fix", 1000);
  ros::Publisher lgnss_vel_pub = nh->advertise<geometry_msgs::TwistWithCovarianceStamped>("left_gnss/fix_vel", 1000);
  ros::Rate loop_rate(5);

  while (ros::ok())
  {
    Eigen::Vector3f anntena_pos;
    Eigen::Vector3d p_covariance;
    Eigen::Vector3d anntena_vel;
    Eigen::Vector3d v_covariance;

    anntena_pos << x - 0.23 * cos(theta), y - 0.23 * sin(theta), z + 0.05;
    p_covariance << 0.002, 0.002, 0.01;
    anntena_vel << vx + 0.23 * w * sin(theta), vy - 0.23 * w * cos(theta), z;
    v_covariance << 0.002, 0.002, 0.01;

    anntena_pos = DummyData_Publisher::add_noise(anntena_pos, p_covariance);
    anntena_vel = DummyData_Publisher::add_noise(anntena_vel, v_covariance);
    Eigen::Vector3d enu = dummy_ekf->transform_xyz2enu(anntena_pos[0], anntena_pos[1], anntena_pos[2]);

    sensor_msgs::NavSatFix lgnss_fix;
    geometry_msgs::TwistWithCovarianceStamped lgnss_vel;

    lgnss_fix.header.stamp = lgnss_vel.header.stamp = ros::Time::now();
    lgnss_fix.longitude = enu[1];
    lgnss_fix.latitude = enu[0];
    lgnss_fix.altitude = enu[2];
    lgnss_fix.position_covariance[0] = p_covariance[0];
    lgnss_fix.position_covariance[4] = p_covariance[1];
    lgnss_fix.position_covariance[8] = p_covariance[2];

    lgnss_vel.twist.twist.linear.x = anntena_vel[0];
    lgnss_vel.twist.twist.linear.y = anntena_vel[1];
    lgnss_vel.twist.twist.linear.z = anntena_vel[2];
    lgnss_vel.twist.covariance[0] = v_covariance[0];
    lgnss_vel.twist.covariance[7] = v_covariance[1];
    lgnss_vel.twist.covariance[14] = v_covariance[2];

    lgnss_fix_pub.publish(lgnss_fix);
    lgnss_vel_pub.publish(lgnss_vel);
    ros::spinOnce();
    ROS_INFO_STREAM("Publish LGNSS data!!");
    loop_rate.sleep();
  }
}

void DummyData_Publisher::publish_rgnss(){
  ros::Publisher rgnss_fix_pub = nh->advertise<sensor_msgs::NavSatFix>("right_gnss/fix", 1000);
  ros::Publisher rgnss_vel_pub = nh->advertise<geometry_msgs::TwistWithCovarianceStamped>("right_gnss/fix_vel", 1000);
  ros::Rate loop_rate(5);

  while (ros::ok())
  {
    Eigen::Vector3f anntena_pos;
    Eigen::Vector3d p_covariance;
    Eigen::Vector3d anntena_vel;
    Eigen::Vector3d v_covariance;

    anntena_pos << x + 0.23 * cos(theta), y + 0.23 * sin(theta), z + 0.05;
    p_covariance << 0.002, 0.002, 0.01;
    anntena_vel << vx - 0.23 * w * sin(theta), vy + 0.23 * w * cos(theta), z;
    v_covariance << 0.002, 0.002, 0.01;

    anntena_pos = DummyData_Publisher::add_noise(anntena_pos, p_covariance);
    anntena_vel = DummyData_Publisher::add_noise(anntena_vel, v_covariance);
    Eigen::Vector3d enu = dummy_ekf->transform_xyz2enu(anntena_pos[0], anntena_pos[1], anntena_pos[2]);

    sensor_msgs::NavSatFix rgnss_fix;
    geometry_msgs::TwistWithCovarianceStamped rgnss_vel;

    rgnss_fix.header.stamp = rgnss_vel.header.stamp = ros::Time::now();
    rgnss_fix.longitude = enu[1];
    rgnss_fix.latitude = enu[0];
    rgnss_fix.altitude = enu[2];
    rgnss_fix.position_covariance[0] = p_covariance[0];
    rgnss_fix.position_covariance[4] = p_covariance[1];
    rgnss_fix.position_covariance[8] = p_covariance[2];

    rgnss_vel.twist.twist.linear.x = anntena_vel[0];
    rgnss_vel.twist.twist.linear.y = anntena_vel[1];
    rgnss_vel.twist.twist.linear.z = anntena_vel[2];
    rgnss_vel.twist.covariance[0] = v_covariance[0];
    rgnss_vel.twist.covariance[7] = v_covariance[1];
    rgnss_vel.twist.covariance[14] = v_covariance[2];

    rgnss_fix_pub.publish(rgnss_fix);
    rgnss_vel_pub.publish(rgnss_vel);
    ros::spinOnce();
    ROS_INFO_STREAM("Publish RGNSS data!!");
    loop_rate.sleep();
  }
}

void DummyData_Publisher::update_inner_state(){
  ros::Rate loop_rate(1000);
  bool is_started = false;
  while (ros::ok()){

    if(false){
      if(is_initialized){
        t_begin = ros::Time::now();
        is_started = true;
      }
      break;
    }

    t = ros::Time::now();

    //theta = fmodf(w * (t.toSec()),M_PI);
    theta = w * (t-t_begin).toSec();

    x = r * cos(theta);
    y = r * sin(theta);
    z = 0.;

    vx = r * w * sin(theta);
    vy = r * w * cos(theta);
    vz = 0;

    ax = r * w * w;
    ay = 0;
    az = 0;

    wx = 0;
    wy = 0;
    wz = -w;

    loop_rate.sleep();
  }
}

void DummyData_Publisher::init(){
  ros::Subscriber sub = nh->subscribe("is_initialized", 1000, &DummyData_Publisher::initCallback, this);
  ros::spin();
}

void DummyData_Publisher::initCallback(std_msgs::Bool msg){
  is_initialized = msg.data;
  is_initialized = true;
  return;
}

template <typename T, typename U> T DummyData_Publisher::add_noise(T true_value, U covariance){
  if (true_value.size() != covariance.size()){
    cout << "the size of input is invalid!!" << endl;
    return true_value;
  }
  random_device seed;
	mt19937 engine(seed());            // メルセンヌ・ツイスター法
	// std::minstd_rand0 engine(seed());    // 線形合同法
	// std::ranlux24_base engine(seed());   // キャリー付き減算法

	double mean = 0.0;
  for(int i=0;i<true_value.size();i++){
    normal_distribution<> dist(mean, sqrt(covariance[i]));
    true_value[i] += dist(engine);
  }
  return true_value;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "dummy_publisher");

  DummyData_Publisher* ddp;
  ddp = new DummyData_Publisher();
  ddp->loop();

  return 0;
}
