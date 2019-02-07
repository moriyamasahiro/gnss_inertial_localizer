#include "ekf.hpp"

#include <boost/thread.hpp>
#include <random>

#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"

#include "nav_msgs/Odometry.h"

#include "std_msgs/Bool.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv/cv.hpp>


using namespace std;
using namespace boost;

class Position_Drower
{
private:
  ros::NodeHandle* nh;
  ros::Time t, t_begin;
  ekf* dummy_ekf;
  Eigen::VectorXf left_anntena;
  Eigen::VectorXf right_anntena;
  Eigen::VectorXf robot_pos;
  Eigen::VectorXf left_vel;
  Eigen::VectorXf right_vel;
  Eigen::VectorXf robot_vel;
  Eigen::VectorXf center;
  cv::Mat image;
  cv::Mat init_image;

  int scale;


public:
  Position_Drower();
  void loop();
  void show();

  Eigen::VectorXf shift(Eigen::VectorXf);

  void subscribe();

  void LGnssCallback(const sensor_msgs::NavSatFixConstPtr &lgnss);
  void RGnssCallback(const sensor_msgs::NavSatFixConstPtr &rgnss);
  void OdomCallback(nav_msgs::Odometry odom);
  void LGnssVelCallback(geometry_msgs::TwistWithCovarianceStamped lgnss_vel);
  void RGnssVelCallback(geometry_msgs::TwistWithCovarianceStamped rgnss_vel);
};

Position_Drower::Position_Drower(){
  nh = new ros::NodeHandle;
  dummy_ekf = new ekf();

  Eigen::VectorXf zero(3);
  Eigen::VectorXf zero_2(2);
  Eigen::VectorXf vel(3);
  zero <<0., 0., 0.;
  zero_2 <<0., 0.;
  vel << 0., 0., 0.;
  left_anntena = zero;
  right_anntena = zero;
  robot_pos = zero;
  left_vel = vel;
  right_vel = vel;
  robot_vel = vel;
  center =zero_2;

  center << 2500, 2500;
  scale = 100;
}

void Position_Drower::loop(){
  thread thread_subscribe(&Position_Drower::subscribe, this);
  thread thread_show(&Position_Drower::show, this);


  thread_subscribe.join();
  thread_show.join();
}

void Position_Drower::show(){
  ros::Rate loop_rate(25);
  cv::namedWindow("Window", CV_WINDOW_NORMAL );
  while(ros::ok()){
    cv::Mat frame(cv::Size(5000, 5000), CV_8UC3, cv::Scalar(255,255,255));

    //cv::line(frame, cv::Point(int(left_anntena[0]), int(left_anntena[1])), cv::Point(int(right_anntena[0]), int(right_anntena[1])), cv::Scalar(0,0,255), 2, CV_AA);

    cv::circle(frame, cv::Point(int(left_anntena[0]), int(left_anntena[1])), 25, cv::Scalar(255,0,0), -1);
    cv::circle(frame, cv::Point(int(right_anntena[0]), int(right_anntena[1])), 25, cv::Scalar(0,255,0), -1);
    //cv::circle(frame, cv::Point(int(robot_pos[0]), int(robot_pos[1])), 100, cv::Scalar(0,0,255), -1);

    //cv::arrowedLine(frame, cv::Point(int(left_anntena[0]), int(left_anntena[1])), cv::Point(int(left_anntena[0]+left_vel[0]), int(left_anntena[1]+left_vel[1])), cv::Scalar(255,100,100), 2);
    //cv::arrowedLine(frame, cv::Point(int(right_anntena[0]), int(right_anntena[1])), cv::Point(int(right_anntena[0]+right_vel[0]), int(right_anntena[1]+right_vel[1])), cv::Scalar(0,255,0), 2);
    //cv::arrowedLine(frame, cv::Point(int(robot_pos[0]), int(robot_pos[1])), cv::Point(int(robot_pos[0]), int(robot_pos[1])), cv::Scalar(0,0,255),2);

    cv::imshow("Window", frame);//画像を表示．
    cv::waitKey(20);
    loop_rate.sleep();
  }
}

Eigen::VectorXf Position_Drower::shift(Eigen::VectorXf input){
  Eigen::VectorXf output(3);

  output << scale * input[0] + center[0], - scale * input[1] + center[1], input[0];

  return output;
}


void Position_Drower::subscribe(){
  ros::Subscriber sub_lgnss = nh->subscribe("left_gnss/fix", 1000, &Position_Drower::LGnssCallback, this);
  ros::Subscriber sub_rgnss = nh->subscribe("right_gnss/fix", 1000, &Position_Drower::RGnssCallback, this);
  ros::Subscriber sub_odom = nh->subscribe("robot_odom", 1000, &Position_Drower::OdomCallback, this);

  ros::Subscriber sub_lgnss_vel = nh->subscribe("left_gnss/fix_vel", 1000, &Position_Drower::LGnssVelCallback, this);
  ros::Subscriber sub_rgnss_vel = nh->subscribe("right_gnss/fix_vel", 1000, &Position_Drower::LGnssVelCallback, this);

  ros::spin();
}

void Position_Drower::LGnssCallback(const sensor_msgs::NavSatFixConstPtr &lgnss){
  ROS_INFO_STREAM("1");
  Position_Drower::left_anntena = Position_Drower::shift(dummy_ekf->transform_enu2xyz(lgnss->longitude, lgnss->latitude, lgnss->altitude));
  ROS_INFO_STREAM("state: " << left_anntena[0] << ", " << left_anntena[1] << ", " << left_anntena[2] << ", " <<  sqrt(pow(left_anntena[0]-center[0],2)+pow(left_anntena[1]-center[1],2)+pow(left_anntena[2],2)));
  return;
}

void Position_Drower::RGnssCallback(const sensor_msgs::NavSatFixConstPtr &rgnss){
  ROS_INFO_STREAM("2");
  Position_Drower::right_anntena = Position_Drower::shift(dummy_ekf->transform_enu2xyz(rgnss->longitude, rgnss->latitude, rgnss->altitude));
  ROS_INFO_STREAM("state: " << right_anntena[0] << ", " << right_anntena[1] << ", " << right_anntena[2] << ", " << sqrt(pow(right_anntena[0]-center[0],2)+pow(right_anntena[1]-center[1],2)+pow(right_anntena[2],2)));
  return;
}

void Position_Drower::OdomCallback(nav_msgs::Odometry odom){
  ROS_INFO_STREAM("3");
  Eigen::VectorXf odometry(3);
  odometry << float(odom.pose.pose.position.x), float(odom.pose.pose.position.y), float(odom.pose.pose.position.z);
  Position_Drower::robot_pos = Position_Drower::shift(odometry);
  ROS_INFO_STREAM("state: " << robot_pos[0] << ", " << robot_pos[1] << ", " << robot_pos[2] << ", " << sqrt(pow(robot_pos[0],2)+pow(robot_pos[1],2)+pow(robot_pos[2],2)));
  return;
}

void Position_Drower::LGnssVelCallback(geometry_msgs::TwistWithCovarianceStamped lgnss_vel){
  ROS_INFO_STREAM("4");
  Position_Drower::left_vel <<lgnss_vel.twist.twist.linear.x, lgnss_vel.twist.twist.linear.y, lgnss_vel.twist.twist.linear.z;
  return;
}

void Position_Drower::RGnssVelCallback(geometry_msgs::TwistWithCovarianceStamped rgnss_vel){
  ROS_INFO_STREAM("5");
  Position_Drower::right_vel <<rgnss_vel.twist.twist.linear.x, rgnss_vel.twist.twist.linear.y, rgnss_vel.twist.twist.linear.z;
  return;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "position_drower");

  Position_Drower* pd;
  pd = new Position_Drower();
  pd->loop();

  return 0;
}
