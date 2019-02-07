#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"

#define _USE_MATH_DEFINES

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sstream>
#include <cmath>
using namespace std;

struct OBSERVATION{
  Eigen::MatrixXf observation;
  Eigen::MatrixXf covariance;
};

struct STATE{
  Eigen::MatrixXf state;
  Eigen::MatrixXf covariance;
  Eigen::MatrixXf covariance_with_imu;
};

class ekf
{
    private:
      STATE state;
      OBSERVATION observation;
      OBSERVATION observation_imu_init;
      OBSERVATION observation_gnss_init;

      Eigen::MatrixXf imu_output;
      Eigen::MatrixXf LeftGnss_output;
      Eigen::MatrixXf LeftGnss_covariance;
      Eigen::MatrixXf RightGnss_output;
      Eigen::MatrixXf RightGnss_covariance;

      int flag;

      Eigen::MatrixXf jaccobian_f;
      Eigen::MatrixXf jaccobian_h;
      Eigen::MatrixXf jaccobian_h_imu_init;
      Eigen::MatrixXf jaccobian_h_gnss_init;
      ros::Time current_time;
      ros::Time past_time;

      double longitude0;
      double latitude0;
      float threshold[16] = {0.01, 0.01, 0.01, 0, 0, 0, 0.01, 0.01, 0.01, 0.01, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001};

      Eigen::MatrixXf bias;



    public:
      ekf();

      void initialize(sensor_msgs::Imu imu);
      void initialize(const sensor_msgs::NavSatFixConstPtr &lgnss, const sensor_msgs::NavSatFixConstPtr &rgnss);

      void predict_state();
      void predict_state(float accel_x, float accel_y, float accel_z, float gyro_x, float gyro_y, float gyro_z, ros::Time t);
      void predict_state(ros::Time t);
      Eigen::MatrixXf f();
      Eigen::MatrixXf get_jaccobian_f();
      void get_jaccobian_h_with_lgnss();
      void get_jaccobian_h_with_rgnss();
      void get_jaccobian_h();
      Eigen::MatrixXf get_jaccobian_h(string frame_id);
      void get_jaccobian_h_imu_init();
      void get_jaccobian_h_gnss_init();
      Eigen::MatrixXf get_KalmanGain();
      Eigen::MatrixXf get_KalmanGain_init(int flag);
      void update_state();
      void update_state_with_lgnss();
      void update_state_with_lgnss(double longitude, double latitude, double altitude, float x_cov, float y_cov, float z_cov);
      void update_state_with_rgnss();
      void update_state_with_rgnss(double longitude, double latitude, double altitude, float x_cov, float y_cov, float z_cov);
      void set_time();
      void set_time(ros::Time t);
      void debug_msg();
      float get_delta();
      void set_ImuVal(sensor_msgs::Imu imu);
      void set_ImuInitVal(sensor_msgs::Imu imu);
      void set_LGnssVal(const sensor_msgs::NavSatFixConstPtr &gnss);
      void set_LGnssVal(const sensor_msgs::NavSatFixConstPtr &gnss, const geometry_msgs::TwistWithCovarianceStampedConstPtr &gnss_velocity);
      void set_RGnssVal(const sensor_msgs::NavSatFixConstPtr &gnss);
      void set_RGnssVal(const sensor_msgs::NavSatFixConstPtr &gnss, const geometry_msgs::TwistWithCovarianceStampedConstPtr &gnss_velocity);
      Eigen::Vector3f transform_enu2xyz(double longitude, double latitude, double altitude);
      Eigen::Vector3d transform_xyz2enu(float x, float y, float z);
      double Deg2Rad(double degrees);
      double Rad2Deg(double radians);
      void normalize_quaternion();
      void check_init();
      nav_msgs::Odometry get_odometry();
      Eigen::MatrixXf get_state();
      ros::Time get_time();

      bool is_initialized;
      bool is_started;
};
