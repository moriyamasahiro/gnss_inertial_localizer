#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <tf/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>



#define _USE_MATH_DEFINES

#include "ekf.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sstream>
#include <cmath>
#include <string>
#include <vector>
#include <iostream>
#include <boost/shared_ptr.hpp>

using namespace std;



class ekf_publisher
{
    private:
      ros::NodeHandle* node;
      ekf *ekf_;

      bool is_initialized;

      message_filters::Subscriber<sensor_msgs::NavSatFix> lgnss_sub;
      message_filters::Subscriber<geometry_msgs::TwistWithCovarianceStamped> lgnss_velocity_sub;
      message_filters::Subscriber<sensor_msgs::NavSatFix> rgnss_sub;
      message_filters::Subscriber<geometry_msgs::TwistWithCovarianceStamped> rgnss_velocity_sub;

      //message_filters::TimeSynchronizer<sensor_msgs::NavSatFix, geometry_msgs::TwistWithCovarianceStamped> *lsync;
      //message_filters::TimeSynchronizer<sensor_msgs::NavSatFix, geometry_msgs::TwistWithCovarianceStamped> *rsync;

      //tf::TransformBroadcaster* br;
      ros::Publisher odom_pub;
      ros::Publisher init_pub;

      typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::NavSatFix, geometry_msgs::TwistWithCovarianceStamped> GnssSyncPolicy;
      typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::NavSatFix, sensor_msgs::NavSatFix> InitSyncPolicy;
      typedef message_filters::Synchronizer<GnssSyncPolicy> Sync;
      typedef message_filters::Synchronizer<InitSyncPolicy> InitSync;

      boost::shared_ptr<Sync> lsync;
      boost::shared_ptr<Sync> rsync;

      boost::shared_ptr<InitSync> init_sync;

      string imu_topic;
      string lgnss_topic;
      string rgnss_topic;
      string lgnss_velocity_topic;
      string rgnss_velocity_topic;
	    string image_topic;

      std::vector<double> time_lock = {1e10, 1e10, 1e10, 1e10};

      ros::Subscriber imu_sub;
      ros::Subscriber init_imu_sub;
	    ros::Subscriber image_sub;

      //message_filters::Subscriber<sensor_msgs::NavSatFix> lgnss_sub(nh, lgnss_topic, 1);
      //message_filters::Subscriber<geometry_msgs::TwistWithCovarianceStamped> lgnss_velocity_sub(nh, lgnss_velocity_topic, 1);
      //message_filters::Subscriber<sensor_msgs::NavSatFix> rgnss_sub(nh, rgnss_topic, 1);
      //message_filters::Subscriber<geometry_msgs::TwistWithCovarianceStamped> rgnss_velocity_sub(nh, rgnss_velocity_topic, 1);

      //message_filters::TimeSynchronizer<sensor_msgs::NavSatFix, geometry_msgs::TwistWithCovarianceStamped> lsync(lgnss_sub, lgnss_velocity_sub, 10);
      //message_filters::TimeSynchronizer<sensor_msgs::NavSatFix, geometry_msgs::TwistWithCovarianceStamped> rsync(rgnss_sub, rgnss_velocity_sub, 10);

      sensor_msgs::Imu imu;
      sensor_msgs::Imu pre_imu;
      sensor_msgs::NavSatFix lgnss;
      sensor_msgs::NavSatFix rgnss;

    public:
      ekf_publisher();

      void loop();

      void initialize_callback(sensor_msgs::Imu imu);
      void initialize_callback(const sensor_msgs::NavSatFixConstPtr &lgnss, const sensor_msgs::NavSatFixConstPtr &rgnss);
      void test_callback(sensor_msgs::Imu imu);
      void imu_callback(sensor_msgs::Imu imu);
      void lgnss_callback(const sensor_msgs::NavSatFixConstPtr &lgnss);
	    void rgnss_callback(const sensor_msgs::NavSatFixConstPtr &rgnss);
	    void lgnss_callback(const sensor_msgs::NavSatFixConstPtr &lgnss, const geometry_msgs::TwistWithCovarianceStampedConstPtr &lgnss_velocity);
	    void rgnss_callback(const sensor_msgs::NavSatFixConstPtr &rgnss, const geometry_msgs::TwistWithCovarianceStampedConstPtr &rgnss_velocity);
	    void image_callback(sensor_msgs::Image image);

	    bool imu_lock();
	    bool lgnss_lock();
	    bool rgnss_lock();
	    bool image_lock();

	    void imu_unlock();
	    void lgnss_unlock();
	    void rgnss_unlock();
	    void image_unlock();
      void publish_data();
      void publish_tf(Eigen::MatrixXf state, ros::Time t);
      void publish_odom(Eigen::MatrixXf state, ros::Time t);
      void print_timer();
      void check_state();
};
