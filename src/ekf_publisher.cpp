#include "ekf_publisher.hpp"
using namespace std;


ekf_publisher::ekf_publisher()
{
   is_initialized = true;
   imu_topic = "/imu";
   lgnss_topic = "/left_gnss/fix";
   rgnss_topic = "/right_gnss/fix";
   lgnss_velocity_topic = "/left_gnss/fix_vel";
   rgnss_velocity_topic = "/right_gnss/fix_vel";
   lgnss_relpos_topic = "/left_gnss/relpos";
   rgnss_relpos_topic = "/right_gnss/relpos";
   ekf_publisher::image_topic = "/image";

   node = new ros::NodeHandle;
   ekf_ = new ekf();

   ekf_publisher::odom_pub = node->advertise<nav_msgs::Odometry>("robot_odom", 100);
   ekf_publisher::init_pub = node->advertise<std_msgs::Bool>("is_initialized", 100);

   ekf_publisher::imu_sub = node->subscribe(imu_topic, 1000, &ekf_publisher::imu_callback, this);
   //ekf_publisher::imu_sub = node->subscribe(imu_topic, 1000, &ekf_publisher::test_callback, this);
   //ekf_publisher::init_imu_sub = node->subscribe(imu_topic, 1000, &ekf_publisher::initialize_callback, this);

   ////ekf_publisher::image_sub = node->subscribe(ekf_publisher::image_topic, 1000, &ekf_publisher::image_callback, this);

   //message_filters::Subscriber<sensor_msgs::NavSatFix> lgnss_sub(*node, ekf_publisher::lgnss_topic, 10);
   //message_filters::Subscriber<geometry_msgs::TwistWithCovarianceStamped> lgnss_velocity_sub(*node, ekf_publisher::lgnss_velocity_topic, 10);
   //message_filters::Subscriber<sensor_msgs::NavSatFix> rgnss_sub(*node, ekf_publisher::rgnss_topic, 10);
   //message_filters::Subscriber<geometry_msgs::TwistWithCovarianceStamped> rgnss_velocity_sub(*node, ekf_publisher::rgnss_velocity_topic, 10);

   lgnss_sub.subscribe(*node, lgnss_topic, 10);
   lgnss_velocity_sub.subscribe(*node, lgnss_velocity_topic, 10);
   lgnss_relpos_sub.subscribe(*node, lgnss_relpos_topic, 10);
   rgnss_sub.subscribe(*node, rgnss_topic, 10);
   rgnss_velocity_sub.subscribe(*node, rgnss_velocity_topic, 10);
   rgnss_relpos_sub.subscribe(*node, rgnss_relpos_topic, 10);

   //message_filters::TimeSynchronizer<sensor_msgs::NavSatFix, geometry_msgs::TwistWithCovarianceStamped> lsync(lgnss_sub, lgnss_velocity_sub, 10);
   //message_filters::TimeSynchronizer<sensor_msgs::NavSatFix, geometry_msgs::TwistWithCovarianceStamped> rsync(rgnss_sub, rgnss_velocity_sub, 10);

   //lsync = new message_filters::TimeSynchronizer<sensor_msgs::NavSatFix, geometry_msgs::TwistWithCovarianceStamped>(lgnss_sub, lgnss_velocity_sub, 10);
   //rsync = new message_filters::TimeSynchronizer<sensor_msgs::NavSatFix, geometry_msgs::TwistWithCovarianceStamped>(rgnss_sub, rgnss_velocity_sub, 10);
   //lsync->registerCallback(boost::bind(&ekf_publisher::lgnss_callback, this, _1, _2));
   //rsync->registerCallback(boost::bind(&ekf_publisher::rgnss_callback, this, _1, _2));

   //In your constructor initialize and register:
   //lsync.reset(new Sync(GnssSyncPolicy(10), lgnss_sub,lgnss_velocity_sub));
   //lsync->registerCallback(boost::bind(&ekf_publisher::lgnss_callback, this, _1, _2));

   //rsync.reset(new Sync(GnssSyncPolicy(10), rgnss_sub,rgnss_velocity_sub));
   //rsync->registerCallback(boost::bind(&ekf_publisher::rgnss_callback, this, _1, _2));

   lsync.reset(new Sync(GnssSyncPolicy(10), lgnss_relpos_sub, lgnss_sub, lgnss_velocity_sub));
   lsync->registerCallback(boost::bind(&ekf_publisher::gnss_callback, this, _1, _2, _3));

   rsync.reset(new Sync(GnssSyncPolicy(10), rgnss_relpos_sub, rgnss_sub, rgnss_velocity_sub));
   rsync->registerCallback(boost::bind(&ekf_publisher::gnss_callback, this, _1, _2, _3));

   //init_sync.reset(new InitSync(InitSyncPolicy(10), lgnss_relpos_sub,rgnss_relpos_sub));
   //init_sync->registerCallback(boost::bind(&ekf_publisher::initialize_callback, this, _1, _2));
}

void ekf_publisher::loop(){
  ros::spin();
}

void ekf_publisher::test_callback(sensor_msgs::Imu imu){
  //ROS_INFO_STREAM("TEST !!");
}

void ekf_publisher::initialize_callback(sensor_msgs::Imu imu){
  if(is_initialized){
    return;
  }

  //ROS_INFO_STREAM("1");
  //ROS_INFO_STREAM("Now, gnss_localizer is being initialized!!");

  ekf_publisher::time_lock[0] = imu.header.stamp.sec + imu.header.stamp.nsec * 1e-9;
  ros::Duration(1e-5).sleep();

  while(!ekf_publisher::imu_lock()){
    continue;
  }
  is_initialized = ekf_->is_initialized;

  std_msgs::Bool init_bool;
  init_bool.data = is_initialized;
  ekf_publisher::init_pub.publish(init_bool);
  ros::spinOnce();

  ekf_publisher::imu_unlock();
  ////ROS_INFO_STREAM("1_");
}

void ekf_publisher::initialize_callback(const nav_msgs::OdometryConstPtr &lgnss, const nav_msgs::OdometryConstPtr &rgnss){
  if(is_initialized){
  //if(true){
    return;
  }

  ////ROS_INFO_STREAM("2");
  std::cout << "Now, gnss_localizer is being initialized!!" << std::endl;

  ekf_publisher::time_lock[2] = rgnss->header.stamp.sec + rgnss->header.stamp.nsec * 1e-9;
  //ros::Duration(1e-5).sleep();

  while(!(ekf_publisher::rgnss_lock() or ekf_publisher::rgnss_lock())){
    //ROS_INFO_STREAM("Wow!!");
    continue;
  }

  //ROS_INFO_STREAM("Wow!!!");
  //ekf_publisher::ekf_->initialize(lgnss, rgnss);
  //ROS_INFO_STREAM("Wow!!!!!");
  is_initialized = ekf_->is_initialized;

  std_msgs::Bool init_bool;
  init_bool.data = is_initialized;
  ekf_publisher::init_pub.publish(init_bool);
  ros::spinOnce();

  ekf_publisher::rgnss_unlock();
}

void ekf_publisher::imu_callback(sensor_msgs::Imu imu){
  ROS_INFO_STREAM("3");
  if(!is_initialized){
    ////ROS_INFO_STREAM("Now, gnss_localizer is being initialized!!");
    return;
  }
  ekf_publisher::time_lock[0] = imu.header.stamp.sec + imu.header.stamp.nsec * 1e-9;
  //ros::Duration(1).sleep();

  if(time_lock[0] == 0){
    ////ROS_INFO_STREAM("Now, gnss_localizer is being initialized!!");
    return;
  }

  while(!ekf_publisher::imu_lock()){
    continue;
  }

  ekf_publisher::ekf_->set_ImuVal(imu);
  ekf_publisher::ekf_->predict_state(imu.header.stamp);

  ekf_publisher::publish_data();

  ekf_publisher::check_state();

  ekf_publisher::imu_unlock();

}

void ekf_publisher::gnss_callback(const nav_msgs::OdometryConstPtr &gnss, const sensor_msgs::NavSatFixConstPtr &gnss_, const geometry_msgs::TwistWithCovarianceStampedConstPtr &gnss_velocity){
  ROS_INFO_STREAM("113");
  //if(gnss->header.frame_id=="/right_gnss") return;
  //if(!is_initialized){
  if(false){
    //ROS_INFO_STREAM("Now, gnss_localizer is being initialized!!");
    return;
  }
  ekf_publisher::time_lock[1] = gnss->header.stamp.sec + gnss->header.stamp.nsec * 1e-9;

    if(time_lock[1] == 0){
    ////ROS_INFO_STREAM("Now, gnss_localizer is being initialized!!");
    return;
  }


  //ros::Duration(1e-3).sleep();

  while(!ekf_publisher::lgnss_lock()){
    continue;
  }
  ekf_publisher::ekf_->set_GnssVal(gnss, gnss_, gnss_velocity);
  ekf_publisher::ekf_->predict_state(gnss->header.stamp);
  ekf_publisher::ekf_->update_state(gnss->header.frame_id);

  ekf_publisher::check_state();

  ekf_publisher::publish_data();

  ekf_publisher::lgnss_unlock();
}



void ekf_publisher::image_callback(sensor_msgs::Image image){
  ROS_INFO_STREAM("oops_image");

  if(!is_initialized){
    return;
  }
  ekf_publisher::time_lock[3] = image.header.stamp.sec + image.header.stamp.nsec * 1e-9;

  if(time_lock[3] == 0){
    ////ROS_INFO_STREAM("Now, gnss_localizer is being initialized!!");
    return;
  }


  //ros::Duration(1e-3).sleep();

  while(!ekf_publisher::image_lock()){
    continue;
  }

  ekf_publisher::ekf_->predict_state(image.header.stamp);


  ekf_publisher::publish_data();
  ekf_publisher::check_state();

  ekf_publisher::image_unlock();


  /////
  ekf_publisher::time_lock[0] = imu.header.stamp.sec + imu.header.stamp.nsec * 1e-9;
  //ros::Duration(1e-3).sleep();



  while(!ekf_publisher::imu_lock()){
    continue;
  }

  ekf_publisher::ekf_->set_ImuVal(imu);
  ekf_publisher::ekf_->predict_state(imu.header.stamp);

  ekf_publisher::publish_data();

  ekf_publisher::check_state();

  ekf_publisher::imu_unlock();
}

bool ekf_publisher::imu_lock(){
  std::vector<double> data = ekf_publisher::time_lock;
  std::vector<double>::iterator iter = std::min_element(data.begin(), data.end());
  int index = std::distance(data.begin(), iter);
  if(index == 0){
    return true;
  }
  else{
    return false;
  }
}

bool ekf_publisher::lgnss_lock(){
  std::vector<double> data = ekf_publisher::time_lock;
  std::vector<double>::iterator iter = std::min_element(data.begin(), data.end());
  int index = std::distance(data.begin(), iter);
  if(index == 1){
    return true;
  }
  else{
    return false;
  }
}

bool ekf_publisher::rgnss_lock(){
  std::vector<double> data = ekf_publisher::time_lock;
  std::vector<double>::iterator iter = std::min_element(data.begin(), data.end());
  int index = std::distance(data.begin(), iter);
  if(index == 2){
    return true;
  }
  else{
    return false;
  }
}

bool ekf_publisher::image_lock(){
  std::vector<double> data = ekf_publisher::time_lock;
  std::vector<double>::iterator iter = std::min_element(data.begin(), data.end());
  int index = std::distance(data.begin(), iter);
  if(index == 3){
    return true;
  }
  else{
    return false;
  }
}

void ekf_publisher::imu_unlock(){
  ekf_publisher::time_lock[0] = 1e10;
}

void ekf_publisher::lgnss_unlock(){
  ekf_publisher::time_lock[1] = 1e10;
}

void ekf_publisher::rgnss_unlock(){
  ekf_publisher::time_lock[2] = 1e10;
}

void ekf_publisher::image_unlock(){
  ekf_publisher::time_lock[3] = 1e10;
}

void ekf_publisher::publish_data(){
  Eigen::MatrixXf state = ekf_publisher::ekf_->get_state();
  ros::Time t = ekf_publisher::ekf_->get_time();
  ekf_publisher::publish_odom(state, t);
}

void ekf_publisher::publish_tf(Eigen::MatrixXf state, ros::Time t){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(state(0,0), state(1,0), state(2,0)));
  tf::Quaternion q(state(7,0),state(8,0),state(9,0),state(6,0));
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, t, "world", "aadadada"));
}

void ekf_publisher::publish_odom(Eigen::MatrixXf state, ros::Time t){
  nav_msgs::Odometry odom;
  odom.header.stamp = t;
  odom.header.frame_id = "odom";

  //set the position
  odom.pose.pose.position.x = state(0,0);
  odom.pose.pose.position.y = state(1,0);
  odom.pose.pose.position.z = state(2,0);

  odom.pose.pose.orientation.x = state(7,0);
  odom.pose.pose.orientation.y = state(8,0);
  odom.pose.pose.orientation.z = state(9,0);
  odom.pose.pose.orientation.w = state(6,0);
  //set the velocity
  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = state(3,0);
  odom.twist.twist.linear.y = state(4,0);
  odom.twist.twist.linear.z = state(5,0);

  //odom.twist.twist.angular.z = vth;
  //publish the message
  odom_pub.publish(odom);
}

void ekf_publisher::print_timer(){
  //ROS_INFO_STREAM(time_lock[0]);
  //ROS_INFO_STREAM(time_lock[1]);
  //ROS_INFO_STREAM(time_lock[2]);
  //ROS_INFO_STREAM(time_lock[3]);
  return;
}

void ekf_publisher::check_state(){
  Eigen::MatrixXf s = ekf_->get_state();
  ROS_INFO_STREAM("state: " << s(0,0) << ", " << s(1,0) << ", " << s(2,0)<< ", " << s(3,0) << ", " << s(4,0)<< ", " << s(5,0) << ", " << s(6,0)<< ", " << s(7,0) << ", " << s(8,0)<< ", " << s(9,0));
}
