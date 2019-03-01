// %Tag(FULLTEXT)%
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <lcm/lcm-cpp.hpp>

#include <lcm_types.hpp>

#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <math.h>
Eigen::Vector3f qtorpy(Eigen::Quaternionf q);

// Quaternion to roll, pitch, yaw
Eigen::Vector3f qtorpy(Eigen::Quaternionf q){
  Eigen::Matrix3f qrot;
  float sy, roll, pitch, yaw;
  Eigen::Vector3f rpy;
  
  qrot = q.normalized().toRotationMatrix();
  sy = sqrt(qrot(0,0)*qrot(0,0) + qrot(1,0)*qrot(1,0));
  if (sy >= 0.000001){ 		// sy < 1e-6 is singular
	rpy[0] = atan2(qrot(2,1) ,qrot(2,2));
	rpy[1] = atan2(-qrot(2,0),sy);
	rpy[2] = atan2(qrot(1,0) ,qrot(0,0));
  } else {
	rpy[0] = atan2(-qrot(1,2),qrot(1,1));
	rpy[1] = atan2(-qrot(2,0),sy);
	rpy[2] = 0;
  }
return rpy;
}

class App{
	public:
	  App(ros::NodeHandle node_);
	  ~App();

	private:
	  lcm::LCM lcm_;
	  ros::NodeHandle node_;

      SD::raw_gnss_t lcm_raw_gnss_msg_;
	  int raw_gnss_counter_;

	  SD::state_t lcm_state_msg_;
	  SD::raw_sensors_t lcm_raw_sensor_msg_;
	  int state_msg_counter_;
	  
	  int imu_flag, state_flag, odom_flag;
	  
	  ros::Subscriber  imu_sub_;
	  void imu_cb(const sensor_msgs::Imu::ConstPtr &msg);
	  
	  ros::Subscriber  gps_fix_sub_;
	  void gps_fix_cb(const sensor_msgs::NavSatFix::ConstPtr &msg);

	  ros::Subscriber  gps_fix_velocity_sub_;
	  void gps_fix_velocity_cb(const geometry_msgs::Vector3Stamped::ConstPtr &msg);

	  ros::Subscriber  gps_filtered_sub_;
	  void gps_filtered_cb(const sensor_msgs::NavSatFix::ConstPtr &msg);

	  ros::Subscriber  odom_gps_sub_;
	  void odom_gps_cb(const nav_msgs::Odometry::ConstPtr &msg);

};

App::App(ros::NodeHandle node_):
  node_(node_){

ROS_INFO("Initializing Translator");

  if(!lcm_.good()){
	std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }

 
// %Tag(SUBSCRIBER)%
  imu_sub_ 		= node_.subscribe(std::string("imu/data"), 	   1000, &App::imu_cb, this);
  gps_fix_sub_ 		= node_.subscribe(std::string("gps/fix"), 	   1000, &App::gps_fix_cb, this);
  gps_fix_velocity_sub_ = node_.subscribe(std::string("gps/fix_velocity"), 1000, &App::gps_fix_velocity_cb, this);
  gps_filtered_sub_	= node_.subscribe(std::string("gps/filtered"), 	   1000, &App::gps_filtered_cb, this);
  odom_gps_sub_		= node_.subscribe(std::string("odometry/filtered"),		   1000, &App::odom_gps_cb, this);
// %EndTag(SUBSCRIBER)%

raw_gnss_counter_ = 0;
state_msg_counter_ = 0;
imu_flag=0; state_flag=0; odom_flag=0;
};

App::~App() {
};

// %Tag(CALLBACK)%
// Callback for ROS raw gps data, merges gps/fix with gps/fix_velocity
// ROS Topic	: gps/fix
// ROS Type		: sensor_msgs/NavSatFix
// LCM Channel	: SD_RAW_GNSS
// LCM Type 	: SD.raw_gnss_t
void App::gps_fix_cb(const sensor_msgs::NavSatFix::ConstPtr &msg){
  lcm_raw_gnss_msg_.latitude = msg->latitude;
  lcm_raw_gnss_msg_.longitude = msg->longitude;
  lcm_raw_gnss_msg_.height = msg->altitude;

  lcm_raw_gnss_msg_.latitude_standard_deviation = -1;
  lcm_raw_gnss_msg_.longitude_standard_deviation = -1;
  lcm_raw_gnss_msg_.height_standard_deviation = -1;

  lcm_raw_gnss_msg_.tilt = -1;
  lcm_raw_gnss_msg_.heading = -1;

  lcm_raw_gnss_msg_.tilt_standard_deviation = -1;
  lcm_raw_gnss_msg_.heading_standard_devation = -1; //TODO: typo in definition 'devation'

  lcm_raw_gnss_msg_.status.doppler_velocity_valid = 1;
  lcm_raw_gnss_msg_.status.time_valid = 1;
  lcm_raw_gnss_msg_.status.external_gnss = 1;
  lcm_raw_gnss_msg_.status.tilt_valid = 1;
  lcm_raw_gnss_msg_.status.heading_valid = 1;
  lcm_raw_gnss_msg_.status.floating_ambiguity_heading = 1;

  lcm_raw_gnss_msg_.timestamp.seconds = msg->header.stamp.sec;
  lcm_raw_gnss_msg_.timestamp.microseconds = ((msg->header.stamp.nsec*0.001) + 0.5); // nano-seconds to micro-seconds

  lcm_raw_gnss_msg_.seconds = msg->header.stamp.sec;
  lcm_raw_gnss_msg_.microseconds = ((msg->header.stamp.nsec*0.001) + 0.5); // nano-seconds to micro-seconds


  raw_gnss_counter_++;
  if(raw_gnss_counter_ == 2){
	lcm_.publish("SD_RAW_GNSS", &lcm_raw_gnss_msg_);
	raw_gnss_counter_ = 0;
  }
}

// Callback for ROS raw gps data, merges gps/fix with gps/fix_velocity
// ROS Topic	: gps/fix_velocity
// ROS Type		: geometry_msgs/Vector3Stamped
// LCM Channel	: SD_RAW_GNSS
// LCM Type 	: SD.raw_gnss_t
void App::gps_fix_velocity_cb(const geometry_msgs::Vector3Stamped::ConstPtr &msg){
  // ROS: NWU, to NED
  lcm_raw_gnss_msg_.north_velocity = msg->vector.x;		// North
  lcm_raw_gnss_msg_.east_velocity = -msg->vector.y;		// West
  lcm_raw_gnss_msg_.down_velocity = -msg->vector.z;	// Up

  raw_gnss_counter_++;
  if(raw_gnss_counter_ == 2){
	lcm_.publish("SD_RAW_GNSS", &lcm_raw_gnss_msg_);
	raw_gnss_counter_ = 0;
  }
}

// Callback for ROS imu data, merges with filtered gps data, 15Hz
// ROS Topic	: imu/data
// ROS Type		: sensor_msgs/Imu
// LCM Channel	: SD_STATE
// LCM Type 	: SD.state_t.raw_sensors_t
void App::imu_cb(const sensor_msgs::Imu::ConstPtr &msg){
  SD::raw_sensors_t lcm_msg;
  Eigen::Quaternionf q(msg->orientation.w,
		       msg->orientation.x,
		       msg->orientation.y,
		       msg->orientation.z);

  Eigen::Vector3f euler = qtorpy(q);
  lcm_raw_sensor_msg_.x_magnetometer = -1;	//euler[0];
  lcm_raw_sensor_msg_.y_magnetometer = -1;	//euler[1];
  lcm_raw_sensor_msg_.z_magnetometer = -1;	//euler[2];

  lcm_raw_sensor_msg_.x_acceleration = msg->linear_acceleration.x;
  lcm_raw_sensor_msg_.y_acceleration = msg->linear_acceleration.y;
  lcm_raw_sensor_msg_.z_acceleration = msg->linear_acceleration.z;

  lcm_raw_sensor_msg_.x_gyroscope = msg->angular_velocity.x;
  lcm_raw_sensor_msg_.y_gyroscope = msg->angular_velocity.y;
  lcm_raw_sensor_msg_.z_gyroscope = msg->angular_velocity.z;

  lcm_raw_sensor_msg_.imu_temperature = -1;
  lcm_raw_sensor_msg_.pressure = -1;
  lcm_raw_sensor_msg_.pressure_temperature = -1;

  lcm_raw_sensor_msg_.timestamp.seconds = msg->header.stamp.sec;
  lcm_raw_sensor_msg_.timestamp.microseconds = ((msg->header.stamp.nsec*0.001) + 0.5); // nano-seconds to micro-seconds

//  lcm_.publish("SD_RAW_SENSORS", &lcm_raw_sensor_msg_);
  imu_flag = 1;
  if((imu_flag + state_flag + odom_flag) == 3){
	lcm_state_msg_.raw_sensors = lcm_raw_sensor_msg_;
	lcm_.publish("SD_STATE", &lcm_state_msg_);
	imu_flag = 0;
	state_flag = 0;
	odom_flag = 0;
  }
}

// Callback for ROS filtered gps data, merges with imu data, 30Hz
// ROS Topic	: gps/filtered
// ROS Type		: sensor_msgs/NavSatFix
// LCM Channel	: SD_STATE
// LCM Type 	: SD.state_t.system_state
void App::gps_filtered_cb(const sensor_msgs::NavSatFix::ConstPtr &msg){

  lcm_state_msg_.system_state.latitude = msg->latitude;
  lcm_state_msg_.system_state.longitude = msg->longitude;
  lcm_state_msg_.system_state.height = msg->altitude;
   
  lcm_state_msg_.timestamp.seconds = msg->header.stamp.sec;
  lcm_state_msg_.timestamp.microseconds = ((msg->header.stamp.nsec*0.001) + 0.5); // nano-seconds to micro-seconds
   
  state_flag = 1;
  if((imu_flag + state_flag + odom_flag) == 3){
	lcm_state_msg_.raw_sensors = lcm_raw_sensor_msg_;
	lcm_.publish("SD_STATE", &lcm_state_msg_);
	imu_flag = 0;
	state_flag = 0;
	odom_flag = 0;
  }
}

// Callback for ROS estimated state data, merges with filtered gps data, and imu data 15Hz
// ROS Topic	: gps/filtered
// ROS Type		: sensor_msgs/NavSatFix
// LCM Channel	: SD_STATE
// LCM Type 	: SD.state_t.system_state
void App::odom_gps_cb(const nav_msgs::Odometry::ConstPtr &msg){
  Eigen::Quaternionf q(msg->pose.pose.orientation.w,
		       msg->pose.pose.orientation.x,
		       msg->pose.pose.orientation.y,
		       msg->pose.pose.orientation.z);

  Eigen::Vector3f euler = qtorpy(q);

  lcm_state_msg_.system_state.roll  = euler[0];
  lcm_state_msg_.system_state.pitch = euler[1];
  lcm_state_msg_.system_state.heading   = euler[2];

  lcm_state_msg_.system_state.x_angular_velocity = msg->twist.twist.angular.x;
  lcm_state_msg_.system_state.y_angular_velocity = msg->twist.twist.angular.y;
  lcm_state_msg_.system_state.z_angular_velocity = msg->twist.twist.angular.z;
  
  lcm_state_msg_.system_state.north_velocity = msg->twist.twist.linear.x;
  lcm_state_msg_.system_state.east_velocity = msg->twist.twist.linear.y;
  lcm_state_msg_.system_state.down_velocity = msg->twist.twist.linear.z;

 
  odom_flag = 1;
  if((imu_flag + state_flag + odom_flag) == 3){
	lcm_state_msg_.raw_sensors = lcm_raw_sensor_msg_;
	lcm_.publish("SD_STATE", &lcm_state_msg_);
	imu_flag = 0;
	state_flag = 0;
	odom_flag = 0;
  }
}
// %EndTag(CALLBACK)%

int main(int argc, char **argv)
{
// init LCM
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  new App(n);
  
  std::cout << "ros to lcm translator ready\n";
// %Tag(SPIN)%
  ros::spin();
// %EndTag(SPIN)%

  return 0;
}
// %EndTag(FULLTEXT)%

