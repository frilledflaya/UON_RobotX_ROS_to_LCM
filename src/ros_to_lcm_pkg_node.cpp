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

class App{
	public:
	  App(ros::NodeHandle node_);
	  ~App();

	private:
	  lcm::LCM lcm_;
	  ros::NodeHandle node_;
	  
	  ros::Subscriber  imu_sub_;
	  void imu_cb(const sensor_msgs::Imu::ConstPtr &msg);
	  
	  ros::Subscriber  gps_fix_sub_;
	  void gps_fix_cb(const sensor_msgs::NavSatFix::ConstPtr &msg);

	  ros::Subscriber  gps_fix_velocity_sub_;
	  void gps_fix_velocity_cb(const geometry_msgs::Vector3Stamped::ConstPtr &msg);

	  ros::Subscriber  gps_filtered_sub_;
	  void gps_filtered_cb(const sensor_msgs::NavSatFix::ConstPtr &msg);

	  ros::Subscriber  odom_filtered_sub_;
	  void odom_filtered_cb(const nav_msgs::Odometry::ConstPtr &msg);

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
  odom_filtered_sub_	= node_.subscribe(std::string("odometry/filtered"),1000, &App::odom_filtered_cb, this);
// %EndTag(SUBSCRIBER)%

};

App::~App() {
};

// %Tag(CALLBACK)%
void App::imu_cb(const sensor_msgs::Imu::ConstPtr &msg){
  SD::raw_sensors_t lcm_msg;
  Eigen::Quaternionf q(msg->orientation.w,
		       msg->orientation.x,
		       msg->orientation.y,
		       msg->orientation.z);
//  Eigen::Vector3f euler = q.normalized().toRotationMatrix().eulerAngles(2, 1, 0);
// Quaternion to rotation matrix to ZYX euler
// TODO: put in a function
  Eigen::Matrix3f qrot;
  float sy, roll, pitch, yaw;
  
  qrot = q.normalized().toRotationMatrix();
  sy = sqrt(qrot(0,0)*qrot(0,0) + qrot(1,0)*qrot(1,0));
  if (sy >= 0.000001){ // sy < 1e-6 is singular
	roll  = atan2(qrot(2,1) ,qrot(2,2));
	pitch = atan2(-qrot(2,0),sy);
	yaw   = atan2(qrot(1,0) ,qrot(0,0));
  } else {
	roll  = atan2(-qrot(1,2),qrot(1,1));
	pitch = atan2(-qrot(2,0),sy);
	yaw   = 0;
  }
//

  lcm_msg.x_magnetometer = roll;
  lcm_msg.y_magnetometer = pitch;
  lcm_msg.z_magnetometer = yaw;

  lcm_msg.x_acceleration = msg->linear_acceleration.x;
  lcm_msg.y_acceleration = msg->linear_acceleration.y;
  lcm_msg.z_acceleration = msg->linear_acceleration.z;

  lcm_msg.x_gyroscope = msg->angular_velocity.x;
  lcm_msg.y_gyroscope = msg->angular_velocity.y;
  lcm_msg.z_gyroscope = msg->angular_velocity.z;

  lcm_msg.imu_temperature = -1;
  lcm_msg.pressure = -1;
  lcm_msg.pressure_temperature = -1;

  lcm_msg.timestamp.seconds = msg->header.stamp.sec;
  lcm_msg.timestamp.microseconds = ((msg->header.stamp.nsec*0.001) + 0.5); // nano-seconds to micro-seconds

  lcm_.publish("SD_RAW_SENSORS", &lcm_msg);
}

void App::gps_fix_cb(const sensor_msgs::NavSatFix::ConstPtr &msg){
//  ROS_INFO("I heard: [%d]", msg->header.seq);
//  lcm_.publish("EXAMPLE", &msg->header.seq, 2);
}

void App::gps_fix_velocity_cb(const geometry_msgs::Vector3Stamped::ConstPtr &msg){
//  ROS_INFO("I heard: [%d]", msg->header.seq);
//  lcm_.publish("EXAMPLE", &msg->header.seq, 2);
}

void App::gps_filtered_cb(const sensor_msgs::NavSatFix::ConstPtr &msg){

}

void App::odom_filtered_cb(const nav_msgs::Odometry::ConstPtr &msg){

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

