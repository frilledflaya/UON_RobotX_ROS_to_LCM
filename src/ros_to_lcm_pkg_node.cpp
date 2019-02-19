// %Tag(FULLTEXT)%
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>

#include <lcm/lcm-cpp.hpp>

class App{
	public:
	  App(ros::NodeHandle node_);
	  ~App();

	private:
	  lcm::LCM lcm_;
	  ros::NodeHandle node_;
	  
	  ros::Subscriber  imu_sub_;
	  void imu_cb(const sensor_msgs::Imu::ConstPtr &msg);
};

App::App(ros::NodeHandle node_):
  node_(node_){

ROS_INFO("Initializing Translator");

  if(!lcm_.good()){
	std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }

 
// %Tag(SUBSCRIBER)%
  imu_sub_ = node_.subscribe(std::string("imu/data"), 1000, &App::imu_cb, this);
// %EndTag(SUBSCRIBER)%


};

App::~App() {
};

// %Tag(CALLBACK)%
void App::imu_cb(const sensor_msgs::Imu::ConstPtr &msg)
{
  ROS_INFO("I heard: [%d]", msg->header.seq);
  lcm_.publish("EXAMPLE", &msg->header.seq, 2);
}
// %EndTag(CALLBACK)%

int main(int argc, char **argv)
{
// init LCM

  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  new App(n);
  
  std::cout << "ros2lcm translator ready\n";
// %Tag(SPIN)%
  ros::spin();
// %EndTag(SPIN)%

  return 0;
}
// %EndTag(FULLTEXT)%

