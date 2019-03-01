// %Tag(FULLTEXT)%
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>

#include <lcm/lcm-cpp.hpp>
#include <Eigen/Dense>

#include <math.h>

class App{
	public:
	  App(ros::NodeHandle node_);
	  ~App();

	private:
	  lcm::LCM lcm_;
	  ros::NodeHandle node_;
	  
	  Eigen::MatrixXd::MatrixXd map_(100,100);
	  void App::bresh(x0,y0,x1,y1);
	  
	  ros::Subscriber  lidar_sub_;
	  void lidar_cb(const sensor_msgs::PointCloud2::ConstPtr &msg);
};

App::App(ros::NodeHandle node_):
  node_(node_){

  ROS_INFO("Initializing Translator");

  if(!lcm_.good()){
	std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }

 
// %Tag(SUBSCRIBER)%
  lidar_sub_ = node_.subscribe(std::string("lidar_wamv/points"), 1000, &App::lidar_cb, this);
// %EndTag(SUBSCRIBER)%


};

App::~App() {
};

// %Tag(CALLBACK)%
void App::lidar_cb(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  ROS_INFO("I heard: [%d]", msg->header.seq);
  lcm_.publish("EXAMPLE", &msg->header.seq, 2);
}
// %EndTag(CALLBACK)%

void App::bresh(x0,y0,x1,y1){
	float deltax   = x1 - x0;
	float deltay   = y1 - y0;
	float deltaErr = abs(deltay / deltax);
	float error = 0;
	
	int y = y0;
	for (int x = 0; x < x1; x++){
		error += deltaErr;
		if (error >= 0.5){
			y += ((deltay > 0) - (deltay < 0));
			error--;
		}
	}
}

int main(int argc, char **argv){
  ros::init(argc, argv, "mapper");
  ros::NodeHandle n;
  new App(n);
  
  std::cout << "mapper ready\n";
// %Tag(SPIN)%
  ros::spin();
// %EndTag(SPIN)%

  return 0;
}
// %EndTag(FULLTEXT)%