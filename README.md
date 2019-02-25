# UON_RobotX_ROS_to_LCM
This repo should be cloned into ~/vmrc_ws/src/

which should've been created following the setup and installation of vmrc

https://bitbucket.org/osrf/vmrc/wiki/tutorials/SystemSetupInstall

# Sensors
- LiDAR
  - Real World: 8 Beam, output format is hvdir (horizontal angle, vert angle, distance, intensity, and ring). M8 \> hvdir \> x,y,z
  - Simulator: 16 Beam
    - ROS Topic: "lidar_wamv/points"
    - Message Type: sensor_msgs/PointCloud2 (x,y,z,intensity,ring)
  - Plugin: https://bitbucket.org/DataspeedInc/velodyne_simulator
  - Underlying alogrithm uses x,y,z just have two different builds, one with ROS and one with M8 Ethernet

- Cameras
  - Real World: Uses Ararvis and TensorFlow python script all wrapped in together and interface directly with camera/USB
  - Simulator:
    - ROS Topic: front_right_camera/image_raw , front_left_camera/image_raw and some other topics
    - Message Type: sensor_msgs/image
  - Plugin: http://docs.ros.org/melodic/api/gazebo_plugins/html/gazebo__ros__camera_8cpp_source.html
  - Training data is of virtual images, kinda wierd, might help somewhat for real world comp

- Current Sensor
  - I don't think this exists in Sim, additionally we were using it for thrust estimation, I think we won't need that in Sim

- IMU
  - Real World: Published over LCM
  - Simulator:
    - ROS Topic: imu/data
    - Message Type: sensor_msgs/Imu
  - Plugin: http://wiki.ros.org/hector_gazebo_plugins

- GPS
  - Real World: Published over LCM
  - Simulator
    - ROS Topic: gps/fix
    - Message Type: sensor_msgs/NavSatFix in WGS84 coordinates (latitude, longitude and altitude)
    - ROS Topic: gps/fix_velocity
    - Message Type: geometry_msgs/Vector3Stamped in NWU (north, west, up) coordinates
  - Plugin: http://wiki.ros.org/hector_gazebo_plugins

- Localisation
  - Real World: Published over LCM
  - Simulator
    - ROS Topic: gps/filtered
    - Message Type: sensor_msgs/NavSatFix
    - ROS Topic: odometry/filtered
    - Message Type: nav_msgs/Odometry
    - ROS Topic: odometry/gps
    - Message Type: nav_msgs/Odometry
  - Plugin: http://wiki.ros.org/robot_localization




