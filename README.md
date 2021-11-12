# rclpy BNO055 Publisher

## Requirements

- ROS2 (Foxy)
- Adafruit BNO055 IMU
- Sensor_msgs/Imu

## Installation

```bash
pip3 install adafruit-circuitpython-bno055
cd ~/
mkdir -p ros2_ws/src
cd ros2_ws/src
git clone https://github.com/Ar-Ray-code/rclpy_bno055_publisher.git
cd ~/ros2_ws
source /opt/ros/foxy/setup.bash
colcon build --symlink-install
```

## Run

```bash
source /opt/ros/foxy/setup.bash
source ~/ros2_ws/install/local_setup.bash
ros2 run rclpy_bno055_publisher bno055_publisher
```

## Listen topic

```bash
ros2 topic echo /pub_bno055 
```