#!/bin/python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile
# IMU
from sensor_msgs.msg import Imu

# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

import time
import board
import adafruit_bno055

class bno055_ros2(Node):

    def __init__(self) -> None:
        super().__init__("scripts_main")

        i2c = board.I2C()
        self.sensor = adafruit_bno055.BNO055_I2C(i2c)
        self.last_val = 0xFFFF

        pub_qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1)

        self.pub = self.create_publisher(Imu, 'pub_bno055',pub_qos)


        self.declare_parameter('pub_rate',30)
        param = self.get_parameter('pub_rate')
        hz = param.get_parameter_value().integer_value
        self.get_logger().info(str())
        period = 1/hz
        self.get_logger().info('period: '+str(period))
        
        self.create_timer(period,self.publish_as_imu)

# except:
            # msg.orientation.x = 0
    def publish_as_imu(self):
        try:
            qx, qy, qz, qw = self.sensor.quaternion
            gx, gy, gz = self.sensor.gyro
            ax, ay, az = self.sensor.linear_acceleration

            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'imu'
            msg.orientation.x = float(qx)
            msg.orientation.y = float(qy)
            msg.orientation.z = float(qz)
            msg.orientation.w = float(qw)
            # msg.orientation_covariance = [1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6]
            msg.angular_velocity.x = float(gx)
            msg.angular_velocity.y = float(gy)
            msg.angular_velocity.z = float(gz)
            # msg.angular_velocity_covariance = [1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6]
            msg.linear_acceleration.x = float(ax)
            msg.linear_acceleration.y = float(ay)
            msg.linear_acceleration.z = float(az)
            # msg.linear_acceleration_covariance = [1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6]
            self.pub.publish(msg)
        except:
            pass


    def temperature(self):
        global last_val  # pylint: disable=global-statement
        result = self.sensor.temperature
        if abs(self.result - self.last_val) == 128:
            result = self.sensor.temperature
            if abs(result - self.last_val) == 128:
                return 0b00111111 & result
        last_val = result
        return result

def ros_main(args = None):
    rclpy.init(args=args)
    ros_class = bno055_ros2()
    
    try:
        rclpy.spin(ros_class)
    except KeyboardInterrupt:
        pass
    finally:
        ros_class.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    ros_main()