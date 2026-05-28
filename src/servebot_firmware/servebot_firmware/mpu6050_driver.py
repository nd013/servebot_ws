#!/usr/bin/env python3
import smbus
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

# ── MPU6050 register addresses ────────────────────────────────────────────
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47
DEVICE_ADDRESS = 0x68

# ── Scale factors ─────────────────────────────────────────────────────────
# Accelerometer: default ±2 g range → 16384 LSB/g → divide by 16384/9.80665 = 1670.13
ACCEL_SCALE = 1670.13   # LSB → m/s²

# Gyroscope: GYRO_CONFIG=0x00 → FS_SEL=0 → ±250°/s → 131 LSB/°/s
#            divide by 131 * (180/π) = 7509.87 to get rad/s
GYRO_SCALE  = 7509.87   # LSB → rad/s


class MPU6050Driver(Node):

    def __init__(self):
        super().__init__("mpu6050_driver")

        self.is_connected_ = False
        self.init_i2c()

        self.imu_pub_ = self.create_publisher(Imu, "/imu/out", 10)

        self.imu_msg_ = Imu()
        self.imu_msg_.header.frame_id = "imu_link"

        # Orientation is not computed by this node
        self.imu_msg_.orientation_covariance[0] = -1.0

        # Approximate noise covariances (diagonal)
        self.imu_msg_.linear_acceleration_covariance[0] = 0.01
        self.imu_msg_.linear_acceleration_covariance[4] = 0.01
        self.imu_msg_.linear_acceleration_covariance[8] = 0.01

        self.imu_msg_.angular_velocity_covariance[0] = 0.001
        self.imu_msg_.angular_velocity_covariance[4] = 0.001
        self.imu_msg_.angular_velocity_covariance[8] = 0.001

        # 100 Hz
        self.timer_ = self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):
        if not self.is_connected_:
            self.init_i2c()
            return

        try:
            acc_x = self.read_raw_data(ACCEL_XOUT_H)
            acc_y = self.read_raw_data(ACCEL_YOUT_H)
            acc_z = self.read_raw_data(ACCEL_ZOUT_H)

            gyro_x = self.read_raw_data(GYRO_XOUT_H)
            gyro_y = self.read_raw_data(GYRO_YOUT_H)
            gyro_z = self.read_raw_data(GYRO_ZOUT_H)

            self.imu_msg_.linear_acceleration.x = acc_x / ACCEL_SCALE
            self.imu_msg_.linear_acceleration.y = acc_y / ACCEL_SCALE
            self.imu_msg_.linear_acceleration.z = acc_z / ACCEL_SCALE

            self.imu_msg_.angular_velocity.x = gyro_x / GYRO_SCALE
            self.imu_msg_.angular_velocity.y = gyro_y / GYRO_SCALE
            self.imu_msg_.angular_velocity.z = gyro_z / GYRO_SCALE

            self.imu_msg_.header.stamp = self.get_clock().now().to_msg()
            self.imu_pub_.publish(self.imu_msg_)

        except OSError:
            self.get_logger().warn("MPU6050: I2C read error — will retry")
            self.is_connected_ = False

    def init_i2c(self):
        try:
            self.bus_ = smbus.SMBus(1)
            self.bus_.write_byte_data(DEVICE_ADDRESS, SMPLRT_DIV, 7)   # 1 kHz / (7+1) = 125 Hz sample rate
            self.bus_.write_byte_data(DEVICE_ADDRESS, PWR_MGMT_1, 1)   # wake up, use X gyro clock
            self.bus_.write_byte_data(DEVICE_ADDRESS, CONFIG, 0)        # no DLPF
            self.bus_.write_byte_data(DEVICE_ADDRESS, GYRO_CONFIG, 0)   # FS_SEL=0 → ±250°/s
            self.bus_.write_byte_data(DEVICE_ADDRESS, INT_ENABLE, 1)    # data-ready interrupt
            self.is_connected_ = True
            self.get_logger().info("MPU6050: connected on I2C bus 1 at 0x68")
        except OSError:
            self.get_logger().warn("MPU6050: I2C connection failed — will retry")
            self.is_connected_ = False

    def read_raw_data(self, addr):
        high = self.bus_.read_byte_data(DEVICE_ADDRESS, addr)
        low  = self.bus_.read_byte_data(DEVICE_ADDRESS, addr + 1)
        value = (high << 8) | low
        if value > 32768:
            value -= 65536
        return value


def main():
    rclpy.init()
    node = MPU6050Driver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
