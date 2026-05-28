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
ACCEL_XOUT_H = 0x3B   # block-read base: accel(6) + temp(2) + gyro(6) = 14 bytes
DEVICE_ADDRESS = 0x68

# ── Scale factors ─────────────────────────────────────────────────────────
import math
# Accelerometer: ±2 g → 16384 LSB/g
ACCEL_SCALE = 16384.0 / 9.80665        # LSB per (m/s²)  ≈ 1670.70

# Gyroscope: FS_SEL=0 → ±250°/s → 131 LSB/°/s → convert to rad/s
GYRO_SCALE  = 131.0 * (180.0 / math.pi)  # LSB per (rad/s) ≈ 7505.75


class MPU6050Driver(Node):

    def __init__(self):
        super().__init__("mpu6050_driver")

        self.is_connected_ = False
        self.init_i2c()

        self.imu_pub_ = self.create_publisher(Imu, "/imu/out", 10)

        self.imu_msg_ = Imu()
        self.imu_msg_.header.frame_id = "imu_link"

        # No orientation computed here — imu_filter_madgwick handles fusion
        self.imu_msg_.orientation_covariance[0] = -1.0

        # Noise covariances — tuned for MPU6050 at 100 Hz with DLPF 44 Hz
        self.imu_msg_.linear_acceleration_covariance[0] = 0.04
        self.imu_msg_.linear_acceleration_covariance[4] = 0.04
        self.imu_msg_.linear_acceleration_covariance[8] = 0.04

        self.imu_msg_.angular_velocity_covariance[0] = 0.02
        self.imu_msg_.angular_velocity_covariance[4] = 0.02
        self.imu_msg_.angular_velocity_covariance[8] = 0.02

        self.timer_ = self.create_timer(0.01, self.timer_callback)  # 100 Hz

    def timer_callback(self):
        if not self.is_connected_:
            self.init_i2c()
            return

        try:
            # Read 14 bytes atomically starting at ACCEL_XOUT_H:
            #   [0-1] accel_x  [2-3] accel_y  [4-5] accel_z
            #   [6-7] temp     (skipped)
            #   [8-9] gyro_x   [10-11] gyro_y  [12-13] gyro_z
            data = self.bus_.read_i2c_block_data(DEVICE_ADDRESS, ACCEL_XOUT_H, 14)

            ax = self._s16(data[0],  data[1])
            ay = self._s16(data[2],  data[3])
            az = self._s16(data[4],  data[5])
            gx = self._s16(data[8],  data[9])
            gy = self._s16(data[10], data[11])
            gz = self._s16(data[12], data[13])

            self.imu_msg_.linear_acceleration.x = ax / ACCEL_SCALE
            self.imu_msg_.linear_acceleration.y = ay / ACCEL_SCALE
            self.imu_msg_.linear_acceleration.z = az / ACCEL_SCALE

            self.imu_msg_.angular_velocity.x = gx / GYRO_SCALE
            self.imu_msg_.angular_velocity.y = gy / GYRO_SCALE
            self.imu_msg_.angular_velocity.z = gz / GYRO_SCALE

            self.imu_msg_.header.stamp = self.get_clock().now().to_msg()
            self.imu_pub_.publish(self.imu_msg_)

        except OSError:
            self.get_logger().warn("MPU6050: I2C read error — will retry")
            self.is_connected_ = False

    def init_i2c(self):
        try:
            self.bus_ = smbus.SMBus(1)
            # 1 kHz / (9+1) = 100 Hz — matches timer rate
            self.bus_.write_byte_data(DEVICE_ADDRESS, SMPLRT_DIV,  9)
            # Wake up, select X gyro as clock source (more stable than internal 8 MHz)
            self.bus_.write_byte_data(DEVICE_ADDRESS, PWR_MGMT_1,  1)
            # DLPF_CFG=3 → 44 Hz accel / 42 Hz gyro bandwidth — filters stepper motor vibration
            self.bus_.write_byte_data(DEVICE_ADDRESS, CONFIG,       3)
            # FS_SEL=0 → ±250°/s gyro range (highest resolution, sufficient for diff-drive)
            self.bus_.write_byte_data(DEVICE_ADDRESS, GYRO_CONFIG,  0)
            # Enable data-ready interrupt
            self.bus_.write_byte_data(DEVICE_ADDRESS, INT_ENABLE,   1)
            self.is_connected_ = True
            self.get_logger().info("MPU6050: connected on I2C bus 1 at 0x68")
        except OSError:
            self.get_logger().warn("MPU6050: I2C connection failed — will retry")
            self.is_connected_ = False

    @staticmethod
    def _s16(high, low):
        v = (high << 8) | low
        return v - 65536 if v >= 32768 else v


def main():
    rclpy.init()
    node = MPU6050Driver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
