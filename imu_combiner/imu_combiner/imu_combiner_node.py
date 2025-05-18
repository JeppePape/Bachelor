import rclpy
from rclpy.node import Node
from px4_msgs.msg import SensorCombined, VehicleAttitude
from sensor_msgs.msg import Imu, Image
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import csv
import time

class IMUCombiner(Node):
    def __init__(self):
        super().__init__('imu_combiner')

        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.get_logger().info(f"Setting QoS to: {qos_profile.reliability}")

        # Subscriptions
        self.subscription_sensor = self.create_subscription(
            SensorCombined, '/fmu/out/sensor_combined', self.sensor_callback, qos_profile
        )
        self.subscription_attitude = self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude', self.attitude_callback, qos_profile
        )
        self.subscription_combined = self.create_subscription(
            Imu, '/imu_combined', self.imu_combined_callback, qos_profile
        )
        self.subscription_imu0 = self.create_subscription(
            Imu, '/imu0', self.imu0_callback, qos_profile
        )
        self.subscription_image = self.create_subscription(
            Image, '/cam0/image_raw', self.image_callback, qos_profile
        )

        # Publishers
        self.imu_publisher = self.create_publisher(Imu, '/imu_combined', qos_profile)

        # CSV Files
        self.sensor_csv = "sensor_log.csv"
        self.vehicle_csv = "vehicle_log.csv"
        self.combined_csv = "imu_combined_log.csv"
        self.imu0_csv = "imu0_log.csv"
        self.image_csv = "image_timestamps.csv"

        self.csv_headers = ["timestamp_sec", "timestamp_nanosec",
                            "accel_x", "accel_y", "accel_z",
                            "gyro_x", "gyro_y", "gyro_z",
                            "orient_x", "orient_y", "orient_z", "orient_w"]

        for csv_file in [self.sensor_csv, self.vehicle_csv, self.combined_csv, self.imu0_csv]:
            with open(csv_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(self.csv_headers)

        with open(self.image_csv, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["timestamp_sec", "timestamp_nanosec"])

        self.timer = self.create_timer(1 / 125, self.publish_imu)
        self.get_logger().info("✅ IMUCombiner node initialized with BEST_EFFORT QoS")

        self.sensor_timestamp = (0, 0)
        self.sensor_accel = [0.0, 0.0, 0.0]
        self.sensor_gyro = [0.0, 0.0, 0.0]

        self.attitude_timestamp = (0, 0)
        self.attitude_orientation = [0.0, 0.0, 0.0, 1.0]

        self.last_publish_time = time.time()
        self.publish_timeout = 0.1

    def sensor_callback(self, msg):
        self.sensor_timestamp = self.format_timestamp(msg.timestamp)
        self.sensor_accel = [float(msg.accelerometer_m_s2[0]), float(msg.accelerometer_m_s2[1]), float(msg.accelerometer_m_s2[2])]
        self.sensor_gyro = [float(msg.gyro_rad[0]), float(msg.gyro_rad[1]), float(msg.gyro_rad[2])]
        self.log_data(self.sensor_csv, self.sensor_timestamp, self.sensor_accel, self.sensor_gyro, None)

    def attitude_callback(self, msg):
        self.attitude_timestamp = self.format_timestamp(msg.timestamp)
        self.attitude_orientation = [float(msg.q[1]), float(msg.q[2]), float(msg.q[3]), float(msg.q[0])]
        self.log_data(self.vehicle_csv, self.attitude_timestamp, None, None, self.attitude_orientation)

    def imu_combined_callback(self, msg):
        timestamp = (msg.header.stamp.sec, f"{msg.header.stamp.nanosec:09d}")
        accel = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        gyro = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
        orientation = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        self.log_data(self.combined_csv, timestamp, accel, gyro, orientation)

    def imu0_callback(self, msg):
        timestamp = (msg.header.stamp.sec, f"{msg.header.stamp.nanosec:09d}")
        accel = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        gyro = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
        orientation = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        self.log_data(self.imu0_csv, timestamp, accel, gyro, orientation)

    def image_callback(self, msg):
        timestamp = (msg.header.stamp.sec, f"{msg.header.stamp.nanosec:09d}")
        with open(self.image_csv, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([timestamp[0], timestamp[1]])
        self.get_logger().info(f"📸 Logged image timestamp: {timestamp}")

    def publish_imu(self):
        if self.sensor_timestamp == (0, 0) or self.attitude_timestamp == (0, 0):
            self.get_logger().warn("⚠️ Missing sensor or attitude data!")
            return

        imu_msg = Imu()
        imu_msg.header.stamp.sec = self.sensor_timestamp[0]
        imu_msg.header.stamp.nanosec = int(self.sensor_timestamp[1])
        imu_msg.header.frame_id = 'base_link'

        imu_msg.linear_acceleration.x = self.sensor_accel[0]
        imu_msg.linear_acceleration.y = self.sensor_accel[1]
        imu_msg.linear_acceleration.z = self.sensor_accel[2]

        imu_msg.angular_velocity.x = self.sensor_gyro[0]
        imu_msg.angular_velocity.y = self.sensor_gyro[1]
        imu_msg.angular_velocity.z = self.sensor_gyro[2]

        imu_msg.orientation.x = self.attitude_orientation[0]
        imu_msg.orientation.y = self.attitude_orientation[1]
        imu_msg.orientation.z = self.attitude_orientation[2]
        imu_msg.orientation.w = self.attitude_orientation[3]

        self.imu_publisher.publish(imu_msg)
        self.get_logger().info("✅ Published COMBINED IMU Data")

    def format_timestamp(self, timestamp):
        sec = int(timestamp / 1e6)
        nanosec = f"{int((timestamp % 1e6) * 1e3):09d}"
        return sec, nanosec

    def log_data(self, file_name, timestamp, accel=None, gyro=None, orientation=None):
        if accel is None:
            accel = [0.0, 0.0, 0.0]
        if gyro is None:
            gyro = [0.0, 0.0, 0.0]
        if orientation is None:
            orientation = [0.0, 0.0, 0.0, 1.0]

        log_entry = [
            timestamp[0], timestamp[1],
            accel[0], accel[1], accel[2],
            gyro[0], gyro[1], gyro[2],
            orientation[0], orientation[1], orientation[2], orientation[3]
        ]

        with open(file_name, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(log_entry)

        self.get_logger().info(f"📊 Logged Data to {file_name}: {log_entry}")

def main(args=None):
    rclpy.init(args=args)
    imu_combiner = IMUCombiner()
    rclpy.spin(imu_combiner)
    imu_combiner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
