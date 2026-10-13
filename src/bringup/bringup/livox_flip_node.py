"""
Node that flips Livox MID-360 point cloud and IMU data 180 degrees around the roll axis.

The lidar is installed upside-down (180 degrees roll), so this node applies
a 180-degree rotation around the X axis to all point coordinates and IMU
measurements using SciPy's Rotation class for the transformation.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R

from livox_ros_driver2.msg import CustomMsg, CustomPoint
from sensor_msgs.msg import Imu


class LivoxFlipNode(Node):
    def __init__(self):
        super().__init__("livox_flip_node")

        # Declare & read topic parameters
        self.declare_parameter("input_lidar_topic", "/livox/lidar")
        self.declare_parameter("output_lidar_topic", "/livox/lidar_flipped")
        self.declare_parameter("input_imu_topic", "/livox/imu")
        self.declare_parameter("output_imu_topic", "/livox/imu_flipped")

        input_lidar = self.get_parameter("input_lidar_topic").value
        output_lidar = self.get_parameter("output_lidar_topic").value
        input_imu = self.get_parameter("input_imu_topic").value
        output_imu = self.get_parameter("output_imu_topic").value

        self.get_logger().info(
            f"Lidar: {input_lidar} -> {output_lidar} | IMU: {input_imu} -> {output_imu}"
        )

        # Pre-build the 180-degree roll (X-axis) rotation using SciPy
        self.rotation = R.from_euler("x", 180, degrees=True)

        self.lidar_sub = self.create_subscription(
            CustomMsg, input_lidar, self._lidar_callback, 10
        )
        self.lidar_pub = self.create_publisher(CustomMsg, output_lidar, 10)

        self.imu_sub = self.create_subscription(
            Imu, input_imu, self._imu_callback, 10
        )
        self.imu_pub = self.create_publisher(Imu, output_imu, 10)

    def _lidar_callback(self, msg: CustomMsg):
        out = CustomMsg()
        out.header = msg.header
        out.timebase = msg.timebase
        out.point_num = msg.point_num
        out.lidar_id = msg.lidar_id
        out.rsvd = msg.rsvd

        # Collect all points into a (N, 3) array, transform with SciPy, then write back
        positions = np.array([[p.x, p.y, p.z] for p in msg.points], dtype=np.float64)
        transformed = self.rotation.apply(positions)

        out.points = []
        for i, pt in enumerate(msg.points):
            new_pt = CustomPoint()
            new_pt.offset_time = pt.offset_time
            new_pt.x = float(transformed[i, 0])
            new_pt.y = float(transformed[i, 1])
            new_pt.z = float(transformed[i, 2])
            new_pt.reflectivity = pt.reflectivity
            new_pt.tag = pt.tag
            new_pt.line = pt.line
            out.points.append(new_pt)

        self.lidar_pub.publish(out)

    def _imu_callback(self, msg: Imu):
        out = Imu()
        out.header = msg.header

        # Transform linear acceleration vector
        acc = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        transformed_acc = self.rotation.apply(acc)
        out.linear_acceleration.x = float(transformed_acc[0])
        out.linear_acceleration.y = float(transformed_acc[1])
        out.linear_acceleration.z = float(transformed_acc[2])

        # Transform angular velocity vector
        gyro = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        transformed_gyro = self.rotation.apply(gyro)
        out.angular_velocity.x = float(transformed_gyro[0])
        out.angular_velocity.y = float(transformed_gyro[1])
        out.angular_velocity.z = float(transformed_gyro[2])

        # Transform orientation quaternion by composing the flip rotation
        q = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        if np.linalg.norm(q) > 1e-6:
            original_rot = R.from_quat(q)
            transformed_rot = self.rotation * original_rot
            new_q = transformed_rot.as_quat()  # xyzw
            out.orientation.x = float(new_q[0])
            out.orientation.y = float(new_q[1])
            out.orientation.z = float(new_q[2])
            out.orientation.w = float(new_q[3])
        else:
            # Identity quaternion passed through
            out.orientation = msg.orientation

        # Covariance matrices transformed: R_sigmas R_sigmas^T
        if any(msg.linear_acceleration_covariance):
            cov_lin = np.array(msg.linear_acceleration_covariance).reshape(3, 3)
            out.linear_acceleration_covariance = (
                self.rotation.as_matrix() @ cov_lin @ self.rotation.as_matrix().T
            ).flatten().tolist()
        else:
            out.linear_acceleration_covariance = msg.linear_acceleration_covariance

        if any(msg.orientation_covariance):
            cov_or = np.array(msg.orientation_covariance).reshape(3, 3)
            out.orientation_covariance = (
                self.rotation.as_matrix() @ cov_or @ self.rotation.as_matrix().T
            ).flatten().tolist()
        else:
            out.orientation_covariance = msg.orientation_covariance

        self.imu_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = LivoxFlipNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
