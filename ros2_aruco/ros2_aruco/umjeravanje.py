import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray
import numpy as np
from scipy.spatial.transform import Rotation as R

class DataCollector(Node):
    def __init__(self):
        super().__init__('vector_sum_stats_node')
        self.odom_sub = self.create_subscription(Odometry, '/marinero/main/odometry', self.odom_callback, 10)
        self.poses_sub = self.create_subscription(PoseArray, '/back/aruco/poses', self.poses_callback, 10)

        self.odom_msgs = []
        self.pose_msgs = []
        self.max_count = 200

        ## static transform between base_link and back_right_camera_optical_frame

        self.T_base_link_to_back_right_camera_optical = np.array([
            [0.0, 0.0, -1.0, -0.4626],
            [1.0, 0.0,  0.0,  0.0124],
            [0.0, -1.0, 0.0,  0.0767],
            [0.0, 0.0,  0.0,  1.0]
        ])

        self.last_odometry = Odometry()

        self.xyz_list = []

    def destroy_node(self):
        self.compute_stats()
        super().destroy_node()


    def odom_callback(self, msg):
        self.last_odometry = msg

    def poses_callback(self, msg):
        one_pose = msg.poses[0]

        xyz_optical = np.array([one_pose.position.x, one_pose.position.y, one_pose.position.z])
        quat_optical = np.array([one_pose.orientation.x, one_pose.orientation.y, one_pose.orientation.z, one_pose.orientation.w])

        T_optical = np.eye(4)
        T_optical[:3, :3] = R.from_quat(quat_optical).as_matrix()
        T_optical[:3, 3] = xyz_optical

        T_base_link_to_camera = self.T_base_link_to_back_right_camera_optical

        T_base_link = T_base_link_to_camera @ T_optical  # base_link ← optical

        world_pos = np.array([
            self.last_odometry.pose.pose.position.x,
            self.last_odometry.pose.pose.position.y,
            self.last_odometry.pose.pose.position.z
        ])
        world_quat = np.array([
            self.last_odometry.pose.pose.orientation.x,
            self.last_odometry.pose.pose.orientation.y,
            self.last_odometry.pose.pose.orientation.z,
            self.last_odometry.pose.pose.orientation.w
        ])
        T_world_base = np.eye(4)
        T_world_base[:3, :3] = R.from_quat(world_quat).as_matrix()
        T_world_base[:3, 3] = world_pos

        T_world_optical = T_world_base @ T_base_link  # world ← optical

        xyz_world = T_world_optical[:3, 3]
        quat_world = R.from_matrix(T_world_optical[:3, :3]).as_quat()

        self.xyz_list.append(xyz_world)


    def compute_stats(self):
        arr = np.array(self.xyz_list)

        mean = np.mean(arr, axis=0)
        stddev = np.std(arr, axis=0)

        print(f"Mean:    x={mean[0]:.3f}, y={mean[1]:.3f}, z={mean[2]:.3f}")
        print(f"Stddev:  x={stddev[0]:.3f}, y={stddev[1]:.3f}, z={stddev[2]:.3f}")
        print(f"N of measurements:{len(self.xyz_list)}")


def main(args=None):
    rclpy.init(args=args)
    node = DataCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()   # this triggers compute_stats()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
