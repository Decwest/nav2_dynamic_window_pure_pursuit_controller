#!/usr/bin/env python3
# ground_truth_odom_tf.py
import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class GroundTruthOdomTF(Node):
    def __init__(self):
        super().__init__('ground_truth_odom_tf')
        self.declare_parameter('model_name', 'turtlebot3_waffle')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.model_name = self.get_parameter('model_name').get_parameter_value().string_value
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.sub = self.create_subscription(ModelStates, '/gazebo/model_states', self.cb, 10)
        self.get_logger().info(f'Using Gazebo ground truth of "{self.model_name}"')

    def cb(self, msg: ModelStates):
        try:
            idx = msg.name.index(self.model_name)
        except ValueError:
            return
        pose = msg.pose[idx]
        twist = msg.twist[idx]

        # 現在のシミュレーション時間を使用（少し未来のタイムスタンプで公開）
        now = self.get_clock().now()
        future_time = now + rclpy.duration.Duration(seconds=0.1)
        stamp = future_time.to_msg()

        # /odom
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose = pose
        odom.twist.twist = twist
        self.odom_pub.publish(odom)

        # TF: odom -> base_link
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z
        t.transform.rotation = pose.orientation
        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = GroundTruthOdomTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
