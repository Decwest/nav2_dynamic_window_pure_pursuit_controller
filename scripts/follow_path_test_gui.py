#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path, Odometry
from nav2_msgs.action import FollowPath
from visualization_msgs.msg import Marker, MarkerArray

# --- Gazebo services (両対応) ---
try:
    from gazebo_msgs.srv import SetEntityState as _SetEntityState
    from gazebo_msgs.msg import EntityState as _EntityState
except Exception:
    _SetEntityState = None
    _EntityState = None

try:
    from gazebo_msgs.srv import SetModelState as _SetModelState
    from gazebo_msgs.msg import ModelState as _ModelState
except Exception:
    _SetModelState = None
    _ModelState = None


def yaw_to_quat(z_yaw_rad: float):
    half = z_yaw_rad * 0.5
    qz = math.sin(half)
    qw = math.cos(half)
    return (0.0, 0.0, qz, qw)


def make_path(frame_id: str):
    paths = []
    theta_list = [np.pi/4, np.pi/2, 3*np.pi/4]
    l = 3.0

    for theta in theta_list:
        x1 = np.linspace(0, 1, 100);           y1 = np.zeros_like(x1)
        x2 = np.linspace(1.0, 1.0+l*math.cos(theta), 100)
        y2 = np.linspace(0.0, l*math.sin(theta), 100)
        x3 = np.linspace(1.0+l*math.cos(theta), 4.0+l*math.cos(theta), 100)
        y3 = np.ones_like(x3) * l * math.sin(theta)

        xs = np.concatenate([x1, x2, x3])
        ys = np.concatenate([y1, y2, y3])
        dx = np.gradient(xs); dy = np.gradient(ys)
        yaws = np.unwrap(np.arctan2(dy, dx))

        path = Path()
        path.header.frame_id = frame_id
        for x, y, yaw in zip(xs, ys, yaws):
            ps = PoseStamped()
            ps.header.frame_id = frame_id
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            _, _, qz, qw = yaw_to_quat(yaw)
            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw
            path.poses.append(ps)
        paths.append(path)

    path_A, path_B, path_C = paths
    return (path_A, path_B, path_C)


class FollowPathClient(Node):
    def __init__(self, frame_id: str = "map"):
        super().__init__('follow_path_gui_client')
        self._client = ActionClient(self, FollowPath, '/follow_path')
        self._current_goal_handle = None
        self._frame_id = frame_id

        # --- QoS（RVizに残るようにTRASIENT_LOCAL） ---
        latched_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        # Initial pose publisher (RViz "2D Pose Estimate")
        self._initpose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)

        # --- Visualization publishers ---
        self._label_pub  = self.create_publisher(MarkerArray, '/viz/path_labels', latched_qos)
        self._path_markers_pub = self.create_publisher(MarkerArray, '/viz/path_markers', latched_qos)

        # Robot trajectory (subscribe odom -> publish Visualization Marker)
        self._traj_pub = self.create_publisher(Marker, '/viz/robot_traj', 10)
        self._traj_points = []
        self._traj_frame_id = 'odom'  # 走行軌跡は odom フレームで可視化
        self._traj_lock = threading.Lock()
        self._odom_sub = self.create_subscription(Odometry, '/odom', self._on_odom, qos_profile_sensor_data)

        # Gazebo warp clients
        self._gz_entity_cli = None
        self._gz_model_cli  = None

        # 起動直後：初期姿勢 & tb3 ワープ & パス定期描画
        threading.Thread(target=self._auto_publish_initial_pose, daemon=True).start()
        threading.Thread(target=self._auto_warp_tb3, daemon=True).start()
        threading.Thread(target=self._periodic_path_publish, daemon=True).start()

    # ===== RViz 可視化：パス & ラベル =====
    def publish_paths_and_labels(self, path_A: Path, path_B: Path, path_C: Path):
        # Path visualization markers with different colors
        markers = MarkerArray()
        colors = [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)]  # Red, Green, Blue
        paths = [('Path A', path_A), ('Path B', path_B), ('Path C', path_C)]
        now = self.get_clock().now().to_msg()
        
        for mid, ((name, path), color) in enumerate(zip(paths, colors)):
            if not path.poses:
                continue
                
            marker = Marker()
            marker.header.frame_id = path.header.frame_id
            marker.header.stamp = now
            marker.ns = 'path_visualization'
            marker.id = mid
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.05  # Line width
            marker.color.r = color[0]
            marker.color.g = color[1] 
            marker.color.b = color[2]
            marker.color.a = 1.0
            
            for pose_stamped in path.poses:
                marker.points.append(pose_stamped.pose.position)
            
            markers.markers.append(marker)
        
        self._path_markers_pub.publish(markers)

        # Labels (TEXT_VIEW_FACING)
        labels = MarkerArray()
        for mid, (name, p) in enumerate([('PathA', path_A), ('PathB', path_B), ('PathC', path_C)], start=1):
            m = Marker()
            m.header.frame_id = p.header.frame_id
            m.header.stamp = now
            m.ns = 'path_labels'
            m.id = mid
            m.type = Marker.TEXT_VIEW_FACING
            m.action = Marker.ADD
            # ラベル位置：パスの始点少し上
            if p.poses:
                m.pose.position.x = p.poses[-1].pose.position.x + 0.1
                m.pose.position.y = p.poses[-1].pose.position.y + 0.1
            m.pose.position.z = 0.3
            m.scale.z = 0.25  # 文字サイズ
            m.color.r = 1.0; m.color.g = 1.0; m.color.b = 1.0; m.color.a = 1.0
            m.text = name
            labels.markers.append(m)
        self._label_pub.publish(labels)
        # self.get_logger().info("Published paths and labels with colored markers for RViz.")

    # ===== Robot trajectory =====
    def _on_odom(self, msg: Odometry):
        with self._traj_lock:
            if self._traj_frame_id != msg.header.frame_id:
                # フレームが変わったら合わせる
                self._traj_points = []
                self._traj_frame_id = msg.header.frame_id
            
            # 新しい点を追加（距離チェックで間引き）
            current_pos = msg.pose.pose.position
            if not self._traj_points or self._distance_2d(self._traj_points[-1], current_pos) > 0.05:
                self._traj_points.append(current_pos)
                
                # メモリ対策：必要に応じて上限
                if len(self._traj_points) > 5000:
                    self._traj_points = self._traj_points[-2000:]
                
                # LINE_STRIPマーカーとしてpublish（2点以上ある場合のみ）
                if len(self._traj_points) >= 2:
                    marker = Marker()
                    marker.header.frame_id = self._traj_frame_id
                    marker.header.stamp = msg.header.stamp
                    marker.ns = 'robot_trajectory'
                    marker.id = 0
                    marker.type = Marker.LINE_STRIP
                    marker.action = Marker.ADD
                    marker.scale.x = 0.03  # Line width
                    marker.color.r = 1.0
                    marker.color.g = 0.5
                    marker.color.b = 0.0  # Orange color
                    marker.color.a = 0.8
                    marker.points = self._traj_points.copy()
                    
                    self._traj_pub.publish(marker)
    
    def _distance_2d(self, p1, p2):
        dx = p1.x - p2.x
        dy = p1.y - p2.y
        return math.sqrt(dx*dx + dy*dy)

    def clear_trajectory(self):
        with self._traj_lock:
            self._traj_points = []
        
        # 空のマーカーをPublishしてRViz側の表示をクリア
        marker = Marker()
        marker.header.frame_id = self._traj_frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'robot_trajectory'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.DELETE
        self._traj_pub.publish(marker)
        self.get_logger().info("Cleared robot trajectory.")

    # ===== Initial pose =====
    def publish_initial_pose(self, x: float = 0.0, y: float = 0.0, yaw_rad: float = 0.0):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        _, _, qz, qw = yaw_to_quat(yaw_rad)
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        cov = [0.0] * 36
        cov[0] = 0.0
        cov[7] = 0.0
        cov[35] = 0.0
        msg.pose.covariance = cov

        self._initpose_pub.publish(msg)
        self.get_logger().info(f"Published initial pose at ({x:.2f}, {y:.2f}, yaw={yaw_rad:.2f} rad) in frame '{self._frame_id}'")

    def _auto_publish_initial_pose(self):
        time.sleep(0.5)
        for _ in range(3):
            self.publish_initial_pose(0.0, 0.0, 0.0)
            time.sleep(0.5)

    # ===== Gazebo warp =====
    def _ensure_gazebo_clients(self):
        if _SetEntityState and self._gz_entity_cli is None:
            self._gz_entity_cli = self.create_client(_SetEntityState, '/gazebo/set_entity_state')
        if _SetModelState and self._gz_model_cli is None:
            self._gz_model_cli = self.create_client(_SetModelState, '/gazebo/set_model_state')

    def warp_model(self, model_name: str = 'tb3', x: float = 0.0, y: float = 0.0, z: float = 0.0, yaw_rad: float = 0.0):
        self._ensure_gazebo_clients()
        _, _, qz, qw = yaw_to_quat(yaw_rad)

        def _log_result(api_name: str, fut):
            try:
                res = fut.result()
                if res is not None:
                    self.get_logger().info(f"Warped '{model_name}' via {api_name} to ({x:.2f},{y:.2f},{z:.2f})")
                else:
                    self.get_logger().warn(f"{api_name} returned None")
            except Exception as e:
                self.get_logger().warn(f"{api_name} failed: {e}")

        # まず /gazebo/set_entity_state を試す
        if self._gz_entity_cli is not None and self._gz_entity_cli.wait_for_service(timeout_sec=0.5):
            req = _SetEntityState.Request()
            state = _EntityState()
            state.name = model_name
            state.reference_frame = 'world'
            state.pose.position.x = float(x)
            state.pose.position.y = float(y)
            state.pose.position.z = float(z)
            state.pose.orientation.z = qz
            state.pose.orientation.w = qw
            req.state = state
            fut = self._gz_entity_cli.call_async(req)
            fut.add_done_callback(lambda f: _log_result('set_entity_state', f))
            return True

        # 次に /gazebo/set_model_state を試す
        if self._gz_model_cli is not None and self._gz_model_cli.wait_for_service(timeout_sec=0.5):
            req = _SetModelState.Request()
            ms = _ModelState()
            ms.model_name = model_name
            ms.reference_frame = 'world'
            ms.pose.position.x = float(x)
            ms.pose.position.y = float(y)
            ms.pose.position.z = float(z)
            ms.pose.orientation.z = qz
            ms.pose.orientation.w = qw
            req.model_state = ms
            fut = self._gz_model_cli.call_async(req)
            fut.add_done_callback(lambda f: _log_result('set_model_state', f))
            return True

        self.get_logger().error("Gazebo set_state services not available. Is Gazebo (Classic) running?")
        return False

    def _auto_warp_tb3(self):
        time.sleep(1.5)
        try:
            self.warp_model('tb3', 0.0, 0.0, 0.0, 0.0)
        except Exception as e:
            self.get_logger().warn(f"Auto-warp failed: {e}")

    def _periodic_path_publish(self):
        time.sleep(2.0)  # 初期化待ち
        path_A, path_B, path_C = make_path(self._frame_id)
        while True:
            try:
                self.publish_paths_and_labels(path_A, path_B, path_C)
                time.sleep(1.0)  # 1秒間隔で定期的にpublish
            except Exception as e:
                self.get_logger().warn(f"Periodic path publish failed: {e}")
                time.sleep(5.0)  # エラー時は少し長く待つ

    # ===== follow_path action =====
    def send_path(self, path_msg: Path, controller_id: str, goal_checker_id: str):
        if not self._client.wait_for_server(timeout_sec=2.0):
            raise RuntimeError("`/follow_path` action server not available.")

        goal = FollowPath.Goal()
        goal.path = path_msg
        goal.controller_id = controller_id
        goal.goal_checker_id = goal_checker_id

        send_future = self._client.send_goal_async(goal, feedback_callback=self._feedback_cb)

        def _goal_response_cb(fut):
            self._current_goal_handle = fut.result()
            if not self._current_goal_handle.accepted:
                self.get_logger().warn('Goal rejected by controller_server.')
                return
            self.get_logger().info(f'Goal accepted by controller "{controller_id}".')
            self._current_goal_handle.get_result_async().add_done_callback(self._result_cb)

        send_future.add_done_callback(_goal_response_cb)

    def cancel_current_goal(self):
        gh = self._current_goal_handle
        if gh is None:
            self.get_logger().info('No active goal to cancel.')
            return
        cancel_future = gh.cancel_goal_async()
        cancel_future.add_done_callback(lambda _: self.get_logger().info('Cancel request sent.'))

    def _feedback_cb(self, feedback_msg):
        self.get_logger().debug(f'Feedback: {feedback_msg}')

    def _result_cb(self, fut):
        result = fut.result().result
        status = fut.result().status
        self.get_logger().info(f'Result received. status={status}, result={result}')


class AppGUI:
    def __init__(self, node: FollowPathClient, paths_dict: dict, frame_id: str):
        self.node = node
        self.paths_dict = paths_dict
        self.frame_id = frame_id

        self.root = tk.Tk()
        self.root.title("FollowPath GUI (Nav2)")
        self.root.geometry("560x400")

        tk.Label(self.root, text=f"frame_id: {self.frame_id}").pack(pady=4)

        frm = tk.Frame(self.root); frm.pack(pady=4)

        tk.Label(frm, text="Controller:").grid(row=0, column=0, sticky="e")
        self.controller_var = tk.StringVar(value="PP")
        self.controller_cb = ttk.Combobox(frm, textvariable=self.controller_var,
                                          values=["PP", "APP", "RPP", "DWPP"], state="readonly", width=10)
        self.controller_cb.grid(row=0, column=1, padx=6)

        self.goal_checker_id = "goal_checker"
        tk.Label(frm, text=f"Goal checker: {self.goal_checker_id}").grid(row=1, column=0, columnspan=2, pady=4)

        # Buttons row
        btn_row = tk.Frame(self.root); btn_row.pack(pady=(6, 8))
        tk.Button(btn_row, text="Warp Robot (0,0,0)",   command=self._on_warp_tb3).grid(row=0, column=0, padx=6)
        tk.Button(btn_row, text="Clear Trajectory",     command=self._on_clear_traj).grid(row=0, column=1, padx=6)

        tk.Label(self.root, text="Paths").pack(pady=(6, 2))
        btns = tk.Frame(self.root); btns.pack()

        for i, name in enumerate(self.paths_dict.keys()):
            tk.Button(btns, text=name, width=18,
                      command=lambda n=name: self._on_send(n)).grid(row=i // 2, column=i % 2, padx=6, pady=6)

        tk.Button(self.root, text="Cancel Current", command=self._on_cancel).pack(pady=(8, 4))

        # 初回：PathとラベルをPublish
        self.node.publish_paths_and_labels(
            self.paths_dict["Path A (45 deg)"],
            self.paths_dict["Path B (90 deg)"],
            self.paths_dict["Path C (135 deg)"],
        )

    def _on_set_initial_pose(self):
        try:
            self.node.publish_initial_pose(0.0, 0.0, 0.0)
        except Exception as e:
            messagebox.showerror("Error", str(e))

    def _on_warp_tb3(self):
        try:
            ok = self.node.warp_model('tb3', 0.0, 0.0, 0.0, 0.0)
            if not ok:
                messagebox.showwarning("Warp", "Failed to warp tb3. Check Gazebo services.")
        except Exception as e:
            messagebox.showerror("Warp Error", str(e))

    def _on_clear_traj(self):
        try:
            self.node.clear_trajectory()
        except Exception as e:
            messagebox.showerror("Error", str(e))

    def _on_send(self, path_name: str):
        try:
            controller_id = self.controller_var.get()
            path_msg = self.paths_dict[path_name]
            self.node.get_logger().info(f"Sending path '{path_name}' using controller '{controller_id}'")
            self.node.send_path(path_msg, controller_id, self.goal_checker_id)
        except Exception as e:
            messagebox.showerror("Error", str(e))

    def _on_cancel(self):
        self.node.cancel_current_goal()

    def run(self):
        self.root.mainloop()


def main():
    rclpy.init()

    frame_id = "map"
    path_A, path_B, path_C = make_path(frame_id)

    paths = {
        "Path A (45 deg)": path_A,
        "Path B (90 deg)": path_B,
        "Path C (135 deg)": path_C,
    }

    node = FollowPathClient(frame_id=frame_id)
    # === 安全な executor / spin ===
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        gui = AppGUI(node, paths, frame_id)
        gui.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
