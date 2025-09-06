#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from nav2_msgs.action import FollowPath


def yaw_to_quat(z_yaw_rad: float):
    half = z_yaw_rad * 0.5
    qz = math.sin(half)
    qw = math.cos(half)
    return (0.0, 0.0, qz, qw)


def make_path(frame_id: str, xy_yaw_seq):
    path = Path()
    path.header.frame_id = frame_id
    for (x, y, yaw_val) in xy_yaw_seq:
        yaw = math.radians(yaw_val) if abs(yaw_val) > 2.0 * math.pi else yaw_val
        ps = PoseStamped()
        ps.header.frame_id = frame_id
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        _, _, qz, qw = yaw_to_quat(yaw)
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        path.poses.append(ps)
    return path


class FollowPathClient(Node):
    def __init__(self, frame_id: str = "map"):
        super().__init__('follow_path_gui_client')
        self._client = ActionClient(self, FollowPath, '/follow_path')
        self._current_goal_handle = None
        self._frame_id = frame_id

        # --- initial pose publisher (same as RViz "2D Pose Estimate") ---
        self._initpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 'initialpose', 10
        )

        # 起動直後に初期姿勢を複数回送る（AMCLが確実に受け取れるように）
        threading.Thread(target=self._auto_publish_initial_pose, daemon=True).start()

    # ---- initial pose helpers ----
    def publish_initial_pose(self, x: float = 0.0, y: float = 0.0, yaw_rad: float = 0.0):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        _, _, qz, qw = yaw_to_quat(yaw_rad)
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        # 6x6 共分散（x, y, z, roll, pitch, yaw）
        # 位置は 0.5m 程度、yaw は ~0.3rad^2 程度の緩めの分散に設定（必要に応じて調整）
        cov = [0.0] * 36
        cov[0] = 0.25   # var(x)
        cov[7] = 0.25   # var(y)
        cov[35] = 0.09  # var(yaw) = (0.3 rad)^2
        msg.pose.covariance = cov

        self._initpose_pub.publish(msg)
        self.get_logger().info(f"Published initial pose at ({x:.2f}, {y:.2f}, yaw={yaw_rad:.2f} rad) in frame '{self._frame_id}'")

    def _auto_publish_initial_pose(self):
        # 少し待ってから複数回 publish（AMCL購読準備や遅延対策）
        time.sleep(0.5)
        for _ in range(3):
            self.publish_initial_pose(0.0, 0.0, 0.0)
            time.sleep(0.5)

    # ---- follow_path action ----
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
        self.root.geometry("460x320")

        tk.Label(self.root, text=f"frame_id: {self.frame_id}").pack(pady=4)

        frm = tk.Frame(self.root)
        frm.pack(pady=4)

        tk.Label(frm, text="Controller:").grid(row=0, column=0, sticky="e")
        self.controller_var = tk.StringVar(value="PP")
        self.controller_cb = ttk.Combobox(frm, textvariable=self.controller_var,
                                          values=["PP", "APP", "RPP", "DWPP"], state="readonly", width=10)
        self.controller_cb.grid(row=0, column=1, padx=6)

        self.goal_checker_id = "goal_checker"
        tk.Label(frm, text=f"Goal checker: {self.goal_checker_id}").grid(row=1, column=0, columnspan=2, pady=4)

        # 初期姿勢を明示的に再送したい時のボタン
        tk.Button(self.root, text="Set Initial Pose (0,0,0)", command=self._on_set_initial_pose).pack(pady=(4, 6))

        tk.Label(self.root, text="Paths").pack(pady=(6, 2))
        btns = tk.Frame(self.root)
        btns.pack()

        for i, name in enumerate(self.paths_dict.keys()):
            tk.Button(btns, text=name, width=18,
                      command=lambda n=name: self._on_send(n)).grid(row=i // 2, column=i % 2, padx=6, pady=6)

        tk.Button(self.root, text="Cancel Current", command=self._on_cancel).pack(pady=(8, 4))

        tk.Label(self.root, text="Tips: AMCLが起動していることを確認。必要なら初期姿勢を再送してください。").pack(pady=(4, 2))

    def _on_set_initial_pose(self):
        try:
            self.node.publish_initial_pose(0.0, 0.0, 0.0)
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
    path_A = make_path(frame_id, [(0.0, 0.0, 0.0),
                                  (1.0, 0.0, 0.0),
                                  (2.0, 0.5, 15.0),
                                  (3.0, 1.2, 25.0),
                                  (4.0, 2.0, 35.0)])
    path_B = make_path(frame_id, [(0.0, 0.0, 0.0),
                                  (1.0, 0.0, 0.0),
                                  (2.0, 0.0, 0.0),
                                  (2.0, 1.0, 90.0),
                                  (2.0, 2.0, 90.0)])
    path_C = make_path(frame_id, [(0.2 * i, 0.3 * math.sin(0.5 * i), 0.0) for i in range(0, 26)])

    paths = {
        "Path A (Curve)": path_A,
        "Path B (L-shape)": path_B,
        "Path C (Sine)": path_C,
    }

    node = FollowPathClient(frame_id=frame_id)
    executor_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    executor_thread.start()

    try:
        gui = AppGUI(node, paths, frame_id)
        gui.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
