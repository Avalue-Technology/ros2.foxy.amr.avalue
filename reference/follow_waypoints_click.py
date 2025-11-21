"""
/*
 *-------------------------------------------------------------------------------------------------------------------------------------->
 *	Name: follow_waypoints_click.py
 *-------------------------------------------------------------------------------------------------------------------------------------->
 *	Purpose: Listen rviz2 Waypoint until press ENTER to send whole waypoints to the navigation topic
 *-------------------------------------------------------------------------------------------------------------------------------------->
 *	Dependent Reference
 *-------------------------------------------------------------------------------------------------------------------------------------->
 *	(01)Python 3.8.10
 *-------------------------------------------------------------------------------------------------------------------------------------->
 *	(02)Please run: Start Navigation -> RViz2-NAV2 -> Follow Waypoints Click
 *--------------------------------------------------------------------------------------------------------------------------------------> 
 *	Known Issues
 *-------------------------------------------------------------------------------------------------------------------------------------->
 *	Methodology
 *-------------------------------------------------------------------------------------------------------------------------------------->
 *	References
 *-------------------------------------------------------------------------------------------------------------------------------------->
 *	MSDN documents
 *-------------------------------------------------------------------------------------------------------------------------------------->
 *	Internal documents
 *-------------------------------------------------------------------------------------------------------------------------------------->
 *	Internet documents
 *-------------------------------------------------------------------------------------------------------------------------------------->
*/
"""

#!/usr/bin/env python3
import os
import sys
import math
import json
import threading
import termios
import tty
import select
from typing import List, Optional

import rclpy
from rclpy.node import Node
import rclpy.logging
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time

from geometry_msgs.msg import Point, PointStamped, PoseStamped
from nav2_msgs.action import FollowWaypoints
from visualization_msgs.msg import Marker, MarkerArray

from tf2_ros import Buffer, TransformListener, TransformException

# MapAndWaypointConstants
MAP_RESOLUTION = 0.05                 # meters per cell (for reference)
WAYPOINT_ARROW_LENGTH_M = 0.20        # default arrow shaft length = 25 cm
NEAR_SKIP_THRESH_M = 0.20             # if robot is within this distance of first waypoint, skip it

# LoopConfig
LOOP_INTERVAL_MS = 5000               # per-lap delay after completion
# LOOP_FLAG is toggled by F6, default False
# When LOOP_FLAG=True and you press F7 (Load&Send), laps run in Ping-Pong order with per-lap delay

# StorageAndTopics
SAVE_PATH = os.path.expanduser("~/.follow_waypoints_click.json")
TOPIC_PREVIEW = "/waypoint_preview"
FRAME = "map"

# KeySequences
KEY_ENTER_CR = "\r"
KEY_ENTER_LF = "\n"
KEY_CTRL_C   = "\x03"
KEY_F6       = "\x1b[17~"  # Toggle loop
KEY_F7       = "\x1b[18~"  # Load & Send (single or loop)
KEY_F8       = "\x1b[19~"  # Save

# MathHelpers
def yaw_to_quat(yaw: float):
    """Convert yaw (rad) into a quaternion for 2D orientation."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))

def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Extract yaw from quaternion (assuming planar motion)."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)

# RawKeyReader
class RawKeyReader:
    """
    Read one key from stdin in raw mode.
    - Returns full escape sequence for function keys (e.g. '\x1b[18~' for F7)
    - Returns single-char string for normal keys (e.g. '\r' '\n' '\x03')
    """
    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)

    def __enter__(self):
        tty.setraw(self.fd)
        return self

    def __exit__(self, exc_type, exc, tb):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

    def readkey(self, timeout=0.02) -> str:
        ch = os.read(self.fd, 1).decode("utf-8", errors="ignore")
        if ch != "\x1b":
            return ch
        seq = ch
        while True:
            r, _, _ = select.select([self.fd], [], [], timeout)
            if not r:
                break
            seq += os.read(self.fd, 1).decode("utf-8", errors="ignore")
            if seq.endswith("~"):
                break
            if len(seq) >= 3 and seq.startswith("\x1bO"):
                break
        return seq

# WaypointCollector
class WaypointCollector(Node):
    """Collect RViz clicked points, preview them, and send to /FollowWaypoints."""

    def __init__(self):
        super().__init__('waypoint_collector')

        # LoopState
        self.LOOP_FLAG = False
        self._loop_timer = None                  # one-shot timer per lap
        self._loop_poses_raw: List[PoseStamped] = []
        self._goal_active = False
        self._pingpong_reverse_next = False      # False: forward, True: reverse next lap

        # SubscriptionsAndClients
        self.sub = self.create_subscription(PointStamped, '/clicked_point', self._OnClicked, 10)
        self.cli = ActionClient(self, FollowWaypoints, '/FollowWaypoints')
        self.vis_pub = self.create_publisher(MarkerArray, TOPIC_PREVIEW, 10)

        # TfListener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # PoseStorage
        self._poses: List[PoseStamped] = []
        self._lock = threading.Lock()

        # MarkerBookkeeping
        self._last_marker_count = 0

        # RequestFlags
        self._send_requested = False
        self._load_send_requested = False
        self._poll_timer = self.create_timer(0.1, self._PollRequests)

        # Banner
        self.get_logger().info("============================================================")
        self.get_logger().info("   Enter : Send current RViz waypoints")
        self.get_logger().info("       F6: Toggle LOOP_FLAG (Enable/Disable Loop, Ping-Pong)")
        self.get_logger().info(f"       F7: Load saved waypoints and send (Lap Delay {LOOP_INTERVAL_MS} ms)")
        self.get_logger().info("       F8: Save current RViz waypoints")
        self.get_logger().info("   Ctrl+C: Exit")
        self.get_logger().info("============================================================\n")

        # InitPreview
        self._PublishPreview()

    # RvizClickCallback
    def _OnClicked(self, msg: PointStamped):
        ps = PoseStamped()
        ps.header.frame_id = FRAME
        ps.pose.position.x = msg.point.x
        ps.pose.position.y = msg.point.y
        ps.pose.orientation.w = 1.0

        with self._lock:
            self._poses.append(ps)
            count = len(self._poses)

        self.get_logger().info(f"Point clicked: {msg.point.x:.3f}, {msg.point.y:.3f}  (collected: {count})")
        self._PublishPreview()

    # RobotPoseHelpers
    def _GetRobotPose(self) -> Optional[tuple]:
        """Return (x, y, yaw) of base_link in map frame, or None if TF unavailable."""
        try:
            tf = self.tf_buffer.lookup_transform(FRAME, 'base_link', Time())
            t = tf.transform.translation
            q = tf.transform.rotation
            yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
            return (t.x, t.y, yaw)
        except TransformException as e:
            self.get_logger().warn(f"TF unavailable for {FRAME}->base_link: {e}")
            return None

    def _GetRobotYaw(self) -> Optional[float]:
        p = self._GetRobotPose()
        return p[2] if p is not None else None

    # YawComputationForSend
    def _ComputeYawsForSend(self, poses: List[PoseStamped]) -> List[float]:
        """
        Yaws for Nav2:
          - First yaw = robot heading if available, else toward next
          - Middle yaws = direction to next
          - Last yaw = same as last segment
        """
        n = len(poses)
        if n == 0:
            return []
        if n == 1:
            yaw0 = self._GetRobotYaw()
            return [yaw0 if yaw0 is not None else 0.0]

        yaws: List[float] = []
        robot_yaw = self._GetRobotYaw()
        if robot_yaw is not None:
            yaws.append(robot_yaw)
        else:
            x0, y0 = poses[0].pose.position.x, poses[0].pose.position.y
            x1, y1 = poses[1].pose.position.x, poses[1].pose.position.y
            yaws.append(math.atan2(y1 - y0, x1 - x0))

        for i in range(1, n - 1):
            x0, y0 = poses[i].pose.position.x, poses[i].pose.position.y
            x1, y1 = poses[i + 1].pose.position.x, poses[i + 1].pose.position.y
            yaws.append(math.atan2(y1 - y0, x1 - x0))

        yaws.append(yaws[-1])
        return yaws

    # PoseOrientationApplier
    @staticmethod
    def _ApplyYawsToPoses(poses: List[PoseStamped], yaws: List[float]) -> List[PoseStamped]:
        out: List[PoseStamped] = []
        for ps, yaw in zip(poses, yaws):
            nx = PoseStamped()
            nx.header = ps.header
            nx.pose.position = ps.pose.position
            qx, qy, qz, qw = yaw_to_quat(yaw)
            nx.pose.orientation.x = qx
            nx.pose.orientation.y = qy
            nx.pose.orientation.z = qz
            nx.pose.orientation.w = qw
            out.append(nx)
        return out

    # RouteBuilders
    @staticmethod
    def _CopyPoses(src: List[PoseStamped]) -> List[PoseStamped]:
        out: List[PoseStamped] = []
        for p in src:
            q = PoseStamped()
            q.header = p.header
            # deep copy numeric fields to be safe
            q.pose.position.x = float(p.pose.position.x)
            q.pose.position.y = float(p.pose.position.y)
            q.pose.position.z = float(getattr(p.pose.position, "z", 0.0))
            q.pose.orientation = p.pose.orientation
            out.append(q)
        return out

    @staticmethod
    def _ReversePoses(src: List[PoseStamped]) -> List[PoseStamped]:
        return list(reversed(WaypointCollector._CopyPoses(src)))

    def _BuildLapPoses(self, base: List[PoseStamped], forward: bool) -> List[PoseStamped]:
        """Create the list of poses for this lap in forward or reverse order.
           If robot is already near the first pose, skip it to avoid dithering at the end."""
        route = self._CopyPoses(base) if forward else self._ReversePoses(base)
        if not route:
            return route

        rp = self._GetRobotPose()
        if rp is not None:
            rx, ry, _ = rp
            dx = route[0].pose.position.x - rx
            dy = route[0].pose.position.y - ry
            if math.hypot(dx, dy) < NEAR_SKIP_THRESH_M:
                # Skip the first point because we are already at it
                route = route[1:]
                if not route:
                    return []
        return route

    # PreviewRule RobotToFirstPrevToCurrent
    def _PublishPreview(self):
        with self._lock:
            poses = list(self._poses)

        ma = MarkerArray()
        now = self.get_clock().now().to_msg()

        # PathLine
        line = Marker()
        line.header.frame_id = FRAME
        line.header.stamp = now
        line.ns = 'waypoints'
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.03
        line.color.r = 0.0
        line.color.g = 1.0
        line.color.b = 0.0
        line.color.a = 1.0
        line.pose.orientation.w = 1.0
        line.points = [Point(x=p.pose.position.x, y=p.pose.position.y, z=0.0) for p in poses]
        ma.markers.append(line)

        # Arrows
        n = len(poses)
        for i in range(n):
            arrow = Marker()
            arrow.header.frame_id = FRAME
            arrow.header.stamp = now
            arrow.ns = 'waypoints'
            arrow.id = 100 + i
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.scale.x = WAYPOINT_ARROW_LENGTH_M
            arrow.scale.y = 0.08
            arrow.scale.z = 0.12
            arrow.color.r = 1.0
            arrow.color.g = 0.6
            arrow.color.b = 0.0
            arrow.color.a = 0.95

            if i == 0:
                robot_pose = self._GetRobotPose()
                if robot_pose is None:
                    arrow.pose.position = poses[0].pose.position
                    yaw = 0.0
                else:
                    rx, ry, _ = robot_pose
                    arrow.pose.position.x = rx
                    arrow.pose.position.y = ry
                    curr = poses[0].pose.position
                    yaw = math.atan2(curr.y - ry, curr.x - rx)
            else:
                prev = poses[i - 1].pose.position
                curr = poses[i].pose.position
                arrow.pose.position = prev
                yaw = math.atan2(curr.y - prev.y, curr.x - prev.x)

            qx, qy, qz, qw = yaw_to_quat(yaw)
            arrow.pose.orientation.x = qx
            arrow.pose.orientation.y = qy
            arrow.pose.orientation.z = qz
            arrow.pose.orientation.w = qw
            ma.markers.append(arrow)

        self._last_marker_count = n
        self.vis_pub.publish(ma)

    # PreviewCleaner
    def _ClearPreview(self):
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()

        m_line = Marker()
        m_line.header.frame_id = FRAME
        m_line.header.stamp = now
        m_line.ns = 'waypoints'
        m_line.id = 0
        m_line.action = Marker.DELETE
        ma.markers.append(m_line)

        for i in range(self._last_marker_count):
            m = Marker()
            m.header.frame_id = FRAME
            m.header.stamp = now
            m.ns = 'waypoints'
            m.id = 100 + i
            m.action = Marker.DELETE
            ma.markers.append(m)

        self._last_marker_count = 0
        self.vis_pub.publish(ma)

    # PublicApiKeys
    def request_send(self):
        self._send_requested = True

    def request_load_and_send(self):
        self._load_send_requested = True

    # PollRequests
    def _PollRequests(self):
        if self._send_requested:
            self._send_requested = False
            self._SendCurrentWaypointsAsync()
        if self._load_send_requested:
            self._load_send_requested = False
            self._LoadAndMaybeLoop()

    # SaveLoadHelpers
    def _SaveCurrentWaypoints(self):
        with self._lock:
            poses = list(self._poses)
        if len(poses) == 0:
            self.get_logger().warn("No waypoints to save.")
            return
        data = [{"x": p.pose.position.x, "y": p.pose.position.y} for p in poses]
        try:
            with open(SAVE_PATH, "w") as f:
                json.dump(data, f, indent=2)
            self.get_logger().info(f"Saved {len(poses)} waypoints to {SAVE_PATH}")
        except Exception as e:
            self.get_logger().error(f"Failed to save waypoints: {e}")

    def _LoadPositions(self) -> Optional[List[PoseStamped]]:
        if not os.path.exists(SAVE_PATH):
            self.get_logger().warn(f"No saved file found at {SAVE_PATH}")
            return None
        try:
            with open(SAVE_PATH, "r") as f:
                arr = json.load(f)
            poses: List[PoseStamped] = []
            for item in arr:
                ps = PoseStamped()
                ps.header.frame_id = FRAME
                ps.pose.position.x = float(item["x"])
                ps.pose.position.y = float(item["y"])
                ps.pose.orientation.w = 1.0
                poses.append(ps)
            return poses
        except Exception as e:
            self.get_logger().error(f"Failed to load waypoints: {e}")
            return None

    # CoreSendHelpers
    def _SendPosesAsync(self, base_poses: List[PoseStamped]):
        if not base_poses:
            self.get_logger().warn("No waypoints to send.")
            return
        if not self.cli.wait_for_server(timeout_sec=0.1):
            self.get_logger().error("No FollowWaypoints server! Is nav2_waypoint_follower running?")
            return

        yaws = self._ComputeYawsForSend(base_poses)
        poses_with_yaw = self._ApplyYawsToPoses(base_poses, yaws)

        goal = FollowWaypoints.Goal()
        goal.poses = poses_with_yaw

        self._goal_active = True
        self.get_logger().info(f"Sending {len(poses_with_yaw)} waypoints ...")
        send_future = self.cli.send_goal_async(goal)
        send_future.add_done_callback(self._OnGoalResponse)

    def _SendCurrentWaypointsAsync(self):
        with self._lock:
            base_poses = list(self._poses)
        self._SendPosesAsync(base_poses)

    # LoopScheduling
    def _ScheduleNextLoop(self):
        """Create a one-shot timer to resend after LOOP_INTERVAL_MS from the end of the previous lap."""
        self._StopLoopTimer()
        period = float(LOOP_INTERVAL_MS) / 1000.0

        def _cb():
            try:
                self._loop_timer.cancel()
            except Exception:
                pass
            self._loop_timer = None

            if not self.LOOP_FLAG or self._goal_active or not self._loop_poses_raw:
                return

            # Ping-Pong: alternate between forward and reverse lists, with near-first skipping
            if self._pingpong_reverse_next:
                route = self._BuildLapPoses(self._loop_poses_raw, forward=False)
                mode = "REVERSE"
            else:
                route = self._BuildLapPoses(self._loop_poses_raw, forward=True)
                mode = "FORWARD"

            if not route:
                self.get_logger().warn("No valid route to send in loop (possibly only one waypoint and already at it).")
                return

            self.get_logger().info(f"Loop delay elapsed ({LOOP_INTERVAL_MS} ms). Re-sending waypoints in {mode} order ...")
            self._pingpong_reverse_next = not self._pingpong_reverse_next
            self._SendPosesAsync(route)

        self._loop_timer = self.create_timer(period, _cb)

    def _StopLoopTimer(self):
        if self._loop_timer is not None:
            try:
                self._loop_timer.cancel()
            except Exception:
                pass
            self._loop_timer = None

    # LoadAndMaybeLoop
    def _LoadAndMaybeLoop(self):
        loaded = self._LoadPositions()
        if not loaded:
            return

        with self._lock:
            self._poses = loaded
        self._PublishPreview()

        self._loop_poses_raw = loaded
        self._StopLoopTimer()
        self._pingpong_reverse_next = True  # first lap forward, next lap reverse

        if self.LOOP_FLAG:
            route = self._BuildLapPoses(self._loop_poses_raw, forward=True)
            if not route:
                self.get_logger().warn("Loaded route is empty after near-first filtering; nothing to send.")
                return
            self.get_logger().info(f"LOOP_FLAG=True: First lap FORWARD starts now, next lap will be REVERSE after {LOOP_INTERVAL_MS} ms.")
            self._SendPosesAsync(route)
        else:
            route = self._BuildLapPoses(self._loop_poses_raw, forward=True)
            if not route:
                self.get_logger().warn("Loaded route is empty after near-first filtering; nothing to send.")
                return
            self._SendPosesAsync(route)

    # ActionCallbacks
    def _OnGoalResponse(self, future):
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self._goal_active = False
            self.get_logger().error("FollowWaypoints goal rejected.")
            return
        self.get_logger().info("Goal accepted. Waiting for result ...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._OnResult)

    def _OnResult(self, future):
        try:
            result = future.result().result
        except Exception as e:
            self._goal_active = False
            self.get_logger().error(f"Failed to get result: {e}")
            return

        self._goal_active = False

        if hasattr(result, "missed_waypoints") and result.missed_waypoints:
            missed = list(result.missed_waypoints)
            self.get_logger().warn(f"Missed waypoints: {missed}")
        else:
            self.get_logger().info("All waypoints completed successfully!")

        if self.LOOP_FLAG and self._loop_poses_raw:
            self._ScheduleNextLoop()
            return

        with self._lock:
            self._poses.clear()
        self._ClearPreview()
        self.get_logger().info("Ready for new waypoints. Click in RViz and press Enter again.")

# Main
def main():
    rclpy.init()
    node = WaypointCollector()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        with RawKeyReader() as kr:
            while True:
                key = kr.readkey()
                if key in (KEY_ENTER_CR, KEY_ENTER_LF):
                    node.request_send()
                elif key == KEY_F8:
                    node.get_logger().info("Saving current RViz waypoints ...")
                    node._SaveCurrentWaypoints()
                elif key == KEY_F7:
                    node.get_logger().info("Loading saved waypoints and sending ...")
                    node.request_load_and_send()
                elif key == KEY_F6:
                    node.LOOP_FLAG = not node.LOOP_FLAG
                    node.get_logger().info(f"LOOP_FLAG: {'TRUE' if node.LOOP_FLAG else 'FALSE'}")
                    if not node.LOOP_FLAG:
                        node._StopLoopTimer()
                elif key == KEY_CTRL_C:
                    node.get_logger().info("Ctrl+C pressed, exiting ...")
                    break
    except KeyboardInterrupt:
        pass
    finally:
        node._StopLoopTimer()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=0.5)

if __name__ == '__main__':
    main()
