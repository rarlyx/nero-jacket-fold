#!/usr/bin/env python3
"""
Scripted jacket folding: 9 captured joint-space waypoints (starting and
ending at pose_90deg), sent directly as FollowJointTrajectory goals to
arm_controller and gripper_controller. Same proven approach as
agx_carrot_demo - no MoveItPy, no motion planning, since the exact joint
targets are already known from teaching each pose.

Requires move_group (or the real-arm moveit launch) to already be running,
since that's what starts ros2_control and these controllers.

Exposes /start_jacket_sequence and /stop_jacket_sequence (std_srvs/Trigger).
Stop cancels the currently executing trajectory goal immediately.
"""

import threading

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from std_srvs.srv import Trigger
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]
GRIPPER_JOINTS = ["gripper"]

POSE_90DEG = [-3.490658503988659e-05, -0.0063006385996995295, -5.235987755982989e-05,
              1.5673580281684678, -0.07588691587671345,
              0.0025656340004316645, 0.040317105721069016]
POSE_90DEG_GRIPPER = 0.000195

# (name, arm_positions, gripper_position, seconds_to_reach)
SEQUENCE = [
    ("pose_90deg_start", POSE_90DEG, POSE_90DEG_GRIPPER, 3.0),
    ("state1", [-0.5213298475707062, -0.006213372137099813, 0.0,
                1.567305668290908, -0.07597418233931316,
                0.002530727415391778, 0.04028219913602912], 0.00018099999999999998, 3.0),
    ("state2", [-0.9148841406029077, 0.9236631467404391, 0.18983946273942323,
                1.4297737232337548, -0.026581364507873642,
                -0.6175847558181935, 0.11751301853677822], 0.099675, 3.0),
    ("state3", [-0.8361174314604035, 0.9236631467404391, 0.19001399566462265,
                1.4297737232337548, -0.026494098045273924,
                -0.7299490530615884, 0.12562879955855186], 0.099675, 2.0),
    ("state4", [-0.8360825248753637, 0.9236980533254789, 0.19001399566462265,
                1.4297737232337548, -0.026546457922833756,
                -0.7298443333064688, 0.1256811594361117], 0.001295, 2.0),
    ("state5", [-0.8361697913379633, 0.670712578248901, 0.18985691603194316,
                1.429738816648715, -0.040107666210829694,
                0.12833405989914304, 0.8681791298195394], 0.001269, 3.0),
    ("state6", [0.6493148416194505, 0.5504244862014517, 0.20008454544862994,
                1.252448271231131, -0.039968039870670144,
                0.12824679343654333, 0.8681965831120593], 0.001207, 3.0),
    ("state7", [0.6492275751568507, 0.8795237699575025, 0.20001473227855018,
                1.1378150459601435, -0.03991567999311032,
                0.12829915331410316, 0.8681616765270195], 0.001207, 2.0),
    ("state8", [0.6492450284493706, 0.8795063166649826, 0.20001473227855018,
                1.1378324992526632, -0.039933133285630265,
                0.12826424672906328, 0.8681093166494597], 0.09966599999999999, 2.0),
    ("pose_90deg_end", POSE_90DEG, POSE_90DEG_GRIPPER, 3.0),
]


class JacketSequenceNode(Node):
    def __init__(self):
        super().__init__("jacket_sequence_node")

        self.arm_client = ActionClient(
            self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory"
        )
        self.gripper_client = ActionClient(
            self, FollowJointTrajectory, "/gripper_controller/follow_joint_trajectory"
        )

        self._stop_flag = threading.Event()
        self._running = False
        self._lock = threading.Lock()
        self._current_goal_handle = None

        cb_group = ReentrantCallbackGroup()
        self.create_service(Trigger, "start_jacket_sequence", self.start_cb, callback_group=cb_group)
        self.create_service(Trigger, "stop_jacket_sequence", self.stop_cb, callback_group=cb_group)

        self.get_logger().info("Waiting for arm_controller and gripper_controller action servers...")
        self.arm_client.wait_for_server()
        self.gripper_client.wait_for_server()
        self.get_logger().info(
            "Ready. Call /start_jacket_sequence and /stop_jacket_sequence (std_srvs/srv/Trigger)."
        )

    def start_cb(self, request, response):
        with self._lock:
            if self._running:
                response.success = False
                response.message = "Sequence already running"
                return response
            self._running = True

        self._stop_flag.clear()
        threading.Thread(target=self.run_sequence, daemon=True).start()
        response.success = True
        response.message = f"Sequence started ({len(SEQUENCE)} states)"
        return response

    def stop_cb(self, request, response):
        if not self._running:
            response.success = False
            response.message = "Nothing is running"
            return response

        self._stop_flag.set()
        if self._current_goal_handle is not None:
            self._current_goal_handle.cancel_goal_async()
        response.success = True
        response.message = "Stop requested, cancelling current motion"
        return response

    def _send_trajectory(self, client, joint_names, positions, seconds):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = joint_names
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(seconds=seconds).to_msg()
        goal.trajectory.points = [point]

        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        goal_handle = send_future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Trajectory goal rejected")
            return False

        self._current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=seconds + 10.0)
        self._current_goal_handle = None

        result = result_future.result()
        if result is None:
            self.get_logger().error("Trajectory execution timed out or was cancelled")
            return False
        return True

    def run_sequence(self):
        for state_name, arm_positions, gripper_position, seconds in SEQUENCE:
            if self._stop_flag.is_set():
                self.get_logger().info(f"Stopped before '{state_name}'")
                break

            self.get_logger().info(f"-> {state_name} (arm)")
            if not self._send_trajectory(self.arm_client, ARM_JOINTS, arm_positions, seconds):
                self.get_logger().error(f"Arm motion failed at '{state_name}', stopping")
                break

            if self._stop_flag.is_set():
                self.get_logger().info(f"Stopped after arm '{state_name}', before gripper")
                break

            self.get_logger().info(f"-> {state_name} (gripper)")
            if not self._send_trajectory(
                self.gripper_client, GRIPPER_JOINTS, [gripper_position], min(seconds, 2.0)
            ):
                self.get_logger().error(f"Gripper motion failed at '{state_name}', stopping")
                break

        with self._lock:
            self._running = False
        self.get_logger().info("Sequence finished (or stopped).")


def main():
    rclpy.init()
    node = JacketSequenceNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
