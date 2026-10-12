#!/usr/bin/env python3
"""Single-vehicle SITL tutorial: take off, visit two nearby points, land."""
import argparse
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.signals import SignalHandlerOptions
from flight_interfaces.action import Takeoff, ExecuteWaypoints, Land
from flight_interfaces.msg import ControlStatus, Waypoint


class FlightSequence(Node):
    def __init__(self, core):
        super().__init__('flight_sequence_example')
        self.state = None
        self.received_at = 0.0
        self.takeoff_requested = False
        self.last_feedback = None
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.subscription = self.create_subscription(
            ControlStatus, core + '/control_status', self.on_status, qos)
        self.action_clients = {
            'takeoff': ActionClient(self, Takeoff, core + '/takeoff'),
            'waypoints': ActionClient(self, ExecuteWaypoints, core + '/execute_waypoints'),
            'land': ActionClient(self, Land, core + '/land_action'),
        }

    def on_status(self, msg):
        self.state = msg
        self.received_at = time.monotonic()

    def fresh(self):
        if self.state is None:
            return False
        stamp = self.state.header.stamp
        age = self.get_clock().now().nanoseconds / 1e9 - (stamp.sec + stamp.nanosec / 1e9)
        return time.monotonic() - self.received_at < 1.0 and -0.1 <= age < 1.0

    def wait(self, predicate, timeout, label, monitor=False):
        deadline = time.monotonic() + timeout
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if monitor and (not self.fresh() or not self.state.telemetry_valid):
                raise RuntimeError(label + ': status stale or telemetry invalid')
            if predicate():
                return
            if time.monotonic() >= deadline:
                raise TimeoutError(label + ': client deadline exceeded; remote execution may continue')
        raise RuntimeError('ROS context stopped')

    def feedback(self, event):
        data = event.feedback
        key = (data.feedback.phase, getattr(data, 'current_index', None))
        if key != self.last_feedback:
            self.get_logger().info(f'feedback phase={key[0]}, waypoint_index={key[1]}')
            self.last_feedback = key

    def action(self, name, goal, timeout):
        client = self.action_clients[name]
        if not client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError(name + ': action server unavailable')
        self.last_feedback = None
        # A reply can be lost after the server starts executing: mark before send.
        if name == 'takeoff':
            self.takeoff_requested = True
        future = client.send_goal_async(goal, feedback_callback=self.feedback)
        self.wait(future.done, 10.0, name + ' acceptance', monitor=True)
        handle = future.result()
        if not handle.accepted:
            raise RuntimeError(name + ': goal rejected')
        future = handle.get_result_async()
        self.wait(future.done, timeout, name + ' result', monitor=True)
        response = future.result()
        result = response.result.result
        self.get_logger().info(
            f'{name}: status={response.status}, code={result.outcome_code}, {result.message}')
        if response.status != GoalStatus.STATUS_SUCCEEDED or result.outcome_code != 0:
            raise RuntimeError(name + ': action did not succeed')
        return response.result

    def run(self, height):
        self.wait(lambda: self.fresh() and self.state.telemetry_valid and
                  self.state.pose_valid and self.state.ground_state == 1 and
                  not self.state.armed, 30.0, 'ready on ground')
        self.action('takeoff', Takeoff.Goal(height_m=height), 75.0)
        self.wait(lambda: self.state.holding and self.state.pose_valid and
                  self.state.armed and self.state.ground_state == 2,
                  5.0, 'takeoff hold', monitor=True)
        p = self.state.current_pose.position
        q = self.state.current_pose.orientation
        yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
        goal = ExecuteWaypoints.Goal(
            frame_id='map', position_tolerance_m=0.15, check_yaw=True,
            yaw_tolerance_rad=math.radians(10), dwell_sec=0.5,
            waypoint_timeout_sec=60.0, loop=False)
        # Absolute ENU coordinates derived from the observed hold position.
        goal.waypoints = [Waypoint(x=p.x+1.0, y=p.y, z=p.z, yaw_rad=yaw),
                          Waypoint(x=p.x, y=p.y, z=p.z, yaw_rad=yaw)]
        result = self.action('waypoints', goal, 130.0)
        if result.completed_count != 2:
            raise RuntimeError('unexpected completed_count')
        self.get_logger().info('Two waypoints completed; landing next')

    def land(self):
        # Wait briefly for recovery; do not equate stale ground data with landed.
        self.wait(lambda: self.fresh() and self.state.telemetry_valid,
                  10.0, 'fresh status before landing')
        self.action('land', Land.Goal(), 75.0)
        self.wait(lambda: self.state.ground_state == 1 and not self.state.armed,
                  5.0, 'ground and disarmed', monitor=True)
        self.get_logger().info('Landing confirmed: ground_state=1, armed=false')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--core', default='/flight_core')
    parser.add_argument('--height', type=float, default=1.0)
    args = parser.parse_args()
    if not args.core.startswith('/') or not args.core.strip('/'):
        parser.error('--core must be an absolute node name, e.g. /flight_core')
    if not math.isfinite(args.height) or not 0 < args.height <= 10:
        parser.error('--height must be finite and in (0, 10] metres')
    # Keep the ROS context alive for best-effort landing after one Ctrl+C.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = FlightSequence(args.core.rstrip('/'))
    code = 0
    try:
        node.run(args.height)
    except (Exception, KeyboardInterrupt) as error:
        node.get_logger().error(f'Sequence stopped: {type(error).__name__}: {error}')
        code = 1
    finally:
        if node.takeoff_requested:
            try:
                node.land()
            except (Exception, KeyboardInterrupt) as error:
                node.get_logger().error(
                    f'Landing NOT confirmed: {type(error).__name__}: {error}; '
                    'inspect core/PX4 state and use land_action when available')
                code = 1
        node.destroy_node()
        rclpy.shutdown()
    return code


if __name__ == '__main__':
    raise SystemExit(main())
