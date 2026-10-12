#!/usr/bin/env python3
"""Isolated fake-MAVROS integration. Never attaches to the live vehicle domain."""
import os
import signal
import subprocess
import tempfile
import time
import uuid

os.environ['ROS_DOMAIN_ID'] = '184'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, ExtendedState
from mavros_msgs.srv import SetMode, CommandBool
from std_srvs.srv import Trigger
from flight_interfaces.action import Arm, Disarm, Takeoff, Land, ExecuteWaypoints
from flight_interfaces.msg import Waypoint
from flight_interfaces.msg import ControlStatus


def main():
    rclpy.init()
    ns = '/actions_test_' + uuid.uuid4().hex[:8]
    node = rclpy.create_node('mock', namespace=ns)
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    state_pub = node.create_publisher(State, ns+'/mavros/state', qos)
    ext_pub = node.create_publisher(ExtendedState, ns+'/mavros/extended_state', qos)
    pose_pub = node.create_publisher(PoseStamped, ns+'/mavros/local_position/pose', 10)
    mode, armed, z, target = 'OFFBOARD', False, 0.0, 0.0
    pending = []
    suppress_ground_confirmation = False
    setpoints, commands, statuses = [], [], []

    def setpoint(msg):
        nonlocal target
        target = msg.pose.position.z
        setpoints.append(time.monotonic())

    def mode_call(req, res):
        commands.append(('mode', req.custom_mode))
        if not (suppress_ground_confirmation and req.custom_mode == 'AUTO.LOITER'):
            pending.append((time.monotonic()+0.25, 'mode', req.custom_mode))
        res.mode_sent = True
        return res

    def arm_call(req, res):
        commands.append(('arm', req.value))
        pending.append((time.monotonic()+0.25, 'arm', req.value))
        res.success, res.result = True, 0
        return res

    node.create_subscription(PoseStamped, ns+'/mavros/setpoint_position/local', setpoint, 10)
    node.create_subscription(ControlStatus, ns+'/flight_core/control_status', statuses.append, qos)
    node.create_service(SetMode, ns+'/mavros/set_mode', mode_call)
    node.create_service(CommandBool, ns+'/mavros/cmd/arming', arm_call)

    def telemetry():
        nonlocal mode, armed, z
        now = time.monotonic()
        for event in list(pending):
            if now >= event[0]:
                if event[1] == 'mode': mode = event[2]
                else: armed = event[2]
                pending.remove(event)
        if mode == 'OFFBOARD' and armed:
            z += max(-0.08, min(0.08, target-z))
        if mode == 'AUTO.LAND':
            z = max(0.0, z-0.08)
            if z <= 0.001: armed = False
        stamp = node.get_clock().now().to_msg()
        s = State(); s.header.stamp = stamp; s.connected = True; s.armed = armed; s.mode = mode
        e = ExtendedState(); e.header.stamp = stamp; e.landed_state = 2 if z > 0.02 else 1
        p = PoseStamped(); p.header.stamp = stamp; p.header.frame_id = 'map'
        p.pose.position.z = z; p.pose.orientation.w = 1.0
        state_pub.publish(s); ext_pub.publish(e); pose_pub.publish(p)
    node.create_timer(0.05, telemetry)

    def until(predicate, timeout=15):
        end = time.monotonic()+timeout
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.02)
            if predicate(): return
        raise AssertionError('timeout; last status: '+str(statuses[-1] if statuses else None))

    clients = {name: ActionClient(node, cls, ns+'/flight_core/'+name)
               for name, cls in [('arm', Arm), ('disarm', Disarm), ('takeoff', Takeoff), ('land_action', Land), ('execute_waypoints', ExecuteWaypoints)]}
    def send(name, goal):
        until(lambda: clients[name].server_is_ready())
        f = clients[name].send_goal_async(goal); until(f.done)
        handle = f.result(); assert handle.accepted, name+' rejected'
        return handle, handle.get_result_async()
    def complete(result, code=0):
        until(result.done)
        assert result.result().result.result.outcome_code == code, result.result()

    with tempfile.TemporaryFile(mode='w+') as log:
        process = subprocess.Popen(['ros2', 'run', 'flight_core', 'flight_actions_node',
            '--ros-args', '-r', '__ns:='+ns, '-p', 'mavros_namespace:='+ns+'/mavros'],
            stdout=log, stderr=log, start_new_session=True)
        try:
            until(lambda: statuses and statuses[-1].pose_valid)
            assert not commands, 'idle sent commands'
            # A second action node must not acquire the same endpoint.
            second = subprocess.run(['ros2', 'run', 'flight_core', 'flight_actions_node',
                '--ros-args', '-r', '__ns:='+ns, '-p', 'mavros_namespace:='+ns+'/mavros'],
                capture_output=True, text=True, timeout=10)
            assert second.returncode != 0 and 'another flight_core' in second.stderr+second.stdout
            # Resume mock telemetry after the blocking exclusion check.
            old_count = len(statuses)
            until(lambda: len(statuses) >= old_count+5 and statuses[-1].pose_valid)
            # An accepted mode request without telemetry confirmation must never arm.
            suppress_ground_confirmation = True
            _, result = send('arm', Arm.Goal()); complete(result, 5)
            assert commands.count(('mode', 'AUTO.LOITER')) == 3
            assert not any(c[0] == 'arm' for c in commands)
            suppress_ground_confirmation = False
            _, result = send('arm', Arm.Goal()); complete(result)
            assert armed
            assert ('mode', 'AUTO.LOITER') in commands, 'residual OFFBOARD not reset'
            _, result = send('disarm', Disarm.Goal()); complete(result)
            assert not armed
            mode = 'OFFBOARD'
            until(lambda: statuses[-1].backend_mode == mode)
            takeoff_goal = Takeoff.Goal(); takeoff_goal.height_m = 1.0
            start = len(setpoints)
            _, result = send('takeoff', takeoff_goal); complete(result)
            until(lambda: statuses[-1].holding)
            assert abs(z-1) <= 0.1 and armed
            gaps = [b-a for a,b in zip(setpoints[start:], setpoints[start+1:])]
            assert gaps and max(gaps) < 0.25, gaps
            # Closing the completed client's handle does not stop normal hold.
            count = len(setpoints); until(lambda: len(setpoints) >= count+10)
            # Two airborne points, including descent: distinct from takeoff(height_m).
            mission = ExecuteWaypoints.Goal()
            mission.waypoints = [Waypoint(x=0.0,y=0.0,z=0.5,yaw_rad=0.0),
                                 Waypoint(x=0.0,y=0.0,z=1.0,yaw_rad=0.0)]
            _, result = send('execute_waypoints', mission); complete(result)
            assert result.result().result.completed_count == 2
            assert abs(z-1.0) < 0.1
            # A loop keeps executing until canceled, then holds the current pose.
            mission.loop = True
            handle, result = send('execute_waypoints', mission)
            until(lambda: statuses[-1].mission_active)
            cancel = handle.cancel_goal_async(); until(cancel.done)
            assert cancel.result().goals_canceling
            complete(result, 8)
            until(lambda: statuses[-1].holding)
            # The stop service uses the same snapshot hold, without a ROS cancel request.
            _, stopped_result = send('execute_waypoints', mission)
            until(lambda: statuses[-1].mission_active)
            stop_client = node.create_client(Trigger, ns+'/flight_core/stop')
            until(stop_client.service_is_ready)
            stop_result = stop_client.call_async(Trigger.Request()); until(stop_result.done)
            assert stop_result.result().success
            complete(stopped_result, 8)
            until(lambda: statuses[-1].holding)
            node.destroy_client(stop_client)
            # Invalid full-list submission must preserve the existing hold target.
            bad = ExecuteWaypoints.Goal()
            bad.waypoints = [Waypoint(x=30.0, y=0.0, z=1.0, yaw_rad=0.0)]
            _, bad_result = send('execute_waypoints', bad); complete(bad_result, 3)
            assert statuses[-1].holding
            # Disable yaw arrival test without changing the yaw command.
            mission.loop = False; mission.check_yaw = False
            mission.waypoints = [Waypoint(x=0.0,y=0.0,z=1.0,yaw_rad=1.0)]
            _, result = send('execute_waypoints', mission); complete(result)
            _, result = send('disarm', Disarm.Goal()); complete(result, 3)
            # Land preempts a running waypoint loop without competing streams.
            mission.loop = True
            _, mission_result = send('execute_waypoints', mission)
            until(lambda: statuses[-1].mission_active)
            _, result = send('land_action', Land.Goal()); complete(result)
            complete(mission_result, 8)
            assert not armed and z < 0.02
            handle, result = send('takeoff', takeoff_goal)
            # Cancel during prestream; there must be no late arm request.
            cancel = handle.cancel_goal_async(); until(cancel.done)
            assert cancel.result().goals_canceling
            complete(result, 8)
            assert not armed
            _, takeoff_result = send('takeoff', takeoff_goal)
            until(lambda: statuses[-1].control_active and armed)
            _, land_result = send('land_action', Land.Goal())
            complete(takeoff_result, 8); complete(land_result)
            assert not armed
            _, result = send('takeoff', takeoff_goal); complete(result)
            until(lambda: statuses[-1].holding)
            # Yaw mismatch must time out, hold briefly, then autonomously land.
            timeout_goal = ExecuteWaypoints.Goal()
            timeout_goal.waypoints = [Waypoint(x=0.0,y=0.0,z=1.0,yaw_rad=1.0)]
            timeout_goal.waypoint_timeout_sec = 0.8
            timeout_goal.dwell_sec = 0.1
            _, result = send('execute_waypoints', timeout_goal); complete(result, 6)
            until(lambda: not armed and statuses[-1].operation == 'land' and statuses[-1].phase == 'succeeded')
            _, result = send('takeoff', takeoff_goal); complete(result)
            until(lambda: statuses[-1].holding)
            mode = 'POSCTL'
            until(lambda: statuses[-1].phase == 'failed' and not statuses[-1].holding)
            command_count = len(commands); before = time.monotonic()
            until(lambda: time.monotonic()-before > 0.4)
            assert len(commands) == command_count, 'unexpected mode reclaim'
            # Permit already queued setpoints to drain, then check that publication stops.
            count = len(setpoints); before = time.monotonic()
            until(lambda: time.monotonic()-before > 0.4)
            assert len(setpoints) == count, 'stream continued after control loss'
            # Confirm AUTO.LAND, then simulate a mode change on touchdown/disarm.
            # Even inside the cleanup window this is completion, not control loss.
            for touchdown_mode in ('AUTO.LOITER', 'OFFBOARD'):
                mode, armed, z = 'AUTO.LAND', True, 4.0
                until(lambda: statuses[-1].backend_mode == 'AUTO.LAND' and statuses[-1].armed)
                _, result = send('land_action', Land.Goal())
                until(lambda: statuses[-1].operation == 'land' and statuses[-1].phase == 'executing')
                before = time.monotonic()
                until(lambda: time.monotonic()-before > 0.15)
                mode, armed, z = touchdown_mode, False, 0.0
                command_count = len(commands)
                complete(result)
                assert len(commands) == command_count, 'reclaimed mode after touchdown'
                until(lambda: statuses[-1].phase == 'succeeded')
            # Mode loss must still fail when airborne OR grounded but armed.
            for remaining_z in (2.0, 0.0):
                mode, armed, z = 'AUTO.LAND', True, 4.0
                until(lambda: statuses[-1].backend_mode == 'AUTO.LAND' and statuses[-1].armed)
                _, result = send('land_action', Land.Goal())
                until(lambda: statuses[-1].operation == 'land' and statuses[-1].phase == 'executing')
                before = time.monotonic()
                until(lambda: time.monotonic()-before > 0.15)
                mode, armed, z = 'POSCTL', True, remaining_z
                command_count = len(commands)
                complete(result, 5)
                assert 'landing mode changed externally' in result.result().result.result.message
                assert len(commands) == command_count, 'reclaimed external landing mode'
            print(f'PASS: residual-mode reset/confirmation gate/repeated flights, arm/disarm, takeoff/hold, airborne-disarm refusal, land, cancel, '
                  f'landing preemption/post-touchdown mode changes/airborne and armed mode-loss protection, waypoint descent/list/loop/cancel/yaw-option/preemption, control-loss hold stop, local exclusion; max setpoint gap={max(gaps):.3f}s')
        except Exception:
            log.seek(0); print(log.read()); raise
        finally:
            os.killpg(process.pid, signal.SIGINT)
            try: process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL); process.wait()
            for client in clients.values(): client.destroy()
            node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
