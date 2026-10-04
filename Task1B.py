#!/usr/bin/env python3


'''
*****************************************************************************************
*
*                       ===============================================
*                               StrataCobot (SC) Theme (eYRC 2026-27)
*                       ===============================================
*
*  This script should be used to implement Task 1B of StrataCobot (SC) Theme (eYRC 2026-27).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:          [ Team-ID ]
# Author List:          [ Names of team members worked on this file separated by Comma: Name1, Name2, ... ]
# Filename:                 task1b.py
# Functions:
#                               tcpposecb, jointstatecb, armstatuscb, switch_controller,
#                               send_twist, process_waypoints, main
# Nodes:
#                               Publishing Topics  - [ /delta_twist_cmds, /delta_joint_cmds ]
#                   Subscribing Topics - [ /tcp_pose_raw, /joint_states, /arm_status ]


################### IMPORT MODULES #######################

import rclpy
import sys
import math
import numpy as np
from scipy.spatial.transform import Rotation
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from control_msgs.msg import JointJog
from controller_manager_msgs.srv import SwitchController
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32, Float64MultiArray

##################### TASK CONSTANTS #######################

waypoints = [
    (-0.4085, -0.5379, 0.1967),   # waypoint 1
    (-0.8000, -0.0005, 0.3967),   # waypoint 2
    (-0.7430,  0.5280, 0.1967),   # waypoint 3
    (-0.4097,  0.5280, 0.1967),   # waypoint 4
    (-0.0763,  0.5280, 0.1967),   # waypoint 5
]

# Hold time (seconds) at each waypoint above; scored waypoints get a margin over 2 s.
hold_times = [2.5, 2.5, 2.5, 2.5, 2.5]

servo_ns = '/ur_arm_controller'
twist_controller = 'delta_twist_controller'
joint_controller = 'delta_joint_controller'

base_frame = 'base_link'

joint_names = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
]

cap_linear_mps = 0.120
cap_angular_rps = 0.250

command_timeout_s = 0.15

# Own limits, kept safely below the servo caps.
max_linear_speed = 0.04
max_angular_speed = 0.10


##################### CLASS DEFINITION #######################

class arm_waypoints(Node):
    '''
    ___CLASS___

    Description:    Class which serves the purpose to drive the UR7e's tool through the
                    given waypoints using the arm's velocity command interfaces.
    '''

    def __init__(self):
        '''
        Description:    Initialization of class arm_waypoints
        '''

        super().__init__(
            'arm_waypoints_node',
            parameter_overrides=[rclpy.parameter.Parameter(
                'use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        ############ Topic PUBLISHERS ############

        self.twist_pub = self.create_publisher(
            TwistStamped,
            '/delta_twist_cmds',
            10
        )

        self.joint_pub = self.create_publisher(
            Float64MultiArray,
            '/delta_joint_controller/commands',
            10
        )

        ############ Topic SUBSCRIPTIONS ############

        tcp_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=20
        )

        self.tcp_sub = self.create_subscription(
            PoseStamped,
            '/tcp_pose_raw',
            self.tcpposecb,
            tcp_qos
        )

        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.jointstatecb,
            tcp_qos
        )

        self.status_sub = self.create_subscription(
            Int32,
            '/arm_status',
            self.armstatuscb,
            10
        )

       ############################################

        control_rate = 0.05
        self.switch_cli = self.create_client(
            SwitchController, f'{servo_ns}/switch_controller')
        self.timer = self.create_timer(control_rate, self.process_waypoints)

        self.tcp_pose = None
        self.joint_angles = None
        self.arm_status = None
        self.current_orientation = None

        ############ ADD YOUR CODE HERE ############

        self.current_waypoint = 0
        self.hold_start = None
        self.last_status = None

        # Conservative Cartesian motion settings.
        self.position_kp = 0.6
        self.orientation_kp = 0.0

        self.arrive_tolerance = 0.015
        self.drift_tolerance = 0.025

        # Keep commanded motion well below the servo limit.
        self.max_linear_speed = 0.04
        self.max_angular_speed = 0.0

        self.motion_phase = -1
        self.safe_height = 0.65




        ############################################


    def tcpposecb(self, data):
        '''
        Description:    Callback function for the tool pose topic.

        Args:

            data (PoseStamped):    Pose of the tool, reported in base_link

        Returns:
        '''

        ############ ADD YOUR CODE HERE ############

        self.tcp_pose = data.pose

        if not hasattr(self, '_tcp_received'):
            self._tcp_received = True
            self.get_logger().info(
                f'TCP pose received: '
                f'x={data.pose.position.x:.3f}, '
                f'y={data.pose.position.y:.3f}, '
                f'z={data.pose.position.z:.3f}'
            )

        self.current_orientation = Rotation.from_quat([
            data.pose.orientation.x,
            data.pose.orientation.y,
            data.pose.orientation.z,
            data.pose.orientation.w
        ])

        ############################################


    def jointstatecb(self, data):
        '''
        Description:    Callback function for the joint states topic.

        Args:
            data (JointState):    Joint feedback published by the arm

         Returns:
        '''

        ############ ADD YOUR CODE HERE ############

        self.joint_angles = [
             data.position[data.name.index(j)]
            for j in joint_names
        ]

        ############################################


    def armstatuscb(self, data):
        '''
        Description:    Callback function for the arm status topic.

        Args:
            data (Int32):    One state code describing what the arm is doing

        Returns:
        '''

        ############ ADD YOUR CODE HERE ############

        self.arm_status = data.data
        if self.arm_status != self.last_status:
            self.get_logger().info(f'Arm status changed to {self.arm_status}')
            self.last_status = self.arm_status

        ############################################


    def switch_controller(self, controller):
        '''
        Request a controller switch without spinning the node
        from inside a timer callback.
        '''

        if controller == 'twist_controller':
            activate = [twist_controller]
            deactivate = [joint_controller]

        elif controller == 'joint_controller':
            activate = [joint_controller]
            deactivate = [twist_controller]

        else:
            self.get_logger().error(
                f'Unknown controller: {controller}'
            )
            return False

        if not self.switch_cli.service_is_ready():
            self.get_logger().warn(
                'Controller switch service not ready yet.'
            )
            return False

        req = SwitchController.Request()
        req.activate_controllers = activate
        req.deactivate_controllers = deactivate
        req.strictness = SwitchController.Request.STRICT

        self.switch_cli.call_async(req)

        self.get_logger().info(
            f'Requested controller switch to {controller}'
        )

        return True



    def send_twist(self, linear, angular):
        '''
        Convert Cartesian linear/angular velocity to joint velocity
        using a numerical 6x6 Jacobian and publish to the
        delta_joint_controller.
        '''

        if self.joint_angles is None:
            return

        q = np.asarray(self.joint_angles, dtype=float)
        eps = 1e-5

        def fk_full(q):
            dh = [
                (0.0,       np.pi / 2, 0.180,   q[0]),
                (-0.6127,   0.0,       0.0,     q[1]),
                (-0.57155,  0.0,       0.0,     q[2]),
                (0.0,       np.pi / 2, 0.17415, q[3]),
                (0.0,      -np.pi / 2, 0.11985, q[4]),
                (0.0,       0.0,       0.11655, q[5])
            ]

            T = np.eye(4)

            for a, alpha, d, theta in dh:
                ct = np.cos(theta)
                st = np.sin(theta)
                ca = np.cos(alpha)
                sa = np.sin(alpha)
                A = np.array([
                    [ct, -st * ca,  st * sa, a * ct],
                    [st,  ct * ca, -ct * sa, a * st],
                    [0.0, sa,       ca,       d],
                    [0.0, 0.0,      0.0,      1.0]
                ])

                T = T @ A

            return T

        T0 = fk_full(q)
        p0 = T0[:3, 3]
        R0 = T0[:3, :3]

        J = np.zeros((6, 6))

        for i in range(6):
            q2 = q.copy()
            q2[i] += eps

            T2 = fk_full(q2)
            p2 = T2[:3, 3]
            R2 = T2[:3, :3]

            J[:3, i] = (p2 - p0) / eps

            dR = R2 @ R0.T
            d_rot = Rotation.from_matrix(dR).as_rotvec()

            J[3:, i] = d_rot / eps

        desired_linear = np.asarray(linear, dtype=float)

        desired_axis = np.array(
           [0.0, 0.0, -1.0],
            dtype=float
        )

        current_axis = R0[:, 2]

        axis_error = np.cross(
            current_axis,
            desired_axis
        )

        dot_error = np.clip(
            np.dot(current_axis, desired_axis),
            -1.0,
            1.0
        )

        angle_error = math.acos(dot_error)

        if angle_error > 1e-4:
            axis_norm = np.linalg.norm(axis_error)

            if axis_norm > 1e-6:
                axis_error = axis_error / axis_norm

            desired_angular = (
                0.8
                * angle_error
                * axis_error
            )
        else:
            desired_angular = np.zeros(3)

        max_angular_speed = 0.30

        angular_speed = np.linalg.norm(desired_angular)

        self.tcp_pose = None
        self.joint_angles = None
        self.arm_status = None
        self.current_orientation = None

        ############ ADD YOUR CODE HERE ############

        self.current_waypoint = 0
        self.hold_start = None
        self.last_status = None

        # Conservative Cartesian motion settings.
        if angular_speed > max_angular_speed:
           desired_angular *= (
                max_angular_speed / angular_speed
            )

        desired = np.concatenate(
            [desired_linear, desired_angular]
        )

        damping = 0.01
        JT = J.T

        qdot = JT @ np.linalg.inv(
            J @ JT
            + damping * damping * np.eye(6)
        ) @ desired

        max_joint_speed = 0.20

        max_value = np.max(np.abs(qdot))

        if max_value > max_joint_speed:
            qdot *= (
                max_joint_speed / max_value
            )

        msg = Float64MultiArray()

        msg.data = [
            float(v)
            for v in qdot
        ]

        self.joint_pub.publish(msg)

    def process_waypoints(self):
        '''
        Description: Timer function used to drive the tool through waypoints.
        '''

        zero = np.zeros(3)

        self.get_logger().info(
            f'Waypoint process: phase={self.motion_phase}, '
            f'waypoint={self.current_waypoint}',
            throttle_duration_sec=2.0
        )

        if self.tcp_pose is None:
            self.get_logger().warn(
                'Waiting for /tcp_pose_raw',
                throttle_duration_sec=2.0
            )
            return

        if self.joint_angles is None:
            self.get_logger().warn(
                'Waiting for /joint_states',
                throttle_duration_sec=2.0
            )
            return

        if self.arm_status is None:
            self.get_logger().warn(
                'Waiting for /arm_status',
                throttle_duration_sec=2.0
            )
            return

        if self.current_orientation is None:
            self.get_logger().warn(
                'Waiting for TCP orientation',
                throttle_duration_sec=2.0
            )
            return

        self.get_logger().info(
            'ALL INPUTS READY - processing motion',
            throttle_duration_sec=2.0
        )

        current_position = np.array([
            self.tcp_pose.position.x,
            self.tcp_pose.position.y,
            self.tcp_pose.position.z
        ], dtype=float)

        target_position = np.array(
            waypoints[self.current_waypoint],
            dtype=float
        )

        now = self.get_clock().now().nanoseconds / 1e9

        # ---------------------------------------------------------
        # INITIAL CLEARANCE
        # ---------------------------------------------------------
        if self.motion_phase == -1:

            current_z = current_position[2]
            z_error = self.safe_height - current_z

            if abs(z_error) <= 0.01:
                self.motion_phase = 1
                self.send_twist(zero, zero)

                self.get_logger().info(
                    'Initial clearance reached. Moving horizontally.'
                )
                return

            linear_velocity = np.array([
                0.0,
                0.0,
                self.position_kp * z_error
            ], dtype=float)

            # Faster initial vertical movement
            max_vertical_speed = 0.08

            if abs(linear_velocity[2]) > max_vertical_speed:
                linear_velocity[2] = (
                    np.sign(linear_velocity[2])
                    * max_vertical_speed
                )

            self.get_logger().info(
                f'PUBLISHING TWIST: z={linear_velocity[2]:.4f}'
            )

            self.send_twist(linear_velocity, zero)
            return

        # ---------------------------------------------------------
        # PHASE 1
        # Move horizontally at safe height.
        # ONE AXIS AT A TIME.
        # ---------------------------------------------------------
        if self.motion_phase == 1:

            xy_error = target_position[:2] - current_position[:2]
            xy_distance = float(np.linalg.norm(xy_error))

            if xy_distance <= 0.015:
                self.motion_phase = 2
                self.send_twist(zero, zero)

                self.get_logger().info(
                    f'Above waypoint {self.current_waypoint + 1}. '
                    f'Descending.'
                )
                return

            if abs(xy_error[0]) >= abs(xy_error[1]):
                linear_velocity = np.array([
                    self.position_kp * xy_error[0],
                    0.0,
                    0.0
                ], dtype=float)
           else:
                linear_velocity = np.array([
                    0.0,
                    self.position_kp * xy_error[1],
                    0.0
                ], dtype=float)

            max_horizontal_speed = 0.03

            horizontal_speed = float(
                np.linalg.norm(linear_velocity[:2])
            )

            if horizontal_speed > max_horizontal_speed:
                linear_velocity[:2] *= (
                    max_horizontal_speed / horizontal_speed
                )

            self.send_twist(linear_velocity, zero)
            return

        # ---------------------------------------------------------
        # PHASE 2
        # Descend vertically to waypoint.
        # ---------------------------------------------------------
        if self.motion_phase == 2:

            z_error = target_position[2] - current_position[2]

            if abs(z_error) <= self.arrive_tolerance:

                self.send_twist(zero, zero)

                if self.hold_start is None:
                    self.hold_start = now

                    self.get_logger().info(
                        f'Waypoint {self.current_waypoint + 1} reached. '
                        f'Holding for '
                        f'{hold_times[self.current_waypoint]:.1f}s.'
                    )
                    return

                if (now - self.hold_start >=
                        hold_times[self.current_waypoint]):

                    self.current_waypoint += 1
                    self.hold_start = None
                    self.motion_phase = -1

                    self.get_logger().info(
                        f'Waypoint {self.current_waypoint} complete.'
                    )

                return

            linear_velocity = np.array([
                0.0,
                0.0,
                self.position_kp * z_error
            ], dtype=float)

            max_vertical_speed = 0.02

            if abs(linear_velocity[2]) > max_vertical_speed:
                linear_velocity[2] = (
                    np.sign(linear_velocity[2])
                    * max_vertical_speed
                )

            self.send_twist(linear_velocity, zero)
            return

##################### FUNCTION DEFINITION #######################

def main():
    '''
    Description:    Main function which creates a ROS node and spins around for the
                    arm_waypoints class to perform its task
    '''

    rclpy.init(args=sys.argv)

    node = rclpy.create_node('arm_waypoints_process')

    node.get_logger().info('Node created: Arm waypoints process')

    arm_waypoints_class = arm_waypoints()

    rclpy.spin(arm_waypoints_class)

    arm_waypoints_class.destroy_node()


if __name__ == '__main__':
    main()



