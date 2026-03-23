#!/usr/bin/env python3

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
import threading
import numpy as np
from urdf_parser_py.urdf import URDF
from rcl_interfaces.srv import GetParameters
from typing import List

def sine_position(t, pos_start, pos_stop, t_start, t_stop):
    """Sine interpolation between pos_start and pos_stop over [t_start, t_stop].
    The (pos_start, pos_stop values) are interpolated from -pi/2 to pi/2 in terms of sine function.
    The transition is scaled between t_start and t_stop.
    Returns pos_start for t <= t_start, pos_stop for t >= t_stop.
    """
    if t < t_start:
        return pos_start
    elif t > t_stop:
        return pos_stop

    x = (t - t_start) / (t_stop - t_start)
    return pos_start + (pos_stop - pos_start) * 0.5 * (1 + np.sin(x * np.pi - np.pi / 2))

def trajectory(t, limits: List):
    """Generates hard-coded joint configuration

    Parameters
    ----------
    t : float
        Time in seconds.
    limits : List
        Sequence of joint limit objects with .lower and .upper attributes.

    Returns
    -------
    List[float]
        Joint positions (same length as limits). Behavior:
    """
    positions = [0.0] * len(limits)
    
    joint_id = 0
    if joint_id == 0:
        if t <= 1.0:
            positions[joint_id] = limits[joint_id].lower
        if t > 1.0 and t < 4.0:
            positions[joint_id] = sine_position(t, limits[joint_id].lower, limits[joint_id].upper, t_start=1.0, t_stop=4.0)

        if t >= 4.0 and t <= 7.0:
            positions[joint_id] = limits[joint_id].upper

        if t > 7.0 and t < 11.0:
            positions[joint_id] = sine_position(t, limits[joint_id].upper, limits[joint_id].lower, t_start=7.0, t_stop=11.0)

        if t >= 11.0:
            positions[joint_id] = limits[joint_id].lower
        
    joint_id = 1
    if joint_id == 1:
        if t <= 5.0:
            positions[joint_id] = 0.0
        if t > 5.0 and t < 8.0:
            positions[joint_id] = sine_position(t, 0.0, limits[joint_id].upper, t_start=5.0, t_stop=8.0)

        if t >= 8.0:
            positions[joint_id] = limits[joint_id].upper

    return positions

class SimulateJointStates(Node):
    def __init__(self):
        super().__init__('simulate_joint_states')
        self.robot_description = None
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        self.cli = self.create_client(GetParameters, '/robot_state_publisher/get_parameters')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        

    def send_robot_description_req(self):
        req = GetParameters.Request()
        req.names = ['robot_description']

        result = self.cli.call(req)
        self.robot_description = result.values[0].string_value
        self.get_logger().info("We got this!")


    def run(self):
        movable_joints, limits = [], []
        self.send_robot_description_req()
        robot_description = URDF().from_xml_string(self.robot_description)
        self.get_logger().info("Got robot description!")

        for (joint_name, joint) in robot_description.joint_map.items():
            if joint.type not in ("revolute", "prismatic"):
                continue

            movable_joints.append(joint_name)
            limits.append(joint.limit)
            info = joint_name +" "+ str([joint.limit.lower, joint.limit.upper])
            self.get_logger().info(info)

        t_first_tick = self.get_clock().now()
        t_now = self.get_clock().now()
        
        while t_now - t_first_tick < Duration(seconds=15.0):
            t = (t_now-t_first_tick).nanoseconds*1e-9

            msg = JointState()
            msg.name = ['arm_1_joint', 'arm_2_joint', 'arm_3_joint']
            msg.position = trajectory(t, limits)
            msg.header.stamp = self.get_clock().now().to_msg()

            self.publisher_.publish(msg)
            t_now = self.get_clock().now()

        ### Execute the sequence of moves here.
        ### Start with creating a message JointState and fill it out with positions and joint names.
        ### Define a custom trajectory of joint positions. 
        ### Keep the positions within the boundaries -> np.clip(position, lower_limit, upper_limit)
        ### You can use self.get_clock().now() to make the moves time-dependent.
        

def main(args=None):
    rclpy.init(args=args)
    node = SimulateJointStates()

    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    node.run()

    node.destroy_node()
    rclpy.shutdown()
    thread.join()


if __name__ == '__main__':
    main()
