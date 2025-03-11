#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import rclpy.waitable
from sensor_msgs.msg import JointState
import threading
import numpy as np
from urdf_parser_py.urdf import URDF
from rcl_interfaces.srv import GetParameters

class SampleJointStates(Node):
    def __init__(self):
        super().__init__('sample_joint_states')
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
        
        
        ### Execute the sequence of moves here.
        ### Start with creating a message JointState and fill it out with positions and joint names.
        ### Define a custom trajectory of joint positions. 
        ### Keep the positions within the boundaries -> np.clip(position, lower_limit, upper_limit)
        ### You can use self.get_clock().now() to make the moves time-dependent.
        


def main(args=None):
    rclpy.init(args=args)
    node = SampleJointStates()

    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    node.run()

    node.destroy_node()
    rclpy.shutdown()
    thread.join()


if __name__ == '__main__':
    main()
