#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import threading
import numpy as np
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

def range_selector(x, ranges = ((-np.pi/4, np.pi/4), (-np.pi/2, np.pi/2), (-2*np.pi, 2*np.pi))):
    min_val, max_val = np.min(x), np.max(x)
    for r in ranges:
        min_r, max_r = r
        if min_r <= min_val and max_val<=max_r:
            return (min_r, max_r)
        
    return ranges[-1]

class ShowJointStates(Node):
    def __init__(self):
        super().__init__('show_joint_states')
        self.declare_parameter('duration', 1.0)
        self.subscriber_ = self.create_subscription(JointState, '/joint_states', self.get_state,
                                                    qos_profile=qos_profile_sensor_data)

        self.duration = self.get_parameter('duration').value
        self.state = {}

        self.lock = threading.Lock()

    def get_state(self, msg: JointState):
        stamp = msg.header.stamp
        t = stamp.sec + stamp.nanosec * 1e-9

        for position, name in zip(msg.position, msg.name):
            data = (position, t)
            with self.lock:
                if name not in self.state.keys():
                    self.state[name] = [data]
                else:
                    self.state[name].append(data)

    def run(self):
        first_tick = self.get_clock().now()
        while self.get_clock().now() - first_tick < Duration(seconds=self.duration):
            pass

        if self.state:  # if data was collected
            fig, axs = plt.subplots(len(self.state))

            fig.suptitle('Joint states')
            for i, k in enumerate(self.state.keys()):
                with self.lock:
                    data = list(self.state[k])

                ts = np.array([t for _, t in data])
                ts -= ts[0]

                states = np.array([s for s, _ in data])

                self.get_logger().info(f"states min: {np.min(states)} max: {np.max(states)}")
                self.get_logger().info(f"states len: {len(states)} time len: {len(ts)}")

                axs[i].plot(ts, states)
                axs[i].set_title(k)
                axs[i].set_ylim(*range_selector(states))
                axs[i].set_xlim(ts[0] - 0.3, ts[-1]+0.3)

                axs[i].grid(True, linestyle='--', alpha=0.6)
                axs[i].set_ylabel('rad')
                axs[i].set_xlabel('time [s]')


            fig.tight_layout()
            plt.show()

    


def main(args=None):
    rclpy.init(args=args)

    node = ShowJointStates()

    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    node.run()

    node.destroy_node()
    rclpy.shutdown()
    thread.join()


if __name__ == '__main__':
    main()
