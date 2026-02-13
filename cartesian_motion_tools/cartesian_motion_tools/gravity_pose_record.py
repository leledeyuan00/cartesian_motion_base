import time
import argparse
import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, WrenchStamped

from cartesian_controller_msgs.srv import JointMove

setting_joints = [
    [0.0, -1.57, 1.57, -3.14, -1.57, 0.0],
    [0.0, -1.57, 1.57, -3.14, -1.57, -1.57],
    [0.0, -1.57, 1.57, -3.14, -1.57, 1.57],
    [0.0, -1.57, 1.57, -2.355, -1.57, 0.0],
    [0.0, -1.57, 1.57, -1.57, -0.785, 0.0],
    [0.0, -1.57, 1.57, -1.57, -2.355, 0.0],
    [0.0, -1.57, 1.57, -1.57, -1.57, 0.0],
]


class MessageRecorder(Node):

    def __init__(self):
        super().__init__("message_recorder")
        parser = argparse.ArgumentParser(
            description="Record gravity pose messages")
        parser.add_argument('--sim', type=bool, required=False,
                            help='For initialization checking which needs the'
                            ' raw f/t data', default=False)
        args = vars(parser.parse_args())
        self.sim = args['sim']

        self.pose_subscriber = self.create_subscription(
            PoseStamped, "/cartesian_compliance_controller/current_pose",
            self.pose_callback, 10)
        self.wrench_subscriber = self.create_subscription(
            WrenchStamped, "/cartesian_compliance_controller/current_wrench",
            self.wrench_callback, 10)
        self.wrench_raw_subscriber = self.create_subscription(
            WrenchStamped, "/cartesian_compliance_controller/ft_sensor_wrench",
            self.ft_sensor_wrench_callback, 10)

        self.joint_move_client = self.create_client(
            JointMove, "/cartesian_compliance_controller/target_joint")

        self.current_pose = []
        self.current_wrench = []

        self.initialized_pose = False
        self.initialized_wrench = False
        # In sim mode, we do not need the raw f/t data
        self.initialized_wrench_raw = self.sim

        # Create the directory to save the recorded data
        home_dir = os.path.expanduser("~")
        gravity_comp_dir = os.path.join(home_dir, ".gravity_compensation")
        if not os.path.exists(gravity_comp_dir):
            os.makedirs(gravity_comp_dir)
        self.pose_file = open(
            gravity_comp_dir + "/recorded_messages_pose.txt", "w")
        self.wrench_file = open(
            gravity_comp_dir + "/recorded_messages_wrench.txt", "w")

    def pose_callback(self, msg):
        self.current_pose = [msg.pose.position.x, msg.pose.position.y,
                             msg.pose.position.z, msg.pose.orientation.x,
                             msg.pose.orientation.y, msg.pose.orientation.z,
                             msg.pose.orientation.w]
        self.initialized_pose = True

    def wrench_callback(self, msg):
        self.current_wrench = [msg.wrench.force.x, msg.wrench.force.y,
                               msg.wrench.force.z, msg.wrench.torque.x,
                               msg.wrench.torque.y, msg.wrench.torque.z]
        self.initialized_wrench = True

    def ft_sensor_wrench_callback(self, msg):
        self.initialized_wrench_raw = True

    def record(self):
        # recorded quaternion as the format: [x, y, z, w]
        self.pose_file.write(
            f"{ self.current_pose[0]}, {self.current_pose[1]}, \
            {self.current_pose[2]}, {self.current_pose[3]}, \
            {self.current_pose[4]}, {self.current_pose[5]}, \
            {self.current_pose[6]}\n")
        self.wrench_file.write(
            f"{self.current_wrench[0]}, {self.current_wrench[1]}, \
            {self.current_wrench[2]}, {self.current_wrench[3]}, \
            {self.current_wrench[4]}, {self.current_wrench[5]}\n")

    def start_record(self):
        # Wait for all the messages to be initialized
        while not self.initialized_pose or not self.initialized_wrench \
                or not self.initialized_wrench_raw:
            rclpy.spin_once(self)

        for joints in setting_joints:
            duration = 3.0
            joint_move = JointMove.Request()
            joint_move.duration = duration
            joint_move.cmd.data = joints
            future = self.joint_move_client.call_async(joint_move)

            rclpy.spin_until_future_complete(self, future)

            time.sleep(duration + 2.0)
            rclpy.spin_once(self)

            self.record()
        self.pose_file.close()
        self.wrench_file.close()


def main(args=None):
    rclpy.init(args=args)
    node = MessageRecorder()
    node.start_record()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
