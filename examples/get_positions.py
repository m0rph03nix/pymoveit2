#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateListener(Node):
    def __init__(self):
        super().__init__('joint_state_listener')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10)
        self.joint_values = []
        
    def listener_callback(self, msg):
        # List of joints we are interested in
        joint_names_of_interest = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]

        # Retrieve the positions of the joints of interest
        joint_positions = []
        for joint_name in joint_names_of_interest:
            if joint_name in msg.name:
                index = msg.name.index(joint_name)
                joint_positions.append(msg.position[index])

        if len(joint_positions) == len(joint_names_of_interest):
            self.joint_values = joint_positions
            #self.get_logger().info(f"Updated joint values: {self.joint_values}")

def main(args=None):
    rclpy.init(args=args)
    
    joint_state_listener = JointStateListener()
    
    # Use a timer to stop spinning after a certain time
    try:
        rclpy.spin_once(joint_state_listener, timeout_sec=10.0)  # Run for 10 seconds
    except KeyboardInterrupt:
        pass
    
    # Shutdown the ROS client library for Python
    rclpy.shutdown()
    
    # Print the last received joint values
    print(f"\nLatest joint values: {joint_state_listener.joint_values}")

if __name__ == '__main__':
    main()
