#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tactigon_msgs.msg import BraccioCommand  # <--- CHANGE 'your_package_name'

class BraccioUIPublisher(Node):
    def __init__(self):
        super().__init__('braccio_ui_publisher_node')
        self.publisher_ = self.create_publisher(BraccioCommand, '/braccio_command', 10)
        self.timer_period = 1.0  # seconds (not really used for timer, but for structure)
        self.get_logger().info("Braccio UI Publisher Node started. Enter commands when prompted.")
        self.prompt_for_command()

    def prompt_for_command(self):
        while rclpy.ok():
            try:
                self.get_logger().info("\n--- Enter New Braccio Command ---")
                
                x_str = input("Enter X coordinate (integer, e.g., 0): ")
                y_str = input("Enter Y coordinate (integer, e.g., 100): ")
                z_str = input("Enter Z coordinate (integer, e.g., 150): ")
                
                wrist_state_input = input("Enter Wrist state (h for HORIZONTAL, v for VERTICAL): ").strip().lower()
                if wrist_state_input == 'h':
                    wrist_state_str = "HORIZONTAL"
                elif wrist_state_input == 'v':
                    wrist_state_str = "VERTICAL"
                else:
                    self.get_logger().warn("Invalid wrist state. Please enter 'h' or 'v'.")
                    continue

                gripper_state_input = input("Enter Gripper state (o for OPEN, c for CLOSE): ").strip().lower()
                if gripper_state_input == 'o':
                    gripper_state_str = "OPEN"
                elif gripper_state_input == 'c':
                    gripper_state_str = "CLOSE"
                else:
                    self.get_logger().warn("Invalid gripper state. Please enter 'o' or 'c'.")
                    continue

                msg = BraccioCommand()
                msg.x = int(x_str)
                msg.y = int(y_str)
                msg.z = int(z_str)
                msg.wrist_state = wrist_state_str
                msg.gripper_state = gripper_state_str

                self.publisher_.publish(msg)
                self.get_logger().info(f"Published: X={msg.x}, Y={msg.y}, Z={msg.z}, Wrist='{msg.wrist_state}', Gripper='{msg.gripper_state}'")

            except ValueError:
                self.get_logger().error("Invalid input. X, Y, Z must be integers.")
            except KeyboardInterrupt:
                self.get_logger().info("Keyboard interrupt, exiting UI publisher.")
                break
            except EOFError: # handles ctrl-d
                self.get_logger().info("EOF, exiting UI publisher.")
                break


def main(args=None):
    rclpy.init(args=args)
    node = BraccioUIPublisher()
    # We don't spin this node in the traditional sense as input() blocks.
    # The prompt_for_command method has its own loop.
    # rclpy.spin(node) # This would not work as expected due to input()
    try:
        # The loop is inside prompt_for_command. If it exits, we destroy.
        pass # Node will run until prompt_for_command exits
    except KeyboardInterrupt: # Should be caught inside prompt_for_command ideally
        node.get_logger().info("Keyboard interrupt received in main, shutting down.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()