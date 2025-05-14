#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
from tactigon_arduino_braccio import Braccio, BraccioConfig, Wrist, Gripper
# Import the new custom message
from tactigon_msgs.msg import BraccioCommand  # <--- CHANGE 'your_package_name'
# Import BraccioResponse if you have tactigon_msgs and want to use it
from tactigon_msgs.msg import BraccioResponse

class BraccioCommunication(Node):
    def __init__(self):
        super().__init__('braccio_communication_node')

        # Braccio setup
        # IMPORTANT: Replace with your Braccio's Bluetooth MAC address
        braccio_cfg = BraccioConfig("D1:EF:85:90:07:DE") # Or get from params
        self.braccio = Braccio(braccio_cfg)
        
        self.get_logger().info("Attempting to connect to Braccio...")
        try:
            self.braccio.__enter__() 
            time.sleep(2) # Adjust if needed
            
            if not self.braccio.connected:
                self.get_logger().warn("Braccio not immediately connected after __enter__. Waiting...")
                
                for _ in range(100): # Wait up to 10 seconds
                    if self.braccio.connected:
                        break
                    time.sleep(0.1)
            
            if self.braccio.connected:
                self.get_logger().info("Braccio connected successfully.")
            else:
                self.get_logger().error("Failed to connect to Braccio. Exiting.")                
                rclpy.shutdown()
                return 

        except Exception as e:
            self.get_logger().error(f"Error during Braccio connection: {e}")
            rclpy.shutdown()
            return

        # Publisher for Braccio move result
        self.move_result_pub = self.create_publisher(BraccioResponse, '/braccio_move_result', 10)

        # Subscribe to the BraccioCommand topic
        self.command_subscriber = self.create_subscription(
            BraccioCommand,
            '/braccio_command',
            self.command_callback,
            10
        )
        self.get_logger().info("Braccio Communication Node started. Waiting for commands on /braccio_command.")

    def command_callback(self, msg: BraccioCommand):
        self.get_logger().info(
            f"Received command: X={msg.x}, Y={msg.y}, Z={msg.z}, "
            f"Wrist='{msg.wrist_state}', Gripper='{msg.gripper_state}'"
        )

        # Convert string states to Braccio Enum types
        target_wrist = Wrist.HORIZONTAL # Default
        if msg.wrist_state.upper() == "VERTICAL":
            target_wrist = Wrist.VERTICAL
        elif msg.wrist_state.upper() != "HORIZONTAL":
            self.get_logger().warn(f"Invalid wrist state: '{msg.wrist_state}'. Using HORIZONTAL.")

        target_gripper = Gripper.CLOSE # Default
        if msg.gripper_state.upper() == "OPEN":
            target_gripper = Gripper.OPEN
        elif msg.gripper_state.upper() != "CLOSE":
            self.get_logger().warn(f"Invalid gripper state: '{msg.gripper_state}'. Using CLOSE.")

        if not self.braccio.connected:
            self.get_logger().error("Braccio is not connected. Cannot execute move.")
            # Optionally publish a failure response
            response_msg = BraccioResponse()
            response_msg.success = False
            response_msg.status = "Braccio not connected"
            response_msg.move_time = 0.0
            self.move_result_pub.publish(response_msg)
            return

        # Execute the move
        try:
            res, status, move_time = self.braccio.move(
                msg.x, msg.y, msg.z, target_wrist, target_gripper
            )

            # Publish move result
            response_msg = BraccioResponse()
            response_msg.success = bool(res)
            response_msg.status = str(status)
            response_msg.move_time = float(move_time)
            self.move_result_pub.publish(response_msg)

            if res:
                self.get_logger().info(f"Move successful -> status: {status}, t: {move_time:.2f}s")
            else:
                self.get_logger().warn(f"Braccio move failed. Status: {status}")
        except Exception as e:
            self.get_logger().error(f"Exception during braccio.move: {e}")
            response_msg = BraccioResponse()
            response_msg.success = False
            response_msg.status = f"Exception: {e}"
            response_msg.move_time = 0.0
            self.move_result_pub.publish(response_msg)


    def destroy_node(self):
        self.get_logger().info("Shutting down Braccio Simple Controller Node...")
        if hasattr(self, 'braccio') and self.braccio.connected:
            self.get_logger().info("Sending Braccio to home position...")
            try:
                self.braccio.home()
                time.sleep(1) # Give it a moment to complete home
            except Exception as e:
                self.get_logger().warn(f"Could not send Braccio to home: {e}")
            finally:
                self.get_logger().info("Cleaning up Braccio connection...")
                self.braccio.__exit__(None, None, None) # Manually exit context
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BraccioCommunication()
    if not rclpy.ok(): # Check if init failed
        return

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down.")
    except Exception as e:
        node.get_logger().error(f"Unhandled exception in spin: {e}")
    finally:
        if rclpy.ok() and node: # Ensure node exists and rclpy is still up
             if node.braccio.connected : # Check if node was fully initialized
                node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()