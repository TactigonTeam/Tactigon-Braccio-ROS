#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tactigon_msgs.msg import TSkinState as TSkinStateMsg
from tactigon_msgs.msg import BraccioResponse
from tactigon_gear import OneFingerGesture
from tactigon_msgs.msg import BraccioCommand

class TactigonControlNode(Node):
    def __init__(self):
        super().__init__('tactigon_control_node')

        # initial pose/state
        self.x = 0
        self.y = 0
        self.z = 150
        self.wrist = 0  # 0: HORIZONTAL, 1: VERTICAL
        self.gripper = 0  # 0: CLOSE, 1: OPEN

        # live tracking mode flag
        self.live_tracking = False

        # Publisher for Braccio move result
        self.move_result_pub = self.create_publisher(BraccioResponse, '/braccio_move_result', 10)
        # Publisher for BraccioCommand
        self.braccio_command_pub = self.create_publisher(BraccioCommand, '/braccio_command', 10)

        # subscribe to the TSkinState topic
        self.sub = self.create_subscription(
            TSkinStateMsg,
            '/tactigon_state',
            self.tskin_callback,
            10
        )

    def tskin_callback(self, msg: TSkinStateMsg):
        """Called whenever new Tactigon data arrives."""
        modified = False

        # swipe_l toggles live tracking mode
        if msg.gesture_valid and msg.gesture.gesture == "swipe_l":
            self.live_tracking = not self.live_tracking
            self.get_logger().info(f"Live tracking mode {'enabled' if self.live_tracking else 'disabled'}.")
            return
        
        #self.get_logger().info(f"{msg.touchpad.one_finger} -- {OneFingerGesture.SINGLE_TAP.value}")

        if self.live_tracking and msg.touchpad.one_finger == OneFingerGesture.SINGLE_TAP.value:
            self.live_tracking = not self.live_tracking
            self.get_logger().info("Live tracking mode disabled.")
            return
        # stop on circle
        if msg.gesture_valid and msg.gesture.gesture == "circle":
            self.get_logger().info("Circle gesture → shutting down.")
            rclpy.shutdown()
            return

        # up/down sets Z
        if msg.gesture_valid and msg.gesture.gesture == "up":
            self.z = 150
            modified = True
        elif msg.gesture_valid and msg.gesture.gesture == "down":
            self.z = 0
            modified = True

        # twist toggles wrist
        if msg.gesture_valid and msg.gesture.gesture == "twist":
            self.wrist = 1 if self.wrist == 0 else 0
            modified = True

        # single tap toggles gripper
        if msg.touchpad_valid and msg.touchpad.one_finger == OneFingerGesture.SINGLE_TAP.value:
            self.gripper = 1 if self.gripper == 0 else 0
            modified = True

        # tap-and-hold + angle drives X/Y [currenly disabled, gesture swipe_l toggles live tracking]
        if (
            msg.touchpad_valid and
            msg.touchpad.one_finger == OneFingerGesture.TAP_AND_HOLD.value and
            msg.angle_valid
        ):
            # map pitch → Y
            pitch = msg.angle.pitch
            if 0 >= pitch >= -90:
                new_y = int(abs(pitch) * 3.33)
                if new_y != self.y:
                    self.y = new_y
                    modified = True

            # map roll → X
            roll = msg.angle.roll
            if 40 >= roll >= -30:
                new_x = abs(roll * 10) if roll < 0 else -int(roll * 7.5)
                if new_x != self.x:
                    self.x = new_x
                    modified = True

        # live tracking mode: update X/Y/Z from angles
        if self.live_tracking and msg.angle_valid:
            roll = msg.angle.roll
            if 40 >= roll >= -30:
                new_x = abs(roll * 10) if roll < 0 else -int(roll * 7.5)
                if new_x != self.x:
                    self.x = new_x
            pitch = msg.angle.pitch
            if 0 >= pitch >= -90:
                new_y = int(abs(pitch) * 3.33)
                if new_y != self.y:
                    self.y = new_y
            self.z = int(150)
            print(f"X: {self.x}, Y: {self.y}, Z: {self.z}")
            modified = True

        # execute if anything changed
        if modified:
            cmd_msg = BraccioCommand()
            cmd_msg.x = self.x
            cmd_msg.y = self.y
            cmd_msg.z = self.z
            cmd_msg.wrist_state = "HORIZONTAL" if self.wrist == 0 else "VERTICAL"
            cmd_msg.gripper_state = "CLOSE" if self.gripper == 0 else "OPEN"
            self.braccio_command_pub.publish(cmd_msg)
            self.get_logger().info(f"Published BraccioCommand: X={cmd_msg.x}, Y={cmd_msg.y}, Z={cmd_msg.z}, Wrist={cmd_msg.wrist_state}, Gripper={cmd_msg.gripper_state}")


def main(args=None):
    rclpy.init(args=args)
    node = TactigonControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
