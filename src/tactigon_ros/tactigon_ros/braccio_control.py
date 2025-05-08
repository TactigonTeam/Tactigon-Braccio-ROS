#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
from tactigon_arduino_braccio import Braccio, BraccioConfig, Wrist, Gripper
from tactigon_msgs.msg import TSkinState as TSkinStateMsg
from tactigon_msgs.msg import Touch, Angle, Gesture
from tactigon_msgs.msg import BraccioResponse  # Import BraccioResponse message
from tactigon_gear import OneFingerGesture

class TactigonControlNode(Node):
    def __init__(self):
        super().__init__('tactigon_control_node')

        # Braccio setup
        braccio_cfg = BraccioConfig("D1:EF:85:90:07:DE")
        self.braccio = Braccio(braccio_cfg)
        # enter context manually
        self.braccio.__enter__()
        while not self.braccio.connected:
            self.get_logger().info("Waiting for Braccio...")
            time.sleep(0.1)
        self.get_logger().info("Braccio connected.")

        # initial pose/state
        self.x = 0
        self.y = 0
        self.z = 150
        self.wrist = Wrist.HORIZONTAL
        self.gripper = Gripper.CLOSE

        # live tracking mode flag
        self.live_tracking = False

        # Publisher for Braccio move result
        self.move_result_pub = self.create_publisher(BraccioResponse, '/braccio_move_result', 10)

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
        
        self.get_logger().info(f"{msg.touchpad.one_finger} -- {OneFingerGesture.SINGLE_TAP.value}")

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
            self.wrist = (
                Wrist.VERTICAL
                if self.wrist == Wrist.HORIZONTAL
                else Wrist.HORIZONTAL
            )
            modified = True

        # single tap toggles gripper
        if msg.touchpad_valid and msg.touchpad.one_finger == OneFingerGesture.SINGLE_TAP.value:
            self.gripper = (
                Gripper.OPEN if self.gripper == Gripper.CLOSE else Gripper.CLOSE
            )
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
            # Example: map angles to X/Y/Z (customize as needed)
            #self.x = int(msg.angle.roll * 0.03 )
            roll = msg.angle.roll
            if 40 >= roll >= -30:
                new_x = abs(roll * 10) if roll < 0 else -int(roll * 7.5)
                if new_x != self.x:
                    self.x = new_x
                    
            #self.y = int(msg.angle.pitch * 0.03 + 50)
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
            res, status, move_time = self.braccio.move(
                self.x, self.y, self.z, self.wrist, self.gripper
            )
            # Publish move result
            move_msg = BraccioResponse()
            move_msg.success = bool(res)
            move_msg.status = str(status)
            move_msg.move_time = float(move_time)
            self.move_result_pub.publish(move_msg)
            if res:
                self.get_logger().info(f"Moved → status: {status}, t: {move_time:.2f}s")
            else:
                self.get_logger().warn("Braccio move failed.")

    def destroy_node(self):
        # bring Braccio home and clean up
        try:
            self.braccio.home()
        finally:
            self.braccio.__exit__(None, None, None)
        super().destroy_node()


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
