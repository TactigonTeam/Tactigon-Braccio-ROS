#!/usr/bin/env python3
import time
import datetime
from os import path, getcwd
from tactigon_gear import TSkin, TSkinConfig, Hand, GestureConfig, TSkinState
import rclpy
from rclpy.node import Node
from tactigon_msgs.msg import TSkinState as TSkinStateMsg  # Custom message
from tactigon_msgs.msg import Touch, Angle, Gesture  # Nested messages

class TactigonDataPublisher(Node):
    def __init__(self):
        super().__init__('tactigon_data_publisher')
        self.publisher = self.create_publisher(TSkinStateMsg, '/tactigon_state', 10)
        self.timer = self.create_timer(0.02, self.publish_tactigon_data)

        model_folder = getcwd()

        #TSKIN_MAC = "C0:83:3E:39:21:57"
        TSKIN_MAC = "C0:83:1F:34:23:38"  # MAC address of the Tactigon device
        TSKIN_HAND = Hand.RIGHT
        TSKIN_NAME = "TSKIN"

        gesture_config = GestureConfig(
            path.join(model_folder, "models", "demo", "model.pickle"),
            path.join(model_folder, "models", "demo", "encoder.pickle"),
            #path.join(model_folder, "examples", "gear", "modelpy12.pickle"),
            #path.join(model_folder, "examples", "gear", "encoderpy12.pickle"),
            "demo",
            datetime.datetime.now(),
            ["up", "down", "push", "pull", "twist", "circle", "swipe_r", "swipe_l"]
        )

        self.tskin = TSkin(TSkinConfig(TSKIN_MAC, TSKIN_HAND, TSKIN_NAME, gesture_config))
        self.tskin.start()

        self.get_logger().info(f"Connecting to Tactigon: {self.tskin}")

        while not self.tskin.connected:
            time.sleep(0.1)

        self.get_logger().info("Tactigon connected")

    def publish_tactigon_data(self):
        try:
            # Get full device state
            tskin_state = self.tskin.state
            #print(tskin_state)

            # Create and populate message
            msg = TSkinStateMsg()
            
            # Basic state info
            msg.connected = bool(tskin_state.connected)
            msg.battery = float(((tskin_state.battery-3.3)/0.9)*100) # Convert battery voltage to percentage
            
            # Selector information
            if tskin_state.selector is not None:
                msg.selector = int(tskin_state.selector.value)
                msg.selector_valid = True
            else:
                msg.selector_valid = False

            # Touchpad data
            if tskin_state.touchpad is not None:
                touch_msg = Touch()
                touch_msg.one_finger = int(tskin_state.touchpad.one_finger.value)
                touch_msg.two_finger = int(tskin_state.touchpad.two_finger.value)
                touch_msg.x_pos = float(tskin_state.touchpad.x_pos)
                touch_msg.y_pos = float(tskin_state.touchpad.y_pos)
                msg.touchpad = touch_msg
                msg.touchpad_valid = True
            else:
                msg.touchpad_valid = False

            # Orientation data
            if tskin_state.angle is not None:
                angle_msg = Angle()
                msg.angle.roll = tskin_state.angle.roll
                msg.angle.pitch = tskin_state.angle.pitch
                msg.angle.yaw = tskin_state.angle.yaw
                #msg.angle = angle_msg
                msg.angle_valid = True
            else:
                msg.angle_valid = False

            # Gesture recognition
            if tskin_state.gesture is not None:
                gesture_msg = Gesture()
                msg.gesture.gesture = str(tskin_state.gesture.gesture)
                msg.gesture.probability = float(tskin_state.gesture.probability)
                msg.gesture.confidence = float(tskin_state.gesture.confidence)
                msg.gesture.displacement = float(tskin_state.gesture.displacement)
                #msg.gesture = gesture_msg
                msg.gesture_valid = True
            else:
                msg.gesture_valid = False

            # Publish the message
            self.publisher.publish(msg)

            # Log publishing status
            self.get_logger().debug(f"Published TSkinState: {msg}")

        except Exception as e:
            self.get_logger().error(f"Error publishing data: {str(e)}")

    def shutdown(self):
        if self.tskin.connected:
            self.tskin.terminate()
        self.get_logger().info("Shutting down Tactigon data publisher")

def main(args=None):
    rclpy.init(args=args)
    node = TactigonDataPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node.tskin.connected and node.tskin:
            node.tskin.terminate()
    finally:
        node.destroy_node()        
        rclpy.shutdown()

if __name__ == '__main__':
    main()