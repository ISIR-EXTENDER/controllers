#!/usr/bin/env python3

"""
ROS2 bridge between EyeTrax and the rest of the system.

This node connects to the WebSocket server started by EyeTrax, receives
the user's gaze coordinates, and publishes them on the ROS2 topic
/gaze_point as a geometry_msgs/msg/Point message.

Published message convention:
    msg.x : horizontal gaze position, normalized between 0.0 and 1.0
    msg.y : vertical gaze position, normalized between 0.0 and 1.0
    msg.z : prediction confidence, or 1.0 if EyeTrax does not provide one
"""

import asyncio
import json
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

import websockets


class EyeTraxWebSocketBridge(Node):
    """
    ROS2 node that bridges EyeTrax WebSocket data to a ROS2 topic.

    EyeTrax sends JSON messages through WebSocket, for example:
        {"x": 0.68, "y": 0.42}

    This node converts those values into a ROS2 Point message and publishes it on:
        /gaze_point
    """

    def __init__(self):
        """Initialize the ROS2 node, the publisher, and the WebSocket thread."""
        super().__init__("eyetrax_websocket_bridge")

        # ROS2 publisher used to publish the gaze point.
        self.publisher = self.create_publisher(Point, "/gaze_point", 10)

        # Address of the EyeTrax WebSocket server.
        # 127.0.0.1 means the server is running on the same machine.
        # 8001 is the port used when starting EyeTrax.
        self.websocket_url = "ws://127.0.0.1:8001"

        # WebSocket communication is asynchronous.
        # It is started in a separate thread so it does not block rclpy.spin().
        self.thread = threading.Thread(
            target=self.start_websocket_loop,
            daemon=True
        )
        self.thread.start()

        self.get_logger().info("EyeTrax WebSocket bridge started")

    def start_websocket_loop(self):
        """Start the asyncio event loop used for the WebSocket connection."""
        asyncio.run(self.websocket_loop())

    async def websocket_loop(self):
        """
        Connect to the EyeTrax WebSocket server and continuously read messages.

        If the connection fails, the node waits for one second and tries again.
        This allows the bridge to be started before EyeTrax without crashing.
        """
        while rclpy.ok():
            try:
                self.get_logger().info(f"Connecting to {self.websocket_url}")

                # Connect to the EyeTrax WebSocket server.
                async with websockets.connect(self.websocket_url) as ws:
                    self.get_logger().info("Connected to EyeTrax WebSocket")

                    # Continuously read messages sent by EyeTrax.
                    async for message in ws:
                        try:
                            # EyeTrax messages are expected to be JSON strings.
                            data = json.loads(message)

                        except json.JSONDecodeError:
                            # If the received message is not valid JSON, ignore it.
                            self.get_logger().warn(f"Invalid JSON: {message}")
                            continue

                        # EyeTrax may send an initial greeting message, for example:
                        # {"type": "hello", ...}
                        # Such messages do not contain gaze coordinates.
                        if "x" not in data or "y" not in data:
                            self.get_logger().warn(f"No x/y in message: {data}")
                            continue

                        # Extract gaze coordinates.
                        gaze_x = float(data["x"])
                        gaze_y = float(data["y"])

                        # Some messages may include a confidence value.
                        # If no confidence is provided, use 1.0 by default.
                        confidence = float(data.get("confidence", 1.0))

                        # Safety clamp: force coordinates to remain in [0, 1].
                        gaze_x = max(0.0, min(1.0, gaze_x))
                        gaze_y = max(0.0, min(1.0, gaze_y))

                        # Create the ROS2 message.
                        msg = Point()
                        msg.x = gaze_x
                        msg.y = gaze_y
                        msg.z = confidence

                        # Publish the gaze point on /gaze_point.
                        self.publisher.publish(msg)

            except Exception as e:
                # If EyeTrax is not running, the port is wrong,
                # or the connection drops, print the error and retry.
                self.get_logger().warn(f"WebSocket error: {e}")
                await asyncio.sleep(1.0)


def main(args=None):
    """Main entry point of the ROS2 node."""
    rclpy.init(args=args)

    node = EyeTraxWebSocketBridge()

    # Keep the node alive so the publisher remains active.
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()