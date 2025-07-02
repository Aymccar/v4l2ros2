import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

import numpy as np
import av

class VideoWriter(Node):
    def __init__(self):
        super().__init__('video_writer_pyav')
        
        # Open PyAV container to virtual camera (v4l2loopback)
        self.container = av.open('/dev/video42', mode='w', format='v4l2')

        self.width = 1920
        self.height = 1080
        self.fps = 30

        # Add stream to the container
        self.stream = self.container.add_stream('rawvideo', rate=self.fps)
        self.stream.width = self.width
        self.stream.height = self.height
        self.stream.pix_fmt = 'yuv420p'

        # Subscribe directly to the image topic
        self.subscription = self.create_subscription(
            Image,
            '/anglerfish/image',
            self.callback,
            10
        )

    def callback(self, msg: Image):
        try:
            # Ensure the encoding is as expected
            if msg.encoding != 'rgb8':
                self.get_logger().warn(f"Expected 'rgb8' encoding, got '{msg.encoding}'")
                return

            # Convert raw bytes to NumPy array
            frame_array = np.frombuffer(msg.data, dtype=np.uint8)
            frame_array = frame_array.reshape((msg.height, msg.width, 3))  # RGB

            # Create a PyAV video frame from the RGB data
            video_frame = av.VideoFrame.from_ndarray(frame_array, format='rgb24')
            video_frame = video_frame.reformat(format='yuv420p')

            # Encode and mux into the container
            for packet in self.stream.encode(video_frame):
                self.container.mux(packet)

        except Exception as e:
            self.get_logger().error(f"Error processing frame: {e}")

    def destroy_node(self):
        # Flush and close the container on shutdown
        for packet in self.stream.encode():
            self.container.mux(packet)
        self.container.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    video_writer = VideoWriter()

    try:
        rclpy.spin(video_writer)
    finally:
        video_writer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

