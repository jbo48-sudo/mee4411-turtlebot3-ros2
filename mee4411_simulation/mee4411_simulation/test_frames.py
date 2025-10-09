# import rclpy
# import tf2_ros
# import time
# import yaml

# def main():
#     rclpy.init()
#     node = rclpy.create_node('tf2_frame_printer')

#     tf_buffer = tf2_ros.Buffer()
#     tf_listener = tf2_ros.TransformListener(tf_buffer, node)

#     # Give some time for the buffer to populate with transforms

#     time.sleep(5.0)

#     try:
#         frames_yaml = tf_buffer.all_frames_as_yaml()
#         print("All TF Frames (YAML format):")
#         print(frames_yaml)

#         # If you want a list of frame names:
#         frames_dict = yaml.safe_load(frames_yaml)
#         frames_list = list(frames_dict.keys())
#         print("\nList of TF Frame Names:")
#         for frame_name in frames_list:
#             print(f"- {frame_name}")

#     except Exception as e:
#         node.get_logger().error(f"Error getting TF frames: {e}")

#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from tf2_msgs.srv import FrameGraph
import yaml
import time


class FrameGetter(Node):

    def __init__(self):
        super().__init__('frame_getter')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        time.sleep(10.0)
        self._get_frames_and_convert()

    def _get_frames_and_convert(self):
        try:
            # Service client for tf_frames
            self.tf_client = self.create_client(FrameGraph, 'tf_frames')
            if not self.tf_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().error('tf_frames service not available')
                return

            request = FrameGraph.Request()
            future = self.tf_client.call_async(request)
            future.add_done_callback(self._frame_graph_callback)

        except Exception as e:
            self.get_logger().error(f'Error calling tf_frames service: {e}')

    def _frame_graph_callback(self, future):
        try:
            response = future.result()
            if response:
                # This is a simplified representation. A real implementation would
                # process the FrameGraph message's frames and transforms into a more
                # suitable data structure for YAML.
                yaml_data = {
                    'frames': [frame.frame_id for frame in response.frames],
                    'transforms': [
                        {'child_frame_id': tf.child_frame_id,
                         'parent_frame_id': tf.parent_frame_id}
                        for tf in response.transforms
                    ]
                }
                print(yaml.dump(yaml_data))
            else:
                self.get_logger().error('Failed to get frame graph')
        except Exception as e:
            self.get_logger().error(f'Error processing frame graph: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = FrameGetter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
