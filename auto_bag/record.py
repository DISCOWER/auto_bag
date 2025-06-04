import rclpy
from rclpy.node import Node
from auto_bag_interfaces.srv import RecordTopics  
import yaml
import os


class DataRecordNode(Node):
    def __init__(self):
        super().__init__('data_record_node')

        self.declare_parameter('topics_file', '')
        topics_file = self.get_parameter('topics_file').get_parameter_value().string_value

        if not topics_file or not os.path.isfile(topics_file):
            self.get_logger().error(f"Invalid or missing topics file: {topics_file}")
            return

        try:
            with open(topics_file, 'r') as f:
                data = yaml.safe_load(f)
                self.topics = data.get('topics', [])
                if not self.topics:
                    self.get_logger().warn("No topics found in the YAML file.")
                    return
                self.get_logger().info(f"Topics loaded: {self.topics}")
        except Exception as e:
            self.get_logger().error(f"Failed to read YAML: {e}")
            return

        self.client = self.create_client(RecordTopics, 'record_topics')
        while not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for service "record_topics"...')

        request = RecordTopics.Request()
        request.command = 'start'
        request.topics = self.topics

        self.get_logger().info("Sending start recording request...")
        self.future = self.client.call_async(request)
        self.future.add_done_callback(self.handle_response)

    def handle_response(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"Recording started: {response.message}")
            else:
                self.get_logger().error(f"Service error: {response.message}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")


def main():
    rclpy.init()
    node = DataRecordNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

# This code is a ROS 2 node that reads a YAML file containing topics to record and calls a service to start recording those topics.
# It handles errors related to file reading and service calls, providing appropriate logging messages.


