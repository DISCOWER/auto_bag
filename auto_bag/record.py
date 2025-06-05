import rclpy
from rclpy.node import Node
from auto_bag_interfaces.srv import RecordTopics  
import yaml
import os
import subprocess
import datetime

class BagRecorderService(Node):

    def __init__(self):
        """
        Initializes the ROS 2 service node that manages bag recording.
        """
        super().__init__('bag_recorder_service')
        self.srv = self.create_service(RecordTopics, 'record_topics', self.handle_record_bag_request)
        self.recording_process = None  # Process that manages recording

    def handle_record_bag_request(self, request, response):
        """
        Handles the service request to start or stop recording.
        """
        if request.command.lower() == "start":
            if self.recording_process:
                response.success = False
                response.message = "A recording is already in progress."
            else:
                self.recording_process = self.start_rosbag_record(request.topics)
                response.success = True
                response.message = "Recording started."

        elif request.command.lower() == "stop":
            if self.recording_process:
                self.stop_rosbag_record()
                response.success = True
                response.message = "Recording stopped."
            else:
                response.success = False
                response.message = "No active recording."

        else:
            response.success = False
            response.message = "Unknown command, use 'Start' or 'Stop'."

        return response

    def start_rosbag_record(self, selected_topics=None):
        """
        Starts recording specific ROS 2 topics. If no topics are provided, records all active topics.
        """
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        bag_dir = os.path.expanduser(f"~/ros2_lab/bags/session_{timestamp}")

        # If no topics are specified, record all active topics
        if not selected_topics:
            command = ["ros2", "bag", "record", "-a", "-o", bag_dir]
        else:
            command = ["ros2", "bag", "record", "-o", bag_dir] + selected_topics

        # Start the bag recording process
        process = subprocess.Popen(command)
        self.get_logger().info(f"Recording started: {selected_topics if selected_topics else 'ALL'} → {bag_dir}")
        return process

    def stop_rosbag_record(self):
        """
        Stops the currently running recording process.
        """
        if self.recording_process:
            self.recording_process.terminate()
            self.recording_process = None
            self.get_logger().info("Recording stopped.")

def main():
    """
    Main function to initialize the ROS 2 node and start the service.
    """
    rclpy.init()
    node = BagRecorderService()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
