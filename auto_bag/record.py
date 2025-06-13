import rclpy
from rclpy.node import Node
from auto_bag_interfaces.srv import RecordTopics
import yaml
import os
import subprocess
import datetime
from auto_bag.SaveToVideo import LiveVideoRecorder

class BagRecorderService(Node):
    def __init__(self):
        super().__init__('bag_recorder_service')
        self.srv = self.create_service(RecordTopics, 'record_topics', self.handle_record_bag_request)
        self.recording_process = None
        self.video_recorders = [
            LiveVideoRecorder(cam_index=0),
            LiveVideoRecorder(cam_index=1)
        ]

    def handle_record_bag_request(self, request, response):
        if request.command.lower() == "start":
            if self.recording_process:
                response.success = False
                response.message = "A recording is already in progress."
            else:
                timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
                bag_dir = f"/home/emilien/bags/bag_session_{timestamp}"
                video_dir = f"/home/emilien/bags/video_session_{timestamp}"
                os.makedirs(video_dir, exist_ok=True)

                self.recording_process = self.start_rosbag_record(request.topics, bag_dir)

                # Save video in separate folders per camera
                self.get_logger().info("Starting camera video recordings...")
                for i, recorder in enumerate(self.video_recorders):
                    recorder.start_recording(video_dir)

                response.success = True
                response.message = f"Recording started into {bag_dir}"

        elif request.command.lower() == "stop":
            if self.recording_process:
                self.stop_rosbag_record()
                for recorder in self.video_recorders:
                    recorder.stop_recording()
                response.success = True
                response.message = "Recording stopped."
            else:
                response.success = False
                response.message = "No active recording."

        else:
            response.success = False
            response.message = "Unknown command, use 'Start' or 'Stop'."

        return response

    def start_rosbag_record(self, selected_topics=None, bag_dir=None):
        if not bag_dir:
            bag_dir = os.path.expanduser("~/bags/session_default")

        if not selected_topics:
            command = ["ros2", "bag", "record", "-a", "-o", bag_dir]
        else:
            command = ["ros2", "bag", "record", "-o", bag_dir] + selected_topics

        process = subprocess.Popen(command)
        self.get_logger().info(f"Bag recording started: {selected_topics if selected_topics else 'ALL'} → {bag_dir}")
        return process

    def stop_rosbag_record(self):
        if self.recording_process:
            self.recording_process.terminate()
            self.recording_process = None
            self.get_logger().info("Bag recording stopped.")

def main():
    rclpy.init()
    node = BagRecorderService()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
