# auto_bag/camera_capture.py

from SaveToVideo import main as record_video_main

def record_camera_video():
	try:
		record_video_main()
		return True
	except Exception as e:
		print(f"[Camera Error] {e}")
		return False
