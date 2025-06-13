import PySpin
import os
import threading

class LiveVideoRecorder:
    def __init__(self, cam_index=0):
        self.system = PySpin.System.GetInstance()
        self.cam_list = self.system.GetCameras()
        self.cam_index = cam_index
        self.cam = None
        self.nodemap = None
        self.nodemap_tldevice = None
        self.processor = PySpin.ImageProcessor()
        self.processor.SetColorProcessing(PySpin.SPINNAKER_COLOR_PROCESSING_ALGORITHM_HQ_LINEAR)
        self.recording = False
        self.thread = None
        self.images = []
        print(f"[INFO] Found {self.cam_list.GetSize()} cameras.")

    def start_recording(self, destination_folder):
        if self.cam_list.GetSize() <= self.cam_index:
            print(f"[ERROR] Camera index {self.cam_index} out of range.")
            return False

        self.cam = self.cam_list[self.cam_index]
        self.cam.Init()
        self.nodemap = self.cam.GetNodeMap()
        self.nodemap_tldevice = self.cam.GetTLDeviceNodeMap()

        # Acquisition mode: Continuous
        node_acquisition_mode = PySpin.CEnumerationPtr(self.nodemap.GetNode('AcquisitionMode'))
        node_acquisition_mode_continuous = node_acquisition_mode.GetEntryByName('Continuous')
        node_acquisition_mode.SetIntValue(node_acquisition_mode_continuous.GetValue())

        # Optional: Gev settings
        node_resend = PySpin.CBooleanPtr(self.nodemap.GetNode("GevPacketResendEnable"))
        if PySpin.IsAvailable(node_resend) and PySpin.IsWritable(node_resend):
            node_resend.SetValue(True)

        node_scpd = PySpin.CIntegerPtr(self.nodemap.GetNode("GevSCPD"))
        if PySpin.IsAvailable(node_scpd) and PySpin.IsWritable(node_scpd):
            node_scpd.SetValue(200000)

        if not os.path.exists(destination_folder):
            os.makedirs(destination_folder)
        os.chdir(destination_folder)

        self.images = []
        self.recording = True
        self.thread = threading.Thread(target=self._record_loop)
        self.thread.start()
        print(f"[INFO] Camera {self.cam_index} started recording to {destination_folder}")

    def _record_loop(self):
        try:
            self.cam.BeginAcquisition()
            while self.recording:
                image_result = self.cam.GetNextImage(1000)
                if not image_result.IsIncomplete():
                    self.images.append(self.processor.Convert(image_result, PySpin.PixelFormat_RGB8))
                image_result.Release()
        except PySpin.SpinnakerException as ex:
            print(f"[ERROR] Camera {self.cam_index} recording error:", ex)
        finally:
            try:
                self.cam.EndAcquisition()
            except:
                pass

    def stop_recording(self):
        self.recording = False
        if self.thread:
            self.thread.join()
        print(f"[INFO] Camera {self.cam_index} finished acquisition. Saving video...")
        self._save_video()
        self.cam.DeInit()
        del self.cam
        self.cam_list.Clear()
        self.system.ReleaseInstance()

    def _save_video(self):
        try:
            device_serial = ""
            node_serial = PySpin.CStringPtr(self.nodemap_tldevice.GetNode('DeviceSerialNumber'))
            if PySpin.IsReadable(node_serial):
                device_serial = node_serial.GetValue()

            node_fps = PySpin.CFloatPtr(self.nodemap.GetNode('AcquisitionFrameRate'))
            if not PySpin.IsReadable(node_fps):
                print("[WARNING] Could not read frame rate, defaulting to 5 fps")
                fps = 5.0
            else:
                fps = node_fps.GetValue()

            filename = f"video_{device_serial}.mp4"

            option = PySpin.H264Option()
            option.frameRate = fps
            option.bitrate = 1000000
            option.height = self.images[0].GetHeight()
            option.width = self.images[0].GetWidth()
            option.useMP4 = True
            option.crf = 28

            video = PySpin.SpinVideo()
            video.Open(filename, option)

            for img in self.images:
                video.Append(img)

            video.Close()
            print(f"[INFO] Camera {self.cam_index} video saved: {filename}")

        except PySpin.SpinnakerException as ex:
            print(f"[ERROR] Saving video for camera {self.cam_index}: {ex}")




