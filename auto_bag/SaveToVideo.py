import PySpin
import sys
import os
import time
import threading

class LiveVideoRecorder:
    def __init__(self):
        self.system = PySpin.System.GetInstance()
        self.cam_list = self.system.GetCameras()
        self.cam = None
        self.nodemap = None
        self.nodemap_tldevice = None
        self.processor = PySpin.ImageProcessor()
        self.processor.SetColorProcessing(PySpin.SPINNAKER_COLOR_PROCESSING_ALGORITHM_HQ_LINEAR)
        self.recording = False
        self.thread = None
        self.images = []

    def start_recording(self, destination_folder):
        if self.cam_list.GetSize() == 0:
            print("No camera detected.")
            return False

        self.cam = self.cam_list[0]
        self.cam.Init()
        self.nodemap = self.cam.GetNodeMap()
        self.nodemap_tldevice = self.cam.GetTLDeviceNodeMap()

        node_acquisition_mode = PySpin.CEnumerationPtr(self.nodemap.GetNode('AcquisitionMode'))
        node_acquisition_mode_continuous = node_acquisition_mode.GetEntryByName('Continuous')
        acquisition_mode_continuous = node_acquisition_mode_continuous.GetValue()
        node_acquisition_mode.SetIntValue(acquisition_mode_continuous)

        node_resend = PySpin.CBooleanPtr(self.nodemap.GetNode("GevPacketResendEnable"))
        if PySpin.IsAvailable(node_resend) and PySpin.IsWritable(node_resend):
            node_resend.SetValue(True)

        node_scpd = PySpin.CIntegerPtr(self.nodemap.GetNode("GevSCPD"))
        if PySpin.IsAvailable(node_scpd) and PySpin.IsWritable(node_scpd):
            node_scpd.SetValue(200000)

        self.destination_folder = destination_folder
        if not os.path.exists(destination_folder):
            os.makedirs(destination_folder)
        os.chdir(destination_folder)

        self.images = []
        self.recording = True
        self.thread = threading.Thread(target=self._record_loop)
        self.thread.start()
        print("Started recording thread.")

    def _record_loop(self):
        try:
            self.cam.BeginAcquisition()
            print("Acquisition started")
            while self.recording:
                image_result = self.cam.GetNextImage(1000)
                if image_result.IsIncomplete():
                    print('Image incomplete with image status %d...' % image_result.GetImageStatus())
                else:
                    self.images.append(self.processor.Convert(image_result, PySpin.PixelFormat_RGB8))
                    print("Captured image. Total stored:", len(self.images))
                    image_result.Release()
        except PySpin.SpinnakerException as ex:
            print("Recording error:", ex)
        finally:
            try:
                self.cam.EndAcquisition()
                print("Acquisition stopped")
            except:
                pass

    def stop_recording(self):
        self.recording = False
        if self.thread:
            self.thread.join()
        print("Recording thread joined. Saving video...")
        self._save_video()
        self.cam.DeInit()
        del self.cam
        self.cam_list.Clear()
        self.system.ReleaseInstance()

    def _save_video(self):
        print('CREATING VIDEO')
        try:
            result = True
            device_serial_number = ''
            node_serial = PySpin.CStringPtr(self.nodemap_tldevice.GetNode('DeviceSerialNumber'))

            if PySpin.IsReadable(node_serial):
                device_serial_number = node_serial.GetValue()
                print('Device serial number retrieved as %s...' % device_serial_number)

            node_acquisition_framerate = PySpin.CFloatPtr(self.nodemap.GetNode('AcquisitionFrameRate'))
            if not PySpin.IsReadable(node_acquisition_framerate):
                print('Unable to retrieve frame rate. Aborting...')
                return False

            framerate_to_set = node_acquisition_framerate.GetValue()
            print('Frame rate to be set to %d...' % framerate_to_set)

            video_recorder = PySpin.SpinVideo()
            video_filename = 'SaveToVideo-Uncompressed-%s' % device_serial_number

            option = PySpin.AVIOption()
            option.frameRate = framerate_to_set
            option.height = self.images[0].GetHeight()
            option.width = self.images[0].GetWidth()

            video_recorder.Open(video_filename, option)
            print('Appending %d images to file: %s...' % (len(self.images), video_filename))

            for i, img in enumerate(self.images):
                video_recorder.Append(img)
                print(f'Appended image {i}...')

            video_recorder.Close()
            print(f'Video saved at {video_filename}')

        except PySpin.SpinnakerException as ex:
            print('Error:', ex)
            return False

        return result


if __name__ == '__main__':
    recorder = LiveVideoRecorder()
    recorder.start_recording("test_output")
    print("Press Enter to stop...")
    input()
    recorder.stop_recording()
    print("Done.")

