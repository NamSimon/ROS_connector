import subprocess
import numpy as np

class GStreamerSender:
    def __init__(self, gstreamer_uri):
        self.gstreamer_uri = gstreamer_uri
        self.gstreamer_process = None

    def start_pipeline(self):
        """GStreamer 송신 파이프라인을 시작합니다."""
        gstreamer_command = [
            'gst-launch-1.0',
            'appsrc', 'format=GST_FORMAT_TIME', 'is-live=true', 'block=true',
            'caps=video/x-raw,format=BGR,width=640,height=480,framerate=30/1',
            '!', 'videoconvert',
            '!', 'x264enc',
            '!', 'mpegtsmux',
            '!', 'srtsink', f'uri={self.gstreamer_uri}'
        ]

        try:
            self.gstreamer_process = subprocess.Popen(gstreamer_command, stdin=subprocess.PIPE)
            print(f"GStreamer 송신 파이프라인 시작됨. 스트림 토픽: {self.stream_topic}")
        except Exception as e:
            print(f"GStreamer 송신 파이프라인 시작 실패: {e}")

    def send_frame(self, frame):
        """OpenCV 이미지를 GStreamer로 전송."""
        try:
            if self.gstreamer_process and self.gstreamer_process.stdin:
                self.gstreamer_process.stdin.write(frame.tobytes())
        except Exception as e:
            print(f"GStreamer로 이미지 전송 실패: {e}")

    def stop(self):
        """GStreamer 송신 파이프라인을 종료."""
        if self.gstreamer_process:
            self.gstreamer_process.terminate()


class GStreamerReceiver:
    def __init__(self, gstreamer_uri):
        self.gstreamer_uri = gstreamer_uri
        self.gstreamer_process = None

    def start_pipeline(self):
        """GStreamer 수신 파이프라인을 시작합니다."""
        gstreamer_command = [
            'gst-launch-1.0',
            'srtsink', f'uri={self.gstreamer_uri}',
            '!', 'tsdemux',
            '!', 'h264parse',
            '!', 'avdec_h264',
            '!', 'videoconvert',
            '!', 'appsink'
        ]

        try:
            self.gstreamer_process = subprocess.Popen(gstreamer_command, stdout=subprocess.PIPE)
            print(f"GStreamer 수신 파이프라인 시작됨.")
        except Exception as e:
            print(f"GStreamer 수신 파이프라인 시작 실패: {e}")

    def receive_frame(self):
        """GStreamer로부터 프레임을 수신하고 OpenCV 이미지로 변환."""
        frame_data = self.gstreamer_process.stdout.read(640 * 480 * 3)  # BGR 3채널 (640x480)
        if not frame_data:
            return None

        frame = np.frombuffer(frame_data, dtype=np.uint8).reshape((480, 640, 3))
        return frame

    def stop(self):
        """GStreamer 수신 파이프라인을 종료."""
        if self.gstreamer_process:
            self.gstreamer_process.terminate()