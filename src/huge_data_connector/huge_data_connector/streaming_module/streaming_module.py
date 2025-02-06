

# import subprocess
# import numpy as np

# class GStreamerSender:
#     def __init__(self, gstreamer_uri):
#         self.gstreamer_uri = gstreamer_uri
#         self.gstreamer_process = None

#     def start_pipeline(self):
#         """GStreamer 송신 파이프라인을 시작합니다."""
#         gstreamer_command = [
#             'gst-launch-1.0',
#             'appsrc', 'format=GST_FORMAT_TIME', 'is-live=true', 'block=true',
#             'caps=video/x-raw,format=BGR,width=640,height=480,framerate=30/1',
#             '!', 'videoconvert',
#             '!', 'x264enc',
#             '!', 'mpegtsmux',
#             '!', 'srtsink', f'uri={self.gstreamer_uri}'
#         ]

#         try:
#             self.gstreamer_process = subprocess.Popen(gstreamer_command, stdin=subprocess.PIPE)
#             print(f"GStreamer 송신 파이프라인 시작됨. 스트림 토픽: {self.stream_topic}")
#         except Exception as e:
#             print(f"GStreamer 송신 파이프라인 시작 실패: {e}")

#     def send_frame(self, frame):
#         """OpenCV 이미지를 GStreamer로 전송."""
#         try:
#             if self.gstreamer_process and self.gstreamer_process.stdin:
#                 self.gstreamer_process.stdin.write(frame.tobytes())
#         except Exception as e:
#             print(f"GStreamer로 이미지 전송 실패: {e}")

#     def stop(self):
#         """GStreamer 송신 파이프라인을 종료."""
#         if self.gstreamer_process:
#             self.gstreamer_process.terminate()


# class GStreamerReceiver:
#     def __init__(self, gstreamer_uri):
#         self.gstreamer_uri = gstreamer_uri
#         self.gstreamer_process = None

#     def start_pipeline(self):
#         """GStreamer 수신 파이프라인을 시작합니다."""
#         gstreamer_command = [
#             'gst-launch-1.0',
#             'srtsink', f'uri={self.gstreamer_uri}',
#             '!', 'tsdemux',
#             '!', 'h264parse',
#             '!', 'avdec_h264',
#             '!', 'videoconvert',
#             '!', 'appsink'
#         ]

#         try:
#             self.gstreamer_process = subprocess.Popen(gstreamer_command, stdout=subprocess.PIPE)
#             print(f"GStreamer 수신 파이프라인 시작됨.")
#         except Exception as e:
#             print(f"GStreamer 수신 파이프라인 시작 실패: {e}")

#     def receive_frame(self):
#         """GStreamer로부터 프레임을 수신하고 OpenCV 이미지로 변환."""
#         frame_data = self.gstreamer_process.stdout.read(640 * 480 * 3)  # BGR 3채널 (640x480)
#         if not frame_data:
#             return None

#         frame = np.frombuffer(frame_data, dtype=np.uint8).reshape((480, 640, 3))
#         return frame

#     def stop(self):
#         """GStreamer 수신 파이프라인을 종료."""
#         if self.gstreamer_process:
#             self.gstreamer_process.terminate()


import subprocess
import numpy as np
import cv2  # OpenCV 추가


class GStreamerSender:
    def __init__(self, gstreamer_uri, width=640, height=480, fps=30):
        self.gstreamer_uri = gstreamer_uri
        self.width = int(width)
        self.height = int(height)
        self.fps = fps
        self.video_writer = None

    def start_pipeline(self):

        gst_pipeline = (
            f"appsrc ! "
            f"videoconvert ! "
            f"x264enc bitrate=10000 speed-preset=ultrafast tune=zerolatency ! "
            f"mpegtsmux ! "
            f"srtsink uri={self.gstreamer_uri}"
        )

        # OpenCV VideoWriter 초기화 (수정된 부분)
        self.video_writer = cv2.VideoWriter(
            gst_pipeline,
            apiPreference=cv2.CAP_GSTREAMER,
            fourcc=0,
            fps=self.fps,
            frameSize=(self.width, self.height),
            isColor=True
        )

        if not self.video_writer.isOpened():
            print("Error: Unable to open GStreamer pipeline for streaming.")
            return False

        print("GStreamer pipeline started successfully.")
        return True

    def send_frame(self, frame):
        if self.video_writer and self.video_writer.isOpened():
            resized_frame=cv2.resize(frame,(self.width,self.height))
            self.video_writer.write(resized_frame)
        else:
            print("Error: VideoWriter is not opened.")

        """GStreamer 송신 파이프라인을 시작합니다."""
        gstreamer_command = [
            'gst-launch-1.0', '-v',
            'appsrc', 'format=GST_FORMAT_TIME', 'is-live=true', 'block=true',
            'caps=video/x-raw,format=BGR,width=640,height=480,framerate=30/1',
            '!', 'videoconvert',
            '!', 'x264enc', 'tune=zerolatency', 'bitrate=5000',
            '!', 'mpegtsmux',
            '!', 'srtsink', f'uri={self.gstreamer_uri}'  # 🔥 srtsink 유지
        ]

        try:
            self.gstreamer_process = subprocess.Popen(
                gstreamer_command, stdin=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True
            )
            print(f"GStreamer 송신 파이프라인 시작됨. URI: {self.gstreamer_uri}")
        except Exception as e:
            print(f"GStreamer 송신 파이프라인 시작 실패: {e}")

    def send_frame(self, frame):
        """OpenCV 이미지를 640x480으로 변환 후 GStreamer로 전송."""
        try:
            if self.gstreamer_process and self.gstreamer_process.stdin:
                # 🔥 이미지 크기 변환 (640x480)
                frame_resized = cv2.resize(frame, (640, 480))

                # 🔥 GStreamer 파이프라인으로 전송
                self.gstreamer_process.stdin.write(frame_resized.tobytes())
                self.gstreamer_process.stdin.flush()  # 버퍼 비우기
        except Exception as e:
            print(f"GStreamer로 이미지 전송 실패: {e}")


    def stop(self):
        if self.video_writer:
            self.video_writer.release()
            print("GStreamer pipeline stopped.")


class GStreamerReceiver:
    def __init__(self, gstreamer_uri, width=640, height=480):
        self.gstreamer_uri = gstreamer_uri
        self.width = int(width)
        self.height = int(height)
        self.video_capture = None

    def start_pipeline(self):

        gst_pipeline = (
            f"srtsrc uri={self.gstreamer_uri} ! "
            f"tsdemux ! h264parse ! avdec_h264 ! "
            f"videoconvert ! appsink"
        )

        self.video_capture = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)

        if not self.video_capture.isOpened():
            print("Error: Unable to open GStreamer pipeline for receiving.")
            return False

        print("GStreamer receiver pipeline started successfully.")
        return True

    def receive_frame(self):
        if self.video_capture and self.video_capture.isOpened():
            ret, frame = self.video_capture.read()
            if ret:
                return frame
        return None

    def stop(self):
        if self.video_capture:
            self.video_capture.release()
            print("GStreamer receiver pipeline stopped.")

        """GStreamer 수신 파이프라인을 시작합니다."""
        gstreamer_command = [
            'gst-launch-1.0', '-v',
            'srtsink', f'uri={self.gstreamer_uri}',  # 🔥 수신은 srtsrc 사용
            '!', 'tsdemux',
            '!', 'h264parse',
            '!', 'avdec_h264',
            '!', 'videoconvert',
            '!', 'video/x-raw,format=BGR',
            '!', 'appsink'
        ]

        try:
            self.gstreamer_process = subprocess.Popen(
                gstreamer_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True
            )
            print(f"GStreamer 수신 파이프라인 시작됨. URI: {self.gstreamer_uri}")
        except Exception as e:
            print(f"GStreamer 수신 파이프라인 시작 실패: {e}")

    def receive_frame(self):
        """GStreamer로부터 프레임을 수신하고 OpenCV 이미지로 변환."""
        frame_data = self.gstreamer_process.stdout.read(640 * 480 * 3)  # 🔥 BGR 3채널 (640x480)
        if not frame_data:
            return None

        frame = np.frombuffer(frame_data, dtype=np.uint8).reshape((480, 640, 3))
        return frame

    def stop(self):
        """GStreamer 수신 파이프라인을 종료."""
        if self.gstreamer_process:
            self.gstreamer_process.terminate()
