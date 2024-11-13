import cv2

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
            self.video_writer.write(frame)
        else:
            print("Error: VideoWriter is not opened.")

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