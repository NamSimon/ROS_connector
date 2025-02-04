import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
import importlib
from cv_bridge import CvBridge
from huge_data_connector.streaming_module.streaming_module import GStreamerSender, GStreamerReceiver
import cv2
import os
class Huge_data_Connector(Node):
    def __init__(self, ros_topic, ros_type, mode, gstreamer_base_uri=None, width=640, height=480):
        super().__init__(f'ros2streaming_bridge_{ros_topic.replace("/", "_")}')

        self.platform = os.getenv('PLATFORM')
        self.ros2gstreamer_ros_topic = ros_topic
        self.ros2gstreamer_ros_type = ros_type
        self.mode = mode
        self.gstreamer_base_uri = gstreamer_base_uri
        self.width = width
        self.height = height

        self.bridge = CvBridge()
        self.get_logger().info("CvBridge 생성 완료")
        self.ros_msg_type = self.get_ros_msg_type(self.ros2gstreamer_ros_type)
        self.get_logger().info("ros_msg_type 생성 완료")
        self.gstreamer_sender = None
        self.gstreamer_receiver = None
        self.should_stop = threading.Event()
        self.get_logger().info("GStreamer 파이프라인 생성 ready")
        # GStreamer 파이프라인 설정
        self.create_gstreamer_pipeline()
        self.get_logger().info("GStreamer 파이프라인 생성 완료")
        # ROS 2 스피닝 스레드 시작
        self.spin_thread = threading.Thread(target=self.start_spin)
        self.spin_thread.start()

        if self.platform == 'edge':
            if self.mode == 'sub':
                self.start_ros_subscription()
                self.start_ros_to_gstreamer_thread()
            elif self.mode == 'pub':
                self.start_ros_publisher()
                self.start_gstreamer_to_ros_thread()
        elif self.platform == 'user':
            if self.mode == 'pub':
                self.start_ros_subscription()
                self.start_ros_to_gstreamer_thread()
                self.get_logger().info("Success")
            elif self.mode == 'sub':
                self.start_ros_publisher()
                self.start_gstreamer_to_ros_thread()
                self.get_logger().info("Success")

    def create_gstreamer_pipeline(self):
        self.get_logger().info("GStreamer 파이프라인 생성 중1")
        if self.platform == 'edge':
            if self.mode == 'sub':
                self.setup_gstreamer_sender()
            elif self.mode == 'pub':
                self.setup_gstreamer_receiver()
        if self.platform=='user':
            if self.mode == 'pub':
                self.setup_gstreamer_sender()
            elif self.mode == 'sub':
                self.get_logger().info("GStreamer 파이프라인 생성 중2")
                self.setup_gstreamer_receiver()

    def setup_gstreamer_sender(self):
        self.gstreamer_uri = f'srt://{self.gstreamer_base_uri}?streamid=uplive.sls/live{self.ros2gstreamer_ros_topic}'
        # width와 height 값을 전달
        self.gstreamer_sender = GStreamerSender(self.gstreamer_uri, width=self.width, height=self.height)
        self.gstreamer_sender.start_pipeline()

    def setup_gstreamer_receiver(self):
        self.get_logger().info("GStreamer 파이프라인 생성 중3")
        self.gstreamer_uri = f'srt://{self.gstreamer_base_uri}?streamid=live.sls/live{self.ros2gstreamer_ros_topic}'
        self.gstreamer_receiver = GStreamerReceiver(self.gstreamer_uri)
        self.gstreamer_receiver.start_pipeline()

    def start_ros_subscription(self):
        self.ros_subscription = self.create_subscription(
            self.ros_msg_type, self.ros2gstreamer_ros_topic, self.ros_to_gstreamer_callback, 2)

    def start_ros_publisher(self):
        self.ros_publisher = self.create_publisher(
            self.ros_msg_type, self.ros2gstreamer_ros_topic, 2)

    def start_ros_to_gstreamer_thread(self):
        self.ros_to_gstreamer_thread = threading.Thread(target=self.ros_to_gstreamer_worker)
        self.ros_to_gstreamer_thread.start()

    def start_gstreamer_to_ros_thread(self):
        self.gstreamer_to_ros_thread = threading.Thread(target=self.receive_and_publish_frame_worker)
        self.gstreamer_to_ros_thread.start()

    def start_spin(self):
        """ROS 2 내장 스피닝 기능 사용."""
        rclpy.spin(self)

    def ros_to_gstreamer_worker(self):
        while rclpy.ok() and not self.should_stop.is_set():
            try:
                self.receive_and_publish_frame()
            except Exception as e:
                self.get_logger().error(f"작업 중 오류 발생: {e}")

    def receive_and_publish_frame_worker(self):
        while rclpy.ok() and not self.should_stop.is_set():
            self.receive_and_publish_frame()

    def get_ros_msg_type(self, ros_type_name):
        if '/msg/' in ros_type_name:
            ros_type_name = ros_type_name.replace('/msg/', '/')
        package_name, msg_name = ros_type_name.split('/')
        try:
            module = importlib.import_module(f"{package_name}.msg")
            return getattr(module, msg_name, None)
        except (ImportError, AttributeError) as e:
            self.get_logger().error(f"ROS 메시지 타입 가져오기 실패: {e}")
            return None

    def ros_to_gstreamer_callback(self, msg):
        cv_image = self.ros_msg_to_opencv(msg)
        if cv_image is not None and self.gstreamer_sender:
            try:
                self.gstreamer_sender.send_frame(cv_image)
            except Exception as e:
                self.get_logger().error(f"GStreamer 전송 실패: {e}")

    def ros_msg_to_opencv(self, msg):
        try:
            return self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"ROS 메시지 변환 실패: {e}")
            return None

    def receive_and_publish_frame(self):
        if self.gstreamer_receiver:
            frame = self.gstreamer_receiver.receive_frame()
            if frame is not None:
                ros_msg = self.opencv_to_ros_msg(frame)
                if ros_msg:
                    self.ros_publisher.publish(ros_msg)

    def opencv_to_ros_msg(self, frame):
        try:
            return self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"OpenCV 변환 실패: {e}")
            return None

    def stop(self):
        self.should_stop.set()
        if self.gstreamer_sender:
            self.gstreamer_sender.stop()
        if self.gstreamer_receiver:
            self.gstreamer_receiver.stop()