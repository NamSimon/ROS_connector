import rclpy
from rclpy.node import Node
import importlib
from cv_bridge import CvBridge
from huge_data_connector.streaming_module.streaming_module import GStreamerSender, GStreamerReceiver

class Huge_data_Connector(Node):
    def __init__(self, platform, ros_topic, ros_type, mode, stream_topic=None, gstreamer_base_uri=None):
        super().__init__(f'ros2streaming_bridge_{stream_topic.replace("/", "_")}')

        self.platform = platform  # 플랫폼 이름 (edge 또는 user)
        self.ros2gstreamer_ros_topic = ros_topic  # ROS 토픽 이름
        self.ros2gstreamer_ros_type = ros_type  # ROS 메시지 타입
        self.mode = mode  # 모드 ('pub' 또는 'sub')
        self.stream_topic = stream_topic  # GStreamer 송신 스트림 토픽

        # GStreamer URI에서 토픽명으로 대체
       

        # CvBridge 객체 생성 (이미지 변환을 위해)
        self.bridge = CvBridge()

        # ROS 메시지 타입을 동적으로 로드
        self.ros_msg_type = self.get_ros_msg_type(self.ros2gstreamer_ros_type)

        # GStreamer 송신 및 수신 클래스 설정
        self.gstreamer_sender = None
        self.gstreamer_receiver = None

        # 플랫폼 및 모드 설정에 따라 동작 분기
        if self.platform == 'edge':
            if self.mode == 'sub':
                # ROS에서 수신한 데이터를 GStreamer로 송출
                self.get_logger().info(f"모드: sub - GStreamer 수신 후 ROS 퍼블리시")
                self.ros_subscription = self.create_subscription(
                    self.ros_msg_type, self.ros2gstreamer_ros_topic, self.ros_to_gstreamer_callback, 10)
                self.gstreamer_uri = f'srt://{gstreamer_base_uri}?streamid=publish:{stream_topic}&pkt_size=1316'
                # GStreamer 송신 파이프라인 시작
                self.gstreamer_sender = GStreamerSender(self.gstreamer_uri, self.stream_topic)
                self.gstreamer_sender.start_pipeline()

            elif self.mode == 'pub':
                # GStreamer에서 데이터를 수신한 후 ROS로 퍼블리시
                self.get_logger().info("모드: pub - ROS 구독 후 GStreamer로 스트림 전송")
                self.gstreamer_uri = f'srt://{gstreamer_base_uri}?streamid=read:{stream_topic}&pkt_size=1316'
                self.gstreamer_receiver = GStreamerReceiver(self.gstreamer_uri)
                self.gstreamer_receiver.start_pipeline()

                # 퍼블리셔 생성 (수신한 데이터를 ROS로 퍼블리시)
                self.ros_publisher = self.create_publisher(
                    self.ros_msg_type, self.ros2gstreamer_ros_topic, 10)

                # 주기적으로 GStreamer로부터 프레임을 수신하여 ROS로 퍼블리시
                self.timer = self.create_timer(0.1, self.receive_and_publish_frame)
            else:
                self.get_logger().error("올바르지 않은 모드 설정. 'pub' 또는 'sub'만 허용됩니다.")
                return

        elif self.platform == 'user':
            if self.mode == 'pub':
                self.get_logger().info("모드: pub - ROS 구독 후 GStreamer로 스트림 전송")
                self.ros_subscription = self.create_subscription(
                    self.ros_msg_type, self.ros2gstreamer_ros_topic, self.ros_to_gstreamer_callback, 10)
                self.gstreamer_uri = f'srt://{gstreamer_base_uri}?streamid=publish:{stream_topic}&pkt_size=1316'
                # GStreamer 송신 파이프라인 시작
                self.gstreamer_sender =  GStreamerSender(self.gstreamer_uri, self.stream_topic)
                self.gstreamer_sender.start_pipeline()

            elif self.mode == 'sub':
                self.get_logger().info(f"모드: sub - GStreamer 수신 후 ROS 퍼블리시")
                self.gstreamer_uri = f'srt://{gstreamer_base_uri}?streamid=read:{stream_topic}&pkt_size=1316'
                self.gstreamer_receiver = GStreamerReceiver(self.gstreamer_uri)
                self.gstreamer_receiver.start_pipeline()

                # 퍼블리셔 생성 (수신한 데이터를 ROS로 퍼블리시)
                self.ros_publisher = self.create_publisher(
                    self.ros_msg_type, self.ros2gstreamer_ros_topic, 10)

                # 주기적으로 GStreamer로부터 프레임을 수신하여 ROS로 퍼블리시
                self.timer = self.create_timer(0.1, self.receive_and_publish_frame)
            else:
                self.get_logger().error("올바르지 않은 모드 설정. 'pub' 또는 'sub'만 허용됩니다.")
                return
        else:
            self.get_logger().error("올바르지 않은 플랫폼 설정. 'edge' 또는 'user'만 허용됩니다.")
            return

    def get_ros_msg_type(self, ros_type_name):
        """ROS 메시지 타입을 동적으로 로드하는 함수."""
        if '/msg/' in ros_type_name:
            ros_type_name = ros_type_name.replace('/msg/', '/')

        package_name, msg_name = ros_type_name.split('/')

        try:
            module = importlib.import_module(f"{package_name}.msg")
            msg_type = getattr(module, msg_name, None)

            if msg_type is None:
                self.get_logger().error(f"Message type {ros_type_name} not found in package {package_name}")
                return None

            return msg_type
        except (ImportError, AttributeError) as e:
            self.get_logger().error(f"Failed to import ROS message type: {ros_type_name}. Error: {e}")
            return None

    def ros_to_gstreamer_callback(self, msg):
        """ROS에서 수신된 데이터를 GStreamer로 송출하는 콜백 함수."""
        self.get_logger().info(f"ROS에서 메시지 수신: {msg}")

        # ROS 메시지를 OpenCV 이미지로 변환
        frame = self.ros_msg_to_opencv(msg)
        if frame is not None and self.gstreamer_sender:
            # GStreamer 파이프라인으로 전송
            self.gstreamer_sender.send_frame(frame)

    def ros_msg_to_opencv(self, msg):
        """ROS 메시지를 OpenCV 이미지로 변환."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            return cv_image
        except Exception as e:
            self.get_logger().error(f"ROS 메시지를 OpenCV로 변환 실패: {e}")
            return None

    def receive_and_publish_frame(self):
        """GStreamer에서 수신한 프레임을 ROS로 퍼블리시."""
        if self.gstreamer_receiver:
            frame = self.gstreamer_receiver.receive_frame()
            if frame is not None:
                ros_msg = self.opencv_to_ros_msg(frame)
                if ros_msg:
                    self.get_logger().info("GStreamer에서 프레임 수신, ROS로 퍼블리시")
                    self.ros_publisher.publish(ros_msg)

    def opencv_to_ros_msg(self, frame):
        """OpenCV 이미지를 ROS Image 메시지로 변환."""
        try:
            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            return ros_image
        except Exception as e:
            self.get_logger().error(f"OpenCV 이미지를 ROS 메시지로 변환 실패: {e}")
            return None

    def stop(self):
        """노드를 종료할 때 GStreamer 파이프라인을 정리."""
        if self.gstreamer_sender:
            self.gstreamer_sender.stop()
        if self.gstreamer_receiver:
            self.gstreamer_receiver.stop()


