import rclpy
from rclpy.node import Node
import importlib
from rclpy_message_converter import json_message_converter
from .mqtt_module.mqtt_module import MQTTClient  # MQTTClient는 여기서 선언

class Short_Topic_Connector(Node):
    def __init__(self, broker_host, broker_port, mqtt_topic, platform, ros_topic, ros_type, mode):
        super().__init__(f'short_topic_bridge_{ros_topic}')

        self.platform = platform  # 플랫폼 이름
        self.ros2mqtt_ros_topic = ros_topic  # ROS 토픽
        self.ros2mqtt_ros_type = ros_type  # ROS 메시지 타입
        self.mode = mode  # 모드 ('pub' 또는 'sub')

        # ROS 메시지 타입을 동적으로 로드
        self.ros_msg_type = self.get_ros_msg_type(self.ros2mqtt_ros_type)

        # MQTT 클라이언트 설정
        self.mqtt_client = MQTTClient(
            broker_host=broker_host,
            broker_port=broker_port,
            mqtt_topic=mqtt_topic,
            on_message_callback=self.on_mqtt_message_received
        )

        # 플랫폼 및 모드 설정에 따라 동작 분기
        if self.platform == 'edge':
            if self.mode == 'pub':
                self.mqtt_client.subscribe()  # MQTT 구독 시작
                self.get_logger().info("모드: pub - MQTT 구독, ROS 퍼블리시")
                self.ros_publisher = self.create_publisher(
                    self.ros_msg_type,
                    self.ros2mqtt_ros_topic,
                    1
                )

            elif self.mode == 'sub':
                self.get_logger().info("모드: sub - MQTT 퍼블리시, ROS 구독")
                self.ros_subscription = self.create_subscription(
                    self.ros_msg_type,
                    self.ros2mqtt_ros_topic,
                    self.ros_to_mqtt_callback,
                    1
                )
            else:
                self.get_logger().error("올바르지 않은 모드 설정. 'pub' 또는 'sub'만 허용됩니다.")
                return
            
        elif self.platform == 'user':
            if self.mode == 'pub':
                self.get_logger().info("모드: pub - MQTT 구독, ROS 퍼블리시")
                self.ros_subscription = self.create_subscription(
                    self.ros_msg_type,
                    self.ros2mqtt_ros_topic,
                    self.ros_to_mqtt_callback,
                    1
                )
                
            elif self.mode == 'sub':
                self.mqtt_client.subscribe()  # MQTT 구독 시작
                self.get_logger().info("모드: sub - MQTT 퍼블리시, ROS 구독")
                self.ros_publisher = self.create_publisher(
                    self.ros_msg_type,
                    self.ros2mqtt_ros_topic,
                    1
                )

            else:
                self.get_logger().error("올바르지 않은 모드 설정. 'pub' 또는 'sub'만 허용됩니다.")
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
            
            if not hasattr(msg_type, '_TYPE_SUPPORT'):
                self.get_logger().error(f"Message type {ros_type_name} is missing _TYPE_SUPPORT. Ensure it's a valid ROS 2 message.")
                return None

            return msg_type
        except (ImportError, AttributeError) as e:
            self.get_logger().error(f"Failed to import ROS message type: {ros_type_name}. Error: {e}")
            return None

    def ros_to_mqtt_callback(self, msg):
        """ROS에서 수신된 데이터를 MQTT로 퍼블리시하는 콜백 함수."""
        self.get_logger().info(f"ROS에서 데이터 수신: {msg}")

        # ROS 메시지를 JSON으로 직렬화하여 MQTT로 전송
        json_msg = self.ros_msg_to_json(msg)
        if json_msg is not None:
            self.mqtt_client.publish(json_msg)

    def ros_msg_to_json(self, msg):
        """ROS 메시지를 JSON으로 직렬화하는 함수."""
        try:
            # ROS 메시지를 딕셔너리로 변환 후 JSON 직렬화
            return json_message_converter.convert_ros_message_to_json(msg)
        except (TypeError, ValueError) as e:
            self.get_logger().error(f"ROS 메시지 JSON 직렬화 오류: {e}")
            return None

    def on_mqtt_message_received(self, json_msg):
        """MQTT에서 수신된 JSON 메시지를 ROS로 퍼블리시."""
        self.get_logger().info("MQTT 메시지를 ROS로 퍼블리시합니다.")

        # ROS 메시지로 변환하여 퍼블리시
        ros_msg = json_message_converter.convert_json_to_ros_message(self.ros2mqtt_ros_type, json_msg)
        if ros_msg:
            self.ros_publisher.publish(ros_msg)