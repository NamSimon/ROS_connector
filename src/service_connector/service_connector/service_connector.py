import rclpy
from rclpy.node import Node  # Node를 명시적으로 임포트
import importlib
import threading  # threading 모듈 추가
from service_connector.mqtt_module.mqtt_module import MQTTClient
from rclpy_message_converter import json_message_converter
from rclpy.executors import ExternalShutdownException

class Service_Connector(Node):
    def __init__(self, broker_host, broker_port, ros_service, ros_type, mode, platform):
        super().__init__(f'service_bridge_{ros_service.replace("/", "_")}')

        self.platform = platform
        self.ros2mqtt_ros_service = ros_service
        self.ros2mqtt_ros_type = ros_type
        self.mode = mode
        self.ros_srv_type = self.get_ros_srv_type(self.ros2mqtt_ros_type)
        self.ros_response = None
        self.ros_request = None

        # MQTT 클라이언트 초기화
        self.mqtt_client_request = MQTTClient(
            broker_host=broker_host,
            broker_port=broker_port,
            mqtt_topic=ros_service + 'request',
            on_message_callback=self.on_mqtt_message_request_received
        )
        self.mqtt_client_response = MQTTClient(
            broker_host=broker_host,
            broker_port=broker_port,
            mqtt_topic=ros_service + 'response',
            on_message_callback=self.on_mqtt_message_response_received
        )

        # Edge 플랫폼 설정
        if self.platform == 'edge':
            if self.mode == 'client':
                self.mqtt_client_request.subscribe()
                self.ros_client = self.create_client(self.ros_srv_type, self.ros2mqtt_ros_service)
                while not self.ros_client.wait_for_service(timeout_sec=1.0):
                    self.get_logger().info('서비스가 사용 가능하지 않습니다. 다시 대기 중...')

            elif self.mode == 'server':
                self.mqtt_client_response.subscribe()
                self.srv = self.create_service(self.ros_srv_type, self.ros2mqtt_ros_service, self.ros_service_server_callback)
            else:
                self.get_logger().error("올바르지 않은 모드 설정. 'server' 또는 'client'만 허용됩니다.")
                return

        # User 플랫폼 설정
        elif self.platform == 'user':
            if self.mode == 'client':
                self.mqtt_client_response.subscribe()
                self.srv = self.create_service(self.ros_srv_type, self.ros2mqtt_ros_service, self.ros_service_server_callback)
            elif self.mode == 'server':
                self.mqtt_client_request.subscribe()
                self.ros_client = self.create_client(self.ros_srv_type, self.ros2mqtt_ros_service)
                while not self.ros_client.wait_for_service(timeout_sec=1.0):
                    self.get_logger().info('서비스가 사용 가능하지 않습니다. 다시 대기 중...')
            else:
                self.get_logger().error("올바르지 않은 모드 설정. 'server' 또는 'client'만 허용됩니다.")
                return

    def ros_service_server_callback(self, request, response):
            self.get_logger().info("ROS 서비스 요청 수신")

            # 요청을 JSON으로 변환하고 MQTT로 발행
            json_request = json_message_converter.convert_ros_message_to_json(request)
            self.mqtt_client_request.publish(json_request)
            
            # MQTT 응답 대기 (동기화)
            self.ros_response = None  # 이전 값 초기화
            event = threading.Event()  # 동기화 이벤트 객체

            def on_response(json_response):
                """MQTT로부터 응답을 수신했을 때 실행되는 콜백 함수"""
                self.get_logger().info("MQTT 응답 수신")
                self.ros_response = json_message_converter.convert_json_to_ros_message(
                    self.ros_srv_type.Response, json_response, 'response'
                )
                event.set()  # 이벤트 설정으로 대기 해제

            # MQTT 응답 콜백 설정
            self.mqtt_client_response.on_message_callback = on_response

            # 응답 대기
            self.get_logger().info("MQTT 응답 대기 중...")
            if not event.wait(timeout=5.0):  # 5초 동안 응답을 기다림
                self.get_logger().error("MQTT 응답 타임아웃")
                return response  # 빈 응답 반환

            # ROS 서비스의 response 객체 설정
            if self.ros_response:  # MQTT 응답이 존재하면
                # 응답 필드를 명확하게 설정 (사용 중인 서비스 타입에 맞게 수정 필요)
                for field_name in self.ros_srv_type.Response.get_fields_and_field_types().keys():
                    setattr(response, field_name, getattr(self.ros_response, field_name))
                self.get_logger().info("ROS 응답 객체 반환 준비 완료")
            else:
                self.get_logger().error("ROS 응답이 비어있습니다.")

            return response

    def get_ros_srv_type(self, ros_type_name):
        """ROS 메시지 타입을 동적으로 로드하는 함수."""
        if '/srv/' in ros_type_name:
            ros_type_name = ros_type_name.replace('/srv/', '/')

        package_name, srv_name = ros_type_name.split('/')

        try:
            module = importlib.import_module(f"{package_name}.srv")
            srv_type = getattr(module, srv_name, None)

            if srv_type is None:
                self.get_logger().error(f"Message type {ros_type_name} not found in package {package_name}")
                return None

            if not hasattr(srv_type, '_TYPE_SUPPORT'):
                self.get_logger().error(f"Message type {ros_type_name} is missing _TYPE_SUPPORT. Ensure it's a valid ROS 2 message.")
                return None

            return srv_type
        except (ImportError, AttributeError) as e:
            self.get_logger().error(f"Failed to import ROS message type: {ros_type_name}. Error: {e}")
            return None

    def on_mqtt_message_response_received(self, json_response):
        """MQTT에서 수신된 JSON 메시지를 ROS로 변환하여 응답."""
        self.get_logger().info(f"MQTT 메시지 수신: {json_response}")
        self.ros_response = json_message_converter.convert_json_to_ros_message(
            self.ros_srv_type.Response, json_response, 'response'
        )
        
        self.get_logger().info("ROS 메시지로 변환 성공.")

    def on_mqtt_message_request_received(self, json_request):
        """MQTT로부터 요청 수신 및 ROS 서비스 클라이언트 호출."""
        try:
            self.get_logger().info(f"MQTT 요청 메시지 수신: {json_request}")
            self.ros_request = json_message_converter.convert_json_to_ros_message(
                self.ros_srv_type.Request, json_request, 'request'
            )
      
            self.get_logger().info("ROS 메시지로 변환 성공.")
            self.future = self.ros_client.call_async(self.ros_request)
            self.future.add_done_callback(self.on_service_response_received)

        except Exception as e:
            self.get_logger().error(f"ROS 서비스 호출 중 오류 발생: {e}")            

    def on_service_response_received(self, future):
        """ROS 서비스 응답을 처리하고 MQTT로 발행."""
        try:
            self.ros_response = future.result()
            self.get_logger().info(f"ROS 서비스 응답 수신: {self.ros_response}")
            json_response = json_message_converter.convert_ros_message_to_json(self.ros_response)
            self.mqtt_client_response.publish(json_response)
            self.get_logger().info("MQTT 응답 메시지 발행 성공.")
            
        except Exception as e:
            self.get_logger().error(f"MQTT 응답 메시지 발행 중 오류 발생: {e}")
            