import rclpy
from rclpy.node import Node
import importlib
from service_connector.mqtt_module.mqtt_module import MQTTClient
from rclpy_message_converter import json_message_converter
from rclpy.executors import ExternalShutdownException

class Service_Connector(Node):
    def __init__(self, broker_host,broker_port,ros_service,ros_type,mode,platform):
        super().__init__(f'service_bridge_{ros_service.replace("/", "_")}')

        self.platform=platform
        self.ros2mqtt_ros_service = ros_service
        self.ros2mqtt_ros_type = ros_type
        self.mode = mode
        self.ros_srv_type = self.get_ros_srv_type(self.ros2mqtt_ros_type)
        self.ros_response = None
        self.ros_request = None
        self.mqtt_client_request = MQTTClient(
            broker_host=broker_host,
            broker_port=broker_port,
            mqtt_topic=ros_service+'request',
            on_message_callback=self.on_mqtt_message_request_received
        )
        self.mqtt_client_response = MQTTClient(
            broker_host=broker_host,
            broker_port=broker_port,
            mqtt_topic=ros_service+'response',
            on_message_callback=self.on_mqtt_message_response_received
        )
        
        if self.platform=='edge':
            if self.mode == 'client':
                self.mqtt_client_request.subscribe()
                self.create_client(self.ros_srv_type,self.ros2mqtt_ros_service)
                while not self.ros_client.wait_for_service(timeout_sec=1.0):
                    self.get_logger().info('service not available, waiting again...')
                self.future=self.cli.call_async(self.ros_request)
                rclpy.spin_until_future_complete(self, self.future)
                self.ros_response=self.future.result()
                json_response=json_message_converter.convert_ros_message_to_json(self.ros_response)
                self.mqtt_client_response.publish(json_response)
           
            elif self.mode == 'server':
                self.srv=self.create_service(self.ros_srv_type,self.ros2mqtt_ros_service,self.ros_service_server_callback)
            else:
                self.get_logger().error("올바르지 않은 모드 설정. 'server' 또는 'client'만 허용됩니다.")
                return
            
                    
        if self.platform=='user':
            if self.mode == 'client':
                self.srv=self.create_service(self.ros_srv_type,self.ros2mqtt_ros_service,self.ros_service_server_callback)
            elif self.mode == 'server':
                self.mqtt_client_request.subscribe()
                self.create_client(self.ros_srv_type,self.ros2mqtt_ros_service)
                while not self.ros_client.wait_for_service(timeout_sec=1.0):
                    self.get_logger().info('service not available, waiting again...')
                self.future=self.cli.call_async(self.ros_request)
                rclpy.spin_until_future_complete(self, self.future)
                self.ros_response=self.future.result()
                json_response=json_message_converter.convert_ros_message_to_json(self.ros_response)
                self.mqtt_client_response.publish(json_response)
            else:
                self.get_logger().error("올바르지 않은 모드 설정. 'server' 또는 'client'만 허용됩니다.")
                return
        
    def ros_service_server_callback(self,request,response):
            json_request=json_message_converter.convert_ros_message_to_json(request)
            self.mqtt_client_request.publish(json_request)
            self.mqtt_client_response.subscribe()
            response=self.ros_response
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
            self.ros_srv_type, json_response,'response'
        )
        self.get_logger().info("ROS 메시지로 변환 성공.")
    
    def on_mqtt_message_request_received(self, json_request):
        self.get_logger().info(f"MQTT 메시지 수신: {json_request}")
        
        self.get_request = json_message_converter.convert_json_to_ros_message(
            self.ros_srv_type, json_request,'request'
        )
        self.get_logger().info("ROS 메시지로 변환 성공.")