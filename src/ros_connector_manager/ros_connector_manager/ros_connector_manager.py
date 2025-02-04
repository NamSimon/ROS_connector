import rclpy
from rclpy.executors import MultiThreadedExecutor
from short_topic_connector.short_topic_connector import Short_Topic_Connector
from huge_data_connector.huge_data_connector import Huge_data_Connector  # Assuming this is in a file called huge_data_connector.py
from service_connector.service_connector import Service_Connector
import os
import ast  # JSON 대신 Python 형식 문자열 파싱용

class ROSConnectorManager:
    def __init__(self, bridges_config):
        self.bridges = []
        self.executor = MultiThreadedExecutor()

        # 여러 브리지 설정 정의
        for config in bridges_config:
            self.create_bridge(config)

    def create_bridge(self, config):
        # Short_Topic_Connector 노드 생성
        if config['type'] == 'short_topic':
            bridge_node = Short_Topic_Connector(
                broker_host=os.getenv('BROKER_HOST'),
                broker_port=int(os.getenv('BROKER_PORT')),
                ros_topic=config['ros_topic'],
                ros_type=config['ros_type'],
                mode=config['mode']
            )
        # Huge_data_Connector 노드 생성
        if config['type'] == 'huge_topic':
            bridge_node = Huge_data_Connector(
                ros_topic=config['ros_topic'],
                ros_type=config['ros_type'],
                mode=config['mode'],
                gstreamer_base_uri=f"{os.getenv('MEDIA_HOST')}:{os.getenv('MEDIA_PORT')}"
            )
        if config['type']== 'service':
            bridge_node=Service_Connector(
                broker_host=os.getenv('BROKER_HOST'),
                broker_port=int(os.getenv('BROKER_PORT')),
                ros_service=config['ros_service'],
                ros_type=config['ros_type'],
                mode=config['mode']
            )

        self.bridges.append(bridge_node)
        self.executor.add_node(bridge_node)

    def spin(self):
        try:
            self.executor.spin()
        except KeyboardInterrupt:
            pass
        finally:
            self.shutdown()

    def shutdown(self):
        # 모든 노드와 MQTT 클라이언트를 종료
        for bridge in self.bridges:
            if isinstance(bridge, Short_Topic_Connector):
                bridge.mqtt_client.stop()
            bridge.destroy_node()
        rclpy.shutdown()

def main():
    rclpy.init()

    # # 환경 변수에서 ROS_CONFIG 가져오기
    bridges_config_str = os.getenv('ROS_CONFIG')
    if not bridges_config_str:
        raise ValueError("ROS_CONFIG 환경 변수가 설정되지 않았습니다.")

    # Python 형식 문자열을 파싱
    try:
        bridges_config = ast.literal_eval(bridges_config_str)  # JSON 대신 Python 파싱
    except (ValueError, SyntaxError) as e:
        print(f"ROS_CONFIG 값: {bridges_config_str}")
        raise ValueError(f"ROS_CONFIG 파싱 에러: {e}")
    # bridges_config 예시 파일
    
    # bridges_config=[
    #         {
    #             "type": "short_topic",
    #             "ros_topic": "/example/short_topic",
    #             "ros_type": "std_msgs/msg/String",
    #             "mode": "pub",
    #             "platform": "edge"
    #         },
    #         {
    #             "type": "huge_topic",
    #             "ros_topic": "/example/huge_topic",
    #             "ros_type": "sensor_msgs/msg/Image",
    #             "mode": "sub",
    #             "platform": "edge"
    #         },
    #         {
    #             "type": "service",
    #             "ros_service": "empty_service",
    #             "ros_type": "std_srvs/srv/Empty",
    #             "mode": "client",
    #             "platform": "user"
    #         }
    #     ]
    
    
    # ROSBridgeManager 실행
    connector_manager = ROSConnectorManager(bridges_config)
    connector_manager.spin()

if __name__ == '__main__':
    main()