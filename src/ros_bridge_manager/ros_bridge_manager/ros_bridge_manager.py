import rclpy
from rclpy.executors import MultiThreadedExecutor
from short_topic_connector.short_topic_connector import Short_Topic_Connector
from huge_data_connector.huge_data_connector import Huge_data_Connector  # Assuming this is in a file called huge_data_connector.py
import os

class ROSBridgeManager:
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
                platform=config.get('platform', os.getenv('PLATFORM')),
                ros_topic=config['ros_topic'],
                ros_type=config['ros_type'],
                mode=config['mode']
            )
        # Huge_data_Connector 노드 생성
        elif config['type'] == 'huge_topic':
            bridge_node = Huge_data_Connector(
                platform=config.get('platform', os.getenv('PLATFORM')),
                ros_topic=config['ros_topic'],
                ros_type=config['ros_type'],
                mode=config['mode'],
                gstreamer_base_uri=f"{os.getenv('MEDIA_HOST')}:{os.getenv('MEDIA_PORT')}"
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
    bridges_config_str = os.getenv('ROS_CONFIG')
    
    try:
        bridges_config = json.loads(bridges_config_str)
        logging.info(f"브리지 설정 로드 완료: {bridges_config}")
    except json.JSONDecodeError as e:
        logging.error(f"ROS_CONFIG JSON 파싱 에러: {e}")
        raise ValueError(f"ROS_CONFIG JSON 파싱 에러: {e}")

    bridge_manager = ROSBridgeManager(bridges_config)
    bridge_manager.spin()

if __name__ == '__main__':
    main()