import rclpy
from rclpy.executors import MultiThreadedExecutor
from short_topic_connector import Short_Topic_Connector
from huge_data_connector import Huge_data_Connector  # Assuming this is in a file called huge_data_connector.py
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
                mqtt_topic=config['mqtt_topic'],
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
                stream_topic=config['stream_topic'],
                gstreamer_base_uri=config.get('gstreamer_base_uri', os.getenv('GSTREAMER_BASE_URI'))
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

    bridges_config = [
        {
            'type': 'short_topic',
            'ros_topic': '/topic_1',
            'ros_type': 'std_msgs/String',
            'mode': 'pub',
            'mqtt_topic': 'mqtt/topic_1',
        },
        {
            'type': 'short_topic',
            'ros_topic': '/topic_2',
            'ros_type': 'sensor_msgs/Imu',
            'mode': 'sub',
            'mqtt_topic': 'mqtt/topic_2',
        },
        {
            'type': 'huge_topic',
            'ros_topic': '/camera/image_raw',
            'ros_type': 'sensor_msgs/Image',
            'mode': 'pub',
            'stream_topic': 'stream_topic_1',
            'gstreamer_base_uri': 'rtsp://example.com/live',
        },
        # 추가 브리지 설정 가능
    ]

    bridge_manager = ROSBridgeManager(bridges_config)
    bridge_manager.spin()

if __name__ == '__main__':
    main()