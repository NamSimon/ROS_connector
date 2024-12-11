import paho.mqtt.client as mqtt

class MQTTClient:
    def __init__(self, broker_host, broker_port, mqtt_topic, on_message_callback):
        # MQTT 브로커 정보와 토픽 설정
        self.broker_host = broker_host
        self.broker_port = broker_port
        self.mqtt_topic = mqtt_topic

        # 콜백을 외부에서 주입받음
        self.on_message_callback = on_message_callback

        # MQTT 클라이언트 설정
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        self.mqtt_client.connect(self.broker_host, self.broker_port, 60)
        self.mqtt_client.loop_start()

    def on_connect(self, client, userdata, flags, rc):
        """MQTT 브로커 연결 시 콜백 함수."""
        if rc == 0:
            print("MQTT 브로커에 연결되었습니다.")
        else:
            print(f"MQTT 브로커에 연결 실패: {rc}")

    def on_message(self, client, userdata, msg):
        """MQTT 메시지를 수신할 때의 콜백 함수."""
        print(f"MQTT에서 데이터 수신: {msg.topic}")
        # 데이터를 바이너리로 처리 (직렬화된 데이터를 그대로 사용)
        self.on_message_callback(msg.payload)

    def subscribe(self):
        """MQTT 구독을 시작."""
        self.mqtt_client.subscribe(self.mqtt_topic)

    def publish(self, data):
        """MQTT로 데이터를 퍼블리시."""
        self.mqtt_client.publish(self.mqtt_topic, data)

    def stop(self):
        """MQTT 루프를 중지."""
        self.mqtt_client.loop_stop()