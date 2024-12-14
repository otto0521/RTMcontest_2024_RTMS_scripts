import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import asyncio
import json
from threading import Thread
import importlib
import config
import uuid
import os
import websockets  

UUID_FILE_PATH = os.path.join(os.path.dirname(__file__), "robot_uuid.txt")

def import_message_type(type_str):
    module_name, class_name = type_str.rsplit(".", 1)
    module = importlib.import_module(module_name)
    return getattr(module, class_name)

class RosStatePublisher(Node):
    def __init__(self):
        super().__init__('ros_state_publisher')

        self.robot_id = config.ROBOT_ID  
        self.owner = config.OWNER_NAME 

        self.unique_id = self.get_or_create_uuid()
        self.get_logger().info(f"Using UUID: {self.unique_id}")

        # self.websocket_url = f"wss://monitoring.ddns.net/ws/robots/?unique_robot_id={self.unique_id}"
        self.websocket_url = f"ws://localhost:8000//ws/robots/?unique_robot_id={self.unique_id}"

        self.state_keys = {key: "unknown" for key in [sub["state_key"] for sub in config.TOPIC_SUBSCRIPTIONS]}
        self.states = self.state_keys.copy() 
        qos_profile = QoSProfile(depth=10)

        for subscription in config.TOPIC_SUBSCRIPTIONS:
            message_type = import_message_type(subscription["message_type"])  
            callback_function = getattr(config, subscription["callback"])  

            self.create_subscription(
                message_type,
                subscription["topic"],
                lambda msg, key=subscription["state_key"], callback=callback_function: self.dynamic_callback(callback, msg),
                qos_profile
            )
        
        self.running = True
        self.send_thread = Thread(target=self.start_websocket_loop, daemon=True)
        self.send_thread.start()

    def get_or_create_uuid(self):

        if os.path.exists(UUID_FILE_PATH):
            with open(UUID_FILE_PATH, "r") as file:
                unique_id = file.read().strip()
                self.get_logger().info(f"Loaded UUID from file: {unique_id}")
                return unique_id
        else:
            unique_id = uuid.uuid4().hex
            with open(UUID_FILE_PATH, "w") as file:
                file.write(unique_id)
                self.get_logger().info(f"Generated and saved new UUID: {unique_id}")
            return unique_id


    def dynamic_callback(self, user_callback, msg):
        user_callback(self.states, msg) 
    
    def generate_payload(self):

        return {
            "robot_id": self.robot_id,
            "owner": self.owner,
            "unique_id": self.unique_id,
            "state": self.states,
        }

    async def send_state(self):
        while self.running:
            try:
                self.get_logger().info(f"Attempting WebSocket connection to {self.websocket_url}")
                async with websockets.connect(self.websocket_url) as websocket:
                    self.get_logger().info("WebSocket connected.")
                    while self.running:
                        try:

                            # message = await websocket.recv()  # メッセージを受信
                            # data = json.loads(message)
                            # if "ping" in data:  # pingメッセージを受け取った場合
                            #     self.get_logger().info("Received ping, sending pong.")
                            #     await websocket.send(json.dumps({"pong": True}))  # pongを送信

                            # メッセージ送受信デバッグ
                            payload = self.generate_payload()
                            self.get_logger().info(f"Sending payload: {payload}")
                            await websocket.send(json.dumps(payload))
                            self.get_logger().info("Payload sent successfully.")
                            await asyncio.sleep(0.2)  # 状態更新の間隔
                        except Exception as e:
                            self.get_logger().error(f"Error during WebSocket send/recv: {e}")
                            break  # ループを抜けて再接続を試みる
            except Exception as e:
                self.get_logger().error(f"WebSocket connection error: {e}")
                await asyncio.sleep(5)  # 再接続待機


    def start_websocket_loop(self):
        asyncio.run(self.send_state())

    def on_shutdown(self):

        self.get_logger().info("Shutting down...")
        self.running = False  
        if self.send_thread.is_alive():
            self.send_thread.join()  
        self.get_logger().info("WebSocket thread stopped.")

def main(args=None):

    rclpy.init(args=args) 
    node = RosStatePublisher()
    try:
        rclpy.spin(node) 
    except KeyboardInterrupt:
        pass 
    finally:
        node.on_shutdown() 
        node.destroy_node() 
        rclpy.shutdown()  

if __name__ == '__main__':
    main()
