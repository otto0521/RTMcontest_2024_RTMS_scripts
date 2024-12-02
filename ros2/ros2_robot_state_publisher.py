import rclpy
from rclpy.node import Node
import asyncio
import json
from threading import Thread
import importlib
import config
import uuid
import os
import websockets  # WebSocketモジュール

# UUIDを保存するファイルのパス（スクリプトと同じディレクトリに保存）
UUID_FILE_PATH = os.path.join(os.path.dirname(__file__), "robot_uuid.txt")

# 動的にメッセージ型をインポートする関数
def import_message_type(type_str):
    module_name, class_name = type_str.rsplit(".", 1)
    module = importlib.import_module(module_name)
    return getattr(module, class_name)

class RosStatePublisher(Node):
    def __init__(self):
        super().__init__('ros_state_publisher')

        # ユーザー設定を読み込み
        self.robot_id = config.ROBOT_ID  # ロボットID
        self.owner = config.OWNER_NAME  # オーナー名

        # UUIDの読み込みまたは生成
        self.unique_id = self.get_or_create_uuid()
        self.get_logger().info(f"Using UUID: {self.unique_id}")

        # WebSocket URLの設定
        self.websocket_url = f"ws://127.0.0.1:8000/ws/robots/?unique_robot_id={self.unique_id}"

        # 状態キーと初期値を設定
        self.state_keys = {key: "unknown" for key in [sub["state_key"] for sub in config.TOPIC_SUBSCRIPTIONS]}
        self.states = self.state_keys.copy()  # statesはstate_keysのコピーで初期化

        # トピック購読設定を動的に読み込む
        for subscription in config.TOPIC_SUBSCRIPTIONS:
            message_type = import_message_type(subscription["message_type"])  # メッセージ型を動的にインポート
            callback_function = getattr(config, subscription["callback"])  # コールバック関数を設定

            # lambda を使って dynamic_callback を呼び出す
            self.create_subscription(
                message_type,
                subscription["topic"],
                lambda msg, key=subscription["state_key"], callback=callback_function: self.dynamic_callback(key, callback, msg),
                10
            )

        # WebSocket送信用のスレッドを開始
        self.running = True
        self.send_thread = Thread(target=self.start_websocket_loop, daemon=True)
        self.send_thread.start()

    def get_or_create_uuid(self):
        """
        UUIDをファイルから読み込むか、新規作成して保存する。
        """
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

    # 動的コールバック関数
    def dynamic_callback(self, state_key, user_callback, msg):
        """
        状態の更新処理を行い、更新前後の状態をログに出力
        """
        self.get_logger().info(f"Before update {state_key}: {self.states.get(state_key, 'Not Found')}")  # 更新前の状態をログ出力
        user_callback(self.states, msg)  # コールバックで状態を更新
        self.get_logger().info(f"After update {state_key}: {self.states.get(state_key, 'Not Found')}")  # 更新後の状態をログ出力

    # WebSocket送信データ生成
    def generate_payload(self):
        """
        WebSocketで送信する情報を生成
        """
        return {
            "robot_id": self.robot_id,
            "owner": self.owner,
            "unique_id": self.unique_id,
            "state": self.states,
        }

    # WebSocketで状態を送信
    async def send_state(self):
        """
        WebSocket接続を確立し、定期的にロボット状態を送信。
        接続エラー時には再接続を試みる。
        """
        while self.running:
            try:
                async with websockets.connect(self.websocket_url) as websocket:
                    self.get_logger().info("WebSocket connected.")
                    while self.running:
                        try:
                            # WebSocketでpingメッセージを受け取る部分を追加
                            message = await websocket.recv()  # メッセージを受信
                            data = json.loads(message)
                            if "ping" in data:  # pingメッセージを受け取った場合
                                self.get_logger().info("Received ping, sending pong.")
                                await websocket.send(json.dumps({"pong": True}))  # pongを送信

                            # 状態を送信
                            payload = self.generate_payload()
                            await websocket.send(json.dumps(payload))  # 状態を送信
                            self.get_logger().info(f"Sent data via WebSocket: {payload}")
                            await asyncio.sleep(1)  # 1秒間隔で送信
                        except Exception as e:
                            self.get_logger().error(f"Error receiving or sending WebSocket message: {e}")
                            break  # 再接続のためにループを抜ける
            except Exception as e:
                self.get_logger().error(f"WebSocket connection error: {e}")
                await asyncio.sleep(5)  # 再接続までの待機時間

    def start_websocket_loop(self):
        """
        WebSocket送信用の非同期イベントループを開始。
        """
        asyncio.run(self.send_state())

    def on_shutdown(self):
        """
        ノードのシャットダウン処理。
        """
        self.get_logger().info("Shutting down...")
        self.running = False  # スレッドのループを終了
        if self.send_thread.is_alive():
            self.send_thread.join()  # スレッドを終了
        self.get_logger().info("WebSocket thread stopped.")

def main(args=None):
    """
    エントリーポイント。
    ノードの初期化、スピン、および終了処理を実行。
    """
    rclpy.init(args=args)  # ROS 2の初期化
    node = RosStatePublisher()
    try:
        rclpy.spin(node)  # ノードをスピン
    except KeyboardInterrupt:
        pass  # Ctrl+Cで終了
    finally:
        node.on_shutdown()  # シャットダウン処理
        node.destroy_node()  # ノードの破棄
        rclpy.shutdown()  # ROS 2のシャットダウン

if __name__ == '__main__':
    main()
