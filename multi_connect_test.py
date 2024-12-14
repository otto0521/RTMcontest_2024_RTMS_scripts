import asyncio
import websockets
import json
import random
import logging

# logging.basicConfig(level=logging.INFO)
# logger = logging.getLogger(__name__)

class RobotStatePublisher:
    def __init__(self, robot_id, owner, unique_id):
        self.robot_id = robot_id
        self.owner = owner
        self.unique_id = unique_id
        self.websocket_url = f"wss://monitoring.ddns.net/ws/robots/?unique_robot_id={self.unique_id}"
        # self.websocket_url = f"ws://localhost:8000/ws/robots/?unique_robot_id={self.unique_id}"
        
        

    def generate_state(self):
        return {
            "robot_id": self.robot_id,
            "owner": self.owner,
            "unique_id": self.unique_id,
            "state": {
                "pos_x": f"{random.randint(1, 10)}", 
                "pos_y": f"{random.randint(1, 10)}", 
                "speed": f"{random.randint(30, 50)}", 
                "current": f"{random.randint(10, 30)}",
                "occupancy rate": f"{random.randint(62, 70)}",
            },
        }

    async def send_state(self):
        initial_delay = random.uniform(0, 1)
        await asyncio.sleep(initial_delay)

        while True:
            async with websockets.connect(self.websocket_url) as websocket:
                # logger.info(f"[{self.unique_id}] Connected to server at {self.websocket_url}")
                while True:
                    state_data = self.generate_state()
                    await websocket.send(json.dumps(state_data))
                    # logger.info(f"[{self.unique_id}] Sent data: {state_data}")
                    await asyncio.sleep(0.2)  # 0.2秒間隔

async def main():
    publishers = [
        RobotStatePublisher("web_test_1", "tester", "web_test_1_uuid"),
        RobotStatePublisher("web_test_2", "tester", "web_test_2_uuid"),
        RobotStatePublisher("web_test_3", "tester", "web_test_3_uuid"),
        RobotStatePublisher("web_test_4", "tester", "web_test_4_uuid"),
        RobotStatePublisher("web_test_5", "tester", "web_test_5_uuid"),
        RobotStatePublisher("web_test_6", "tester", "web_test_6_uuid"),
        RobotStatePublisher("web_test_7", "tester", "web_test_7_uuid"),
        RobotStatePublisher("web_test_8", "tester", "web_test_8_uuid"),
    ]

    tasks = [publisher.send_state() for publisher in publishers]
    await asyncio.gather(*tasks)

if __name__ == "__main__":
    asyncio.run(main())
