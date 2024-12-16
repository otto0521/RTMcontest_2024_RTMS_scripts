import RTC 
import OpenRTM_aist
import sys
import asyncio
import json
import uuid
import os
import websockets
from threading import Thread
import importlib
import config

websocketsender_spec = ["implementation_id", "WebSocketSender", 
         "type_name",         "WebSocketSender", 
         "description",       "send data to a WebSocket endpoint", 
         "version",           "1.0.0", 
         "vendor",            "meitoku", 
         "category",          "WebSocket", 
         "activity_type",     "STATIC", 
         "max_instance",      "1", 
         "language",          "Python", 
         "lang_type",         "SCRIPT",
         ""]

UUID_FILE_PATH = os.path.join(os.path.dirname(__file__), config.UUID_FILE_PATH)

def import_data_type(type_str):
    module_name, class_name = type_str.rsplit(".", 1)
    if module_name == "RTC":
        return getattr(RTC, class_name)
    else:
        module = importlib.import_module(module_name)
        return getattr(module, class_name)


class WebSocketSender(OpenRTM_aist.DataFlowComponentBase):

    def __init__(self, manager):
        OpenRTM_aist.DataFlowComponentBase.__init__(self, manager)

        self.robot_id = config.ROBOT_ID
        self.owner = config.OWNER_NAME
        self.unique_id = self.get_or_create_uuid(UUID_FILE_PATH)
        self.websocket_url = f"wss://monitoring.ddns.net/ws/robots/?unique_robot_id={self.unique_id}"

        self.states = {port["state_key"]: "unknown" for port in config.INPORTS}

        self.in_ports = {}
        self.setup_inports()

        self.running = True
        self.websocket_thread = Thread(target=self.start_websocket_loop, daemon=True)
        self.websocket_thread.start()

    def get_or_create_uuid(self, uuid_file_path):
        if os.path.exists(uuid_file_path):
            with open(uuid_file_path, "r") as file:
                unique_id = file.read().strip()
                return unique_id
        else:
            unique_id = uuid.uuid4().hex
            with open(uuid_file_path, "w") as file:
                file.write(unique_id)
            return unique_id

    def setup_inports(self):
        for port_config in config.INPORTS:
            port_name = port_config["name"]
            state_key = port_config["state_key"]
            data_type = import_data_type(port_config["data_type"])

            initial_value = data_type(RTC.Time(0, 0), 0)
            in_port = OpenRTM_aist.InPort(port_name, initial_value)

            self.addInPort(port_name, in_port)
            self.in_ports[port_name] = {"port": in_port, "state_key": state_key}

    def onInitialize(self):
        try:
            if hasattr(RTC, 'RTC_OK'):
                return RTC.RTC_OK
            elif hasattr(OpenRTM_aist, 'RTC_OK'):
                return OpenRTM_aist.RTC_OK
            else:
                raise AttributeError("RTC_OK is not defined.")
        except Exception as e:
            print(f"onInitialize error: {e}")
            return RTC.RTC_ERROR if hasattr(RTC, 'RTC_ERROR') else -1

    def onExecute(self, ec_id):
        for port_name, port_info in self.in_ports.items():
            if port_info["port"].isNew():
                data = port_info["port"].read()
                self.states[port_info["state_key"]] = data.data
                self.send_payload()
        return RTC.RTC_OK if hasattr(RTC, 'RTC_OK') else 0

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
                async with websockets.connect(self.websocket_url) as websocket:
                    print("WebSocket connected successfully.")
                    while self.running:
                        payload = self.generate_payload()
                        await websocket.send(json.dumps(payload))
                        await asyncio.sleep(0.2)
            except Exception as e:
                print(f"WebSocket connection error: {e}")
                await asyncio.sleep(5)


    async def _send_payload(self, payload):
        try:
            async with websockets.connect(self.websocket_url) as websocket:
                await websocket.send(json.dumps(payload))
        except Exception as e:
            print(f"Error: {e}")

    def send_payload(self):
        payload = self.generate_payload()
        asyncio.run(self._send_payload(payload))

    def start_websocket_loop(self):
        asyncio.run(self.send_state())

    def on_shutdown(self):
        self.running = False
        if self.websocket_thread.is_alive():
            self.websocket_thread.join()


def WebSocketSenderInit(manager):
    profile = OpenRTM_aist.Properties(defaults_str=websocketsender_spec)
    manager.registerFactory(profile, WebSocketSender, OpenRTM_aist.Delete)


def main():
    mgr = OpenRTM_aist.Manager.init(sys.argv)
    mgr.activateManager()
    comp = mgr.createComponent("WebSocketSender")
    if comp is None:
        print("Failed to create component.")
        return
    comp.onInitialize()
    comp.start_websocket_loop()


if __name__ == "__main__":
    main()
