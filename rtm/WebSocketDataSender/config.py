
# ロボット設定
ROBOT_ID = "RTM_test"  # ロボットIDを定義してください
OWNER_NAME = "tester"  # オーナー名を定義してください　※WEBサービスのサインアップ名と一致している必要があります

# InPortの設定
INPORTS = [
    {
        "name": "data_in_pos_x",  # InPort名
        "state_key": "pos_x",     # 状態データのキー
        "data_type": "RTC.TimedLong",  # int型
    },
    {
        "name": "data_in_pos_y",
        "state_key": "pos_y",
        "data_type": "RTC.TimedLong",
    },
    {
        "name": "data_in_speed",
        "state_key": "speed",
        "data_type": "RTC.TimedLong",
    },
    {
        "name": "data_in_current",
        "state_key": "current",
        "data_type": "RTC.TimedLong",
    },
]

UUID_FILE_PATH = "robot_uuid.txt"