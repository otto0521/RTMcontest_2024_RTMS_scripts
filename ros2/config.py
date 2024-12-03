
# ロボット設定
ROBOT_ID = "your_robot_id"  # ロボットIDを定義してください
OWNER_NAME = "charachi"  # オーナー名を定義してください　※WEBサービスのサインアップ名と一致している必要があります

# 購読トピック設定
TOPIC_SUBSCRIPTIONS = [
    {
        "topic": "/turtle1/cmd_vel",
        "message_type": "geometry_msgs.msg.Twist",
        "callback": "update_state1",
        "state_key": "default_state1"
    },
    # {
    #     "topic": "トピック名",
    #     "message_type": "メッセージ型",
    #     "callback": "コールバック関数",
    #     "state_key": "状態キー"
    # },
]

# ユーザー定義のコールバック関数
def update_state1(states, msg):
    """
    トピックのデータを処理して、ロボットの状態を更新するコールバック関数。
    """
    states["default_state1"] = msg.linear.x
   
# def コールバック関数(states, msg):
#     """
#     トピックのデータを処理して状態を更新するユーザー定義関数
#     """
#     states[状態キー] = 出力
