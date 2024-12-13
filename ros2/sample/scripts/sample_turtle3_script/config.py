
# ロボット設定
ROBOT_ID = "turtle3"  # ロボットIDを定義してください
OWNER_NAME = "tester"  # オーナー名を定義してください　※WEBサービスのサインアップ名と一致している必要があります

# 購読トピック設定
TOPIC_SUBSCRIPTIONS = [
    {
        "topic": "/turtle3/pose",
        "message_type": "turtlesim.msg.Pose",
        "callback": "update",
        "state_key": "pos_x"
    },
  
    # {
    #     "topic": "トピック名",
    #     "message_type": "メッセージ型",
    #     "callback": "コールバック関数",
    #     "state_key": "状態キー"
    # },
]

# ユーザー定義のコールバック関数
def update(states, msg):
    states["pos_x"] = msg.x
    states["pos_y"] = msg.y
    states["theta"] = msg.theta

# def update_state2(states, msg):
    
#     states["pos_y"] = msg.y

# def update_state3(states, msg):
    
#     states["theta"] = msg.theta

   
# def コールバック関数(states, msg):
#     """
#     トピックのデータを処理して状態を更新するユーザー定義関数
#     """
#     states[状態キー] = 出力
