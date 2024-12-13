
# ロボット設定
ROBOT_ID = "your_robot_id"  # ロボットIDを定義してください
OWNER_NAME = "your_web_service_id"  # オーナー名を定義してください　※WEBサービスのサインアップ名と一致している必要があります

# 購読トピック設定
TOPIC_SUBSCRIPTIONS = [

    # {
    #     "topic": "トピック名",
    #     "message_type": "メッセージ型",
    #     "callback": "コールバック関数",
    #     "state_key": "状態キー"
    # },
]

  
# def コールバック関数(states, msg):
#     """
#     トピックのデータを処理して状態を更新するユーザー定義関数
#     """
#     states[状態キー] = 出力
