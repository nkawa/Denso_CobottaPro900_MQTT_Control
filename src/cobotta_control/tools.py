from cobotta_control.twofg_control import TWOFGControl

# ホルダーの座標系は、ベース座標系（ツール座標系ではない）であることに注意
tool_infos = [
    {
        "id": -1,
        "name": "no_tool",
        "id_in_robot": 0,
    },
    {
        "id": 1,
        "name": "onrobot_2fg7",
        "holder_waypoints": {
            "enter_path": [530, 480, 200, 180, 0, 90],
            "disengaged": [530, 480, 50, 180, 0, 90],
            "tool_holder": [530, 480, 0, 180, 0, 90],
            "locked": [530, 580, 0, 180, 0, 90],
            "exit_path": [530, 580, 200, 180, 0, 90],
        },
        "tool_def": [0.0, 0.0, 160.0, 0.0, 0.0, 0.0],
        "id_in_robot": 1,
    },
    {
        "id": 2,
        "name": "robotiq_epick",
        "holder_waypoints": {
            "enter_path": [630, 480, 200, 180, 0, 90],
            "disengaged": [630, 480, 50, 180, 0, 90],
            "tool_holder": [630, 480, 0, 180, 0, 90],
            "locked": [630, 580, 0, 180, 0, 90],
            "exit_path": [630, 580, 200, 180, 0, 90],
        },
        "tool_def": [0.0, 0.0, 160.0, 0.0, 0.0, 0.0],
        # NOTE: デモ用。元に戻す
        "id_in_robot": 1,
    },
]

tool_classes = {
    "onrobot_2fg7": TWOFGControl,
    "robotiq_epick": TWOFGControl,    
}
