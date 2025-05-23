from cobotta_control.dummy_hand_control import DummyHandControl
from cobotta_control.twofg_control import TWOFGControl

# ホルダーの座標系は、ベース座標系（ツール座標系ではない）であることに注意
tool_infos = [
    {
        "id": -1,
        "name": "no_tool",
        "tool_def": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "id_in_robot": 0,
    },
    {
        "id": 1,
        "name": "onrobot_2fg7",
        "holder_waypoints": {
            "enter_path": [402.57, 445.05, 21.20 + 200, 180, 0, -90],
            "disengaged": [402.57, 445.05, 21.20 + 30, 180, 0, -90],
            "tool_holder": [402.57, 445.05, 21.20, 180, 0, -90],
            "locked": [402.57, 445.05 + 30, 21.20, 180, 0, -90],
            "exit_path_1": [402.57, 445.05 + 120, 21.20, 180, 0, -90],
            "exit_path_2": [402.57, 445.05 + 120, 21.20 + 200, 180, 0, -90],
        },
        "tool_def": [0.0, 0.0, 160.0, 0.0, 0.0, 0.0],
        "id_in_robot": 1,
    },
    {
        "id": 2,
        "name": "cutter",
        "holder_waypoints": {
            "enter_path": [541.98, 444.11, 21.20 + 200, 180, 0, -90],
            "disengaged": [541.98, 444.11, 21.20 + 30, 180, 0, -90],
            "tool_holder": [541.98, 444.11, 21.20, 180, 0, -90],
            "locked": [541.98, 444.11 + 30, 21.20, 180, 0, -90],
            "exit_path_1": [541.98, 444.11 + 120, 21.20, 180, 0, -90],
            "exit_path_2": [541.98, 444.11 + 120, 21.20 + 200, 180, 0, -90],
        },
        "tool_def": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "id_in_robot": 2,
    },
]

tool_classes = {
    "onrobot_2fg7": TWOFGControl,
    "cutter": DummyHandControl,    
}
