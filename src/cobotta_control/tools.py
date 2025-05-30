from cobotta_control.dummy_hand_control import DummyHandControl
from cobotta_control.twofg_control import TWOFGControl
from cobotta_control.vgc10_control import VGC10Control

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
        "tool_def": [0.0, 0.0, 190.0, 0.0, 0.0, 0.0],
        "id_in_robot": 1,
    },
    {
        "id": 2,
        "name": "onrobot_vgc10",
        "holder_waypoints": {
            "enter_path": [541.98, 444.11, 21.20 + 200, 180, 0, -90],
            "disengaged": [541.98, 444.11, 21.20 + 30, 180, 0, -90],
            "tool_holder": [541.98, 444.11, 21.20, 180, 0, -90],
            "locked": [541.98, 444.11 + 30, 21.20, 180, 0, -90],
            "exit_path_1": [541.98, 444.11 + 120, 21.20, 180, 0, -90],
            "exit_path_2": [541.98, 444.11 + 120, 21.20 + 200, 180, 0, -90],
        },
        "tool_def": [0.0, 0.0, 205.0, 0.0, 0.0, 0.0],
        "id_in_robot": 2,
    },
    {
        "id": 3,
        "name": "cutter",
        # NOTE(20250530): 現状は実際には3つ目のツールホルダーはないので座標値は要調整
        "holder_waypoints": {
            "enter_path": [681.39, 444.11, 21.20 + 200, 180, 0, -90],
            "disengaged": [681.39, 444.11, 21.20 + 30, 180, 0, -90],
            "tool_holder": [681.39, 444.11, 21.20, 180, 0, -90],
            "locked": [681.39, 444.11 + 30, 21.20, 180, 0, -90],
            "exit_path_1": [681.39, 444.11 + 120, 21.20, 180, 0, -90],
            "exit_path_2": [681.39, 444.11 + 120, 21.20 + 200, 180, 0, -90],
        },
        "tool_def": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "id_in_robot": 3,
    },
]

tool_classes = {
    "onrobot_2fg7": TWOFGControl,
    "onrobot_vgc10": VGC10Control,
    "cutter": DummyHandControl,
}
