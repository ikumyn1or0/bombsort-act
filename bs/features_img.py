# 合格ライン・本命の入力：関節角5つ + 斜めのカメラ（/bs_camera、128x128 RGB）。物体の位置は入れない
features = {
    "observation.state": {
        "dtype": "float32",
        "shape": (5,),
        "names": ["joint1", "joint2", "joint3", "joint4", "gripper"],
    },
    "observation.images.front": {
        "dtype": "video",
        "shape": (128, 128, 3),
        "names": ["height", "width", "channels"],
    },
    "action": {
        "dtype": "float32",
        "shape": (5,),
        "names": ["joint1", "joint2", "joint3", "joint4", "gripper"],
    },
}
robot_type = "crane_plus"
repo_id = "bombsort_img"
task_name = "bombsort"
