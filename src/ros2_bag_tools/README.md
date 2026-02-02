# ros2_bag_tools

实车 ros2 bag 录制/回放工具（ROS2 Humble）。

常用：

- 录制：`ros2 run ros2_bag_tools record_bag`
- 回放：`ros2 run ros2_bag_tools play_bag <bag_dir>`
（已移除容量估算工具；如需可用临时脚本或手录一次再查看目录大小）

话题白名单在：
- `topics/bag_topics_basic.txt`
- `topics/bag_topics_full.txt`

录包默认目录：`src/ros2_bag_tools/bags/`（已在工作区 .gitignore 里忽略）。
