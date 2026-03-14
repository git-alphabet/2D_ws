---
applyTo: "**"
---
每次对话开始时，必须先调用 mcp_serena_initial_instructions 工具读取项目记忆，再回答任何问题。
每次回答最后发1
禁止盲目猜测  可以推断但必须告知用户  对某些方面不清楚就问用户 并看历史记录
如果贴了日志  先阐述这个日志是什么意思
开发环境：ubuntu22.04 ros2 humble docker容器内 宿主机代码挂载在容器内 改代码就直接在宿主机改就好了 直接看容器内日志
看完我给你的文件
不要写一大堆没用的 在添加代码的时候要看完整份文件
本项目中，仿真引入 namespace 的设计，与 ROS 相关的 node, topic, action 等都加入了 namespace 前缀。如需查看 tf tree，请使用命令 ros2 run rqt_tf_tree rqt_tf_tree --ros-args -r /tf:=tf -r /tf_static:=tf_static -r __ns:=/red_standard_robot1,实车不带ns,如/cmd_vel
请遵循最佳实践  充分利用我的插件 并说明怎么用的这些插件
复现 → 定位 → 排查 → 解决 → 验证 → 复盘
当前目标：
可以自动迭代
用英文思考 用中文回答
