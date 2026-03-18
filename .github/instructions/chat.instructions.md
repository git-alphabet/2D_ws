---
applyTo: "**"
---
第一次回答最后发1,第二次最后发2,以此类推
禁止盲目猜测  可以推断但必须告知用户  对某些方面不清楚就问用户 并看历史记录
如果贴了日志  先阐述这个日志是什么意思
开发环境：ubuntu22.04 ros2 humble docker容器内 宿主机代码挂载在容器内 改代码就直接在宿主机改就好了 直接看容器内日志  任何命令都要先进容器 docker exec!  每次对话你都会新开终端  所以需要先进入容器
看完我给你的文件
不要写一大堆没用的 在添加代码的时候要看完整份文件
本项目中，仿真引入 namespace 的设计，与 ROS 相关的 node, topic, action 等都加入了 namespace 前缀。如需查看 tf tree，请使用命令 ros2 run rqt_tf_tree rqt_tf_tree --ros-args -r /tf:=tf -r /tf_static:=tf_static -r __ns:=/red_standard_robot1,实车不带ns,如/cmd_vel
请遵循最佳实践  充分利用我的插件 并说明怎么用的这些插件
复现 → 定位 → 排查 → 解决 → 验证 → 复盘
当前目标：rviz上cancel之后能停止所有动作包括旋转  然后startup后保持之前的自旋转  当一次运行中  开过非匀速小陀螺 则无视常量角速度  一次运行中只允许非匀速或匀速
可以自动迭代
用英文思考 用中文回答
