# GXU_ROBOTZ_NAV2026

GXU RobotZ 2026 赛季 ROS 2（Humble）导航工作空间。

- 基于 Nav2 的仿真/实车导航与建图流程
- 集成 NeuPAN（`neupan_nav2_controller`）控制器插件：**必须在虚拟环境中构建与运行**（见下文）
- 主要启动/构建入口集中在 `./scripts/`

> 说明：本工作空间包含上游 `pb2025_sentry_nav` 的完整功能与更详细文档，推荐同时阅读：
> - `src/pb2025_sentry_nav/README.md`

---

## 1. 环境要求

- OS：Ubuntu 22.04（建议）
- ROS 2：Humble
- 构建工具：`colcon`, `rosdep`
- 图形终端：脚本默认优先使用 `gnome-terminal`，否则使用 `x-terminal-emulator`

> 备注：多个脚本会导出 NVIDIA PRIME 相关环境变量（`__NV_PRIME_RENDER_OFFLOAD` 等）。如果你不是双显卡/PRIME 环境，一般不影响功能；必要时可自行取消。

---

## 2. 目录结构（约定）

- `src/`：ROS 2 packages（导航/仿真/驱动/控制器等）
- `scripts/`：一键脚本（建环境、构建、仿真、实车启动）
- `neupan_env/`：NeuPAN Python 虚拟环境（**必须**，并且目录内带 `COLCON_IGNORE` 防止被 colcon 当成包）
- `build/ install/ log/`：colcon 生成目录

---

## 3. 快速开始（推荐路径）

### 3.1 安装依赖（一次性）

在工作空间根目录执行：

```bash
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

> 如果你是首次使用 rosdep，请先完成 rosdep 初始化（此处不赘述）。

### 3.2 初始化 NeuPAN 虚拟环境（一次性或依赖变更后）

**neupan 相关一定要在虚拟环境构建**：

```bash
./scripts/setup_neupan_env.sh
```

常用可选参数（按需）：

- 重新创建 venv：

```bash
RECREATE_VENV=1 ./scripts/setup_neupan_env.sh
```

- 指定 python：

```bash
PYTHON_BIN=python3 ./scripts/setup_neupan_env.sh
```

该脚本会：
- 创建/更新 `./neupan_env`
- 安装 `src/neupan_nav2_controller/requirements.txt` 中的依赖（包含 CPU 版 PyTorch 索引）
- 处理 ECOS 的兼容性补丁（脚本内置 best-effort patch）

### 3.3 构建（每次代码更新后）

推荐使用脚本构建，它会先构建非 NeuPAN 包，再激活虚拟环境构建 `neupan_nav2_controller`：

- 稳定构建（串行，资源占用更低）：

```bash
./scripts/complete_build.sh
```

- 快速构建（并行，机器性能好可用）：

```bash
./scripts/quick_build.sh
```

---

## 4. 运行

> 运行前提：已经成功构建并生成 `install/setup.bash`。

### 4.1 仿真：导航

```bash
./scripts/nav_sim.sh
```

该脚本会启动 Gazebo 与 Nav2；如果你的参数文件中选择了 NeuPAN 控制器插件，会自动激活 `neupan_env`（见 5.1）。

### 4.2 仿真：建图（SLAM）

```bash
./scripts/sim_mapping.sh
```

### 4.3 实车：导航（非 SLAM）

```bash
./scripts/start_navigation.sh
```

### 4.4 实车：建图（SLAM）

```bash
./scripts/mapping.sh
```

### 4.5 其他常用脚本

- 启动串口驱动：

```bash
./scripts/serial_driver.sh
```

- 发布决策/比赛相关话题（调试用）：

```bash
./scripts/publish_script.sh
```

---

## 4.6 实车数据录包（ros2 bag）

当前阶段建议：**实车每次运行都录包**，用于复现与回放验证算法。

- 最小录包（默认 sqlite3，按时间命名输出目录）：

```bash
./scripts/record_bag.sh
```

- 回放：

```bash
ros2 run ros2_bag_tools play_bag ./src/ros2_bag_tools/bags/<时间目录>
```

如需估算录包大小，最直接的方法是录一次然后查看目录大小（`du -sh <bag_dir>`）。
更多说明见：`src/ros2_bag_tools/README.md`

---

## 5. 配置与开关

### 5.1 NeuPAN 虚拟环境是否会被自动启用？

脚本会读取参数文件中的：

- `pb_navigation_switches.ros__parameters.controller_plugin`

当该字段为以下值时，脚本会自动 `source neupan_env/bin/activate` 并把 venv site-packages 加到 `PYTHONPATH`：

- `neupan_nav2_controller`
- （部分脚本也支持）`neupan_slam_controller`

对应默认参数文件路径：

- 仿真：`src/pb2025_sentry_nav/pb2025_nav_bringup/config/simulation/nav2_params.yaml`
- 实车：`src/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/nav2_params.yaml`

### 5.2 NeuPAN 模型加载（如果需要）

- 实车脚本默认尝试加载：

`install/neupan_models/share/neupan_models/local_setup.bash`

若该文件不存在，会给出 warning 并跳过。

- 仿真脚本默认不设置模型 hook（`NEUPAN_MODEL_SETUP` 为空）。若你需要外部模型包，可在运行前指定：

```bash
NEUPAN_MODEL_SETUP=/path/to/local_setup.bash ./scripts/nav_sim.sh
```

---

## 6. 常见问题（FAQ）

1) **提示 NeuPAN venv 不存在**（`NeuPAN virtualenv not found .../neupan_env/bin/activate`）

- 先执行：`./scripts/setup_neupan_env.sh`

2) **没有可用的图形终端**（脚本报 `No supported graphical terminal available.`）

- 安装 `gnome-terminal`，或设置 `TERMINAL_CMD` 指向你系统可用的终端程序。

3) **运行时 Python 依赖/ABI 报错**

- NeuPAN 依赖在 `src/neupan_nav2_controller/requirements.txt` 中对 `numpy<2`、`scipy<1.15` 有约束。
- 优先使用 `./scripts/setup_neupan_env.sh` 统一安装，不建议混用系统 pip。

---

## 7. 参考

- 上游导航包（含详细运行说明、launch 参数、地图/点云等）：
  - `src/pb2025_sentry_nav/README.md`

---

## 8. Docker 镜像（部署到小电脑）

本工作空间根目录提供了多阶段 [Dockerfile](Dockerfile)：构建阶段会先 `colcon build`（跳过 `neupan_nav2_controller`），再创建/安装 NeuPAN Python 依赖虚拟环境，最后在 venv 下编译 `neupan_nav2_controller`。

### 8.1 构建镜像

首次构建或 **任何 `package.xml` 变更后**，先生成 rosdep 依赖快照（减少重复跑 `rosdep install` 的时间）：

```bash
./scripts/gen_rosdep_src.sh
```

```bash
docker build -t gxu_robotz_nav2026:latest .
```

可选参数（按需）：

```bash
DOCKER_BUILDKIT=1 docker build --progress=plain --network host -t gxu_robotz_nav2026:latest \
--build-arg APT_MIRROR=mirrors.tuna.tsinghua.edu.cn \
--build-arg http_proxy=http://127.0.0.1:7897 --build-arg https_proxy=http://127.0.0.1:7897 \
--build-arg HTTP_PROXY=http://127.0.0.1:7897 --build-arg HTTPS_PROXY=http://127.0.0.1:7897 \
. 2>&1 | tee docker_build_plain.log

```

> 注意：`COLCON_SKIP_PACKAGES` 需要是“空格分隔”的包名列表。若你要裁剪仿真相关包以减小镜像体积，请先告诉我你要保留/裁剪的包，我再帮你给出一份默认推荐列表（避免瞎猜）。

### 8.1.1（推荐）带版本号构建 + 同步 latest + 推送 Docker Hub

说明：
- `latest` 始终指向“最新可用镜像”
- 同时打一个日期版本号 tag（便于回滚/对齐小电脑部署）

```bash
# 你的 Docker Hub 命名空间（用户名或组织名）
export DOCKERHUB_NS=alphabet2006
# 仓库名
export IMAGE_REPO=gxu_robotz_nav2026
# 版本号（示例：v20251228_2359）
export IMAGE_TAG="v$(date +%Y%m%d_%H%M)"

DOCKER_BUILDKIT=1 docker build --progress=plain --network host \
  -t ${IMAGE_REPO}:latest \
  -t ${DOCKERHUB_NS}/${IMAGE_REPO}:${IMAGE_TAG} \
  -t ${DOCKERHUB_NS}/${IMAGE_REPO}:latest \
  --build-arg APT_MIRROR=mirrors.tuna.tsinghua.edu.cn \
  --build-arg http_proxy=http://127.0.0.1:7897 --build-arg https_proxy=http://127.0.0.1:7897 \
  --build-arg HTTP_PROXY=http://127.0.0.1:7897 --build-arg HTTPS_PROXY=http://127.0.0.1:7897 \
  . 2>&1 | tee docker_build_plain.log

docker login -u ${DOCKERHUB_NS}
docker push ${DOCKERHUB_NS}/${IMAGE_REPO}:${IMAGE_TAG}
docker push ${DOCKERHUB_NS}/${IMAGE_REPO}:latest
```

### 8.2 运行镜像

容器默认进入 bash，并已自动 source：`/opt/ros/$ROS_DISTRO/setup.bash` 与 `/ws/install/setup.bash`。

#### 8.2.1（有桌面/需要 RViz）允许容器访问宿主机 X11

```bash
xhost +local:docker
```

#### 8.2.2 创建并进入容器

说明：
- 使用 `--network host` 便于 ROS 2 发现与多机通信
- 映射 X11 用于 RViz
- 映射 `/dev` 便于后续访问雷达/串口/手柄等设备（按需）

```bash
docker run -it --rm --name gxu_robotz_nav2026 \
  --network host \
  -e "DISPLAY=$DISPLAY" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /dev:/dev \
  alphabet2006/gxu_robotz_nav2026:latest
```

#### 8.2.3（无桌面小电脑）最小运行

```bash
docker run -it --rm --name gxu_robotz_nav2026 \
  --network host \
  -v /dev:/dev \
  alphabet2006/gxu_robotz_nav2026:latest
```

> 说明：仓库里的 `scripts/*` 多数会尝试调用图形终端（`gnome-terminal`/`x-terminal-emulator`）打开新窗口；在无桌面/无 X11 的小电脑上更建议直接在容器内运行 `ros2 launch ...`。
