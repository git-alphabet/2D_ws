# 8. 行为树 XML 路径配置规范

行为树 XML 路径（behavior_tree 配置项）推荐写法：

- 推荐：包内相对路径
  - 例如：`config/RMUL_2026/rmul_2026.xml`
  - 解析时会自动定位到 rm_behavior_tree 包的 share/config 目录
- 支持：包名前缀路径
  - 例如：`rm_behavior_tree:config/RMUL_2026/rmul_2026.xml`
  - 解析时会先查包名，再拼接相对路径
- 不推荐：仓库相对路径/绝对路径
  - 例如：`src/RM_Behavior_Tree/rm_behavior_tree/config/RMUL_2026/rmul_2026.xml` 或 `/home/xxx/.../rmul_2026.xml`
  - 容易因工作目录/包结构变化导致找不到文件

> 行为树 XML 路径解析逻辑详见 `src/gxu2026_sentry_nav/gxu2026_nav_bringup/launch/navigation_launch.py` 的 `_resolve_bt_style_path`。

实际配置示例：

```yaml
pb_navigation_switches:
  ros__parameters:
    behavior_tree: "config/RMUL_2026/rmul_2026.xml"
```

如需自定义行为树，建议将 XML 放在 rm_behavior_tree 包的 config 子目录下，并用包内相对路径引用。
# GXU_ROBOTZ_NAV2026

GXU RobotZ 2026 赛季导航工作空间。

- 基于 Nav2 的仿真/实车导航与建图流程
- 集成 NeuPAN（`neupan_nav2_controller`）控制器插件：**必须在虚拟环境中构建与运行**（见下文）
- 主要启动/构建入口集中在 `./scripts/`

> 说明：本工作空间包含上游 `gxu2026_sentry_nav` 的完整功能与更详细文档，推荐同时阅读：
> - `src/gxu2026_sentry_nav/README.md`

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

> 如果你是首次使用 rosdep，请先完成 rosdep 初始化。

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

> 建议：首次构建或切到新分支后的冷构建，优先使用 `complete_build.sh`，避免并行编译导致内存峰值过高。

- 快速构建（并行，机器性能好可用）：

```bash
./scripts/quick_build.sh
```

> `quick_build.sh` 更适合增量构建；脚本会在检测到“冷构建”时自动回退为串行执行（可用 `FORCE_PARALLEL=1` 强制并行）。

---

## 4. 运行

> 运行前提：已经成功构建并生成 `install/setup.bash`。

### 4.1 仿真：导航

```bash
./scripts/nav_sim.sh
```

该脚本会启动 Gazebo 与 Nav2；如果你的参数文件中选择了 NeuPAN 控制器插件，会自动激活 `neupan_env`（见 3.2）。

### 4.2 仿真：建图（SLAM）

```bash
./scripts/sim_mapping.sh
```

### 4.3 实车：导航（非 SLAM）

```bash
./scripts/start_navigation.sh
```

默认会同时启动两路 ros2 bag：
- `full`：全量回放包
- `raw`：原始数据调试包

录包总参数放在实车启动总参数 `src/gxu2026_sentry_nav/gxu2026_nav_bringup/config/reality/nav2_params.yaml`

如需临时关闭：

```bash
AUTO_RECORD_BAG=0 ./scripts/start_navigation.sh
```

### 4.4 实车：建图（SLAM）

```bash
./scripts/mapping.sh
```

默认会同时启动两路 ros2 bag：
- `full`：全量回放包
- `raw`：原始数据调试包

录包总参数放在实车启动总参数 `src/gxu2026_sentry_nav/gxu2026_nav_bringup/config/reality/nav2_params.yaml`

如需临时关闭：

```bash
AUTO_RECORD_BAG=0 ./scripts/mapping.sh
```

### 4.4.1 bag 回放：建图（SLAM）

```bash
./scripts/mapping_bag.sh
```

该脚本固定：
- `use_sim_time:=True`
- `AUTO_RECORD_BAG=0`

用于 `ros2 bag play --clock ...` 时直接回放建图，不需要再手改 `mapping.sh`。

### 4.3.1 bag 回放：导航

```bash
./scripts/nav_bag.sh
```

该脚本固定：
- `use_sim_time:=True`
- `AUTO_RECORD_BAG=0`

用于 `ros2 bag play --clock ...` 时直接回放导航，不需要再手改 `nav.sh`。



## 4.5 实车数据录包（ros2 bag）

当前阶段建议：**实车每次运行都录包**，用于复现与回放验证算法。

- 最小录包（默认 mcap，按时间命名输出目录）：

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

## 5. 常见问题（FAQ）

1) **提示 NeuPAN venv 不存在**（`NeuPAN virtualenv not found .../neupan_env/bin/activate`）

- 先执行：`./scripts/setup_neupan_env.sh`


2) **运行时 Python 依赖/ABI 报错**

- NeuPAN 依赖在 `src/neupan_nav2_controller/requirements.txt` 中对 `numpy<2`、`scipy<1.15` 有约束。
- 优先使用 `./scripts/setup_neupan_env.sh` 统一安装，不建议混用系统 pip。

---

## 6. Docker 镜像（部署到小电脑）

### 6.1.1（推荐）带版本号构建 + 同步 latest + 推送 Docker Hub

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


## 7. Docker 开发环境（`Dockerfile.env` + 挂载代码）

与第 6 节不同，本节的镜像 **只打包依赖，不编译代码**。
开发时把 `src/` 挂载进容器，在容器内 `colcon build`，改代码后无需重建镜像。

### 7.1 镜像内包含的依赖

| 类别 | 具体内容 |
|---|---|
| **基础镜像** | `ros:humble-ros-base`（Ubuntu 22.04 + ROS2 Humble） |
| **编译工具** | `build-essential` / `cmake` / `git` / `curl` / `wget` |
| **Python 工具链** | `python3-pip` / `python3-venv` / `python3-dev` / `python3-colcon-common-extensions` / `python3-rosdep` |
| **点云 / 线性代数** | `libpcl-dev` / `libeigen3-dev` / `libomp-dev` |
| **OpenGL / EGL 渲染** | `libgl1-mesa-dev` / `libgles2-mesa-dev` / `libegl1-mesa-dev` / `mesa-utils` / `xvfb` |
| **仿真（Ignition Fortress）** | `ros-humble-ros-gz-sim` / `ros-humble-ros-gz-bridge` / `ignition-fortress`（通过 OSRF apt 源 + rosdep） |
| **ROS 包依赖** | 由 `docker/rosdep_src/` 下的 `package.xml` 快照 `rosdep install` 安装（含导航、感知、描述等全部包依赖） |
| **small_gicp** | 预编译并 `cmake --install` 到系统（重定位 / 点云配准用） |
| **NeuPAN Python venv** | `numpy<2` / `scipy<1.15` / `torch==2.1.0+cpu` / `cvxpy` / `cvxpylayers` / `diffcp` / `ecos` / `gctl` / `clarabel` / `osqp` / `scs` / `matplotlib` / `scikit-learn` |
| **NVIDIA GPU 透传** | `NVIDIA_VISIBLE_DEVICES=all` + `NVIDIA_DRIVER_CAPABILITIES=graphics,compute,display,utility`，配合 nvidia-container-toolkit 透传宿主机 GPU（Gazebo 渲染 + CUDA） |

> **不包含**：CUDA/cuDNN 运行时（torch 为 CPU 版）、TensorRT。如需 CUDA 推理需替换 base 镜像。

### 7.2 宿主机前置条件（一次性）

#### 步骤 1：安装 nvidia-container-toolkit（仅 NVIDIA 显卡宿主机）

```bash
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

#### 步骤 2：配置 X11 自动授权（系统服务，支持 Wayland + NoMachine）

> **为什么需要这步**：
> 1) `/tmp` 每次重启会清空，`/tmp/.docker.xauth` 会丢失。  
> 2) Wayland 和 NoMachine 的授权 cookie 位置不固定，单纯绑定 `~/.Xauthority` 经常不够用。  
> 3) 旧方案是 user service + 单一 `DISPLAY=:0`，容易在 Wayland / NoMachine / 连屏幕切换后失效。
>
> 新方案使用 **system service + timer**，周期性汇总 `/home/*/.Xauthority`、`/run/user/*/Xauthority`、`/run/user/*/.mutter-Xwaylandauth.*`，自动生成：
> - `/tmp/.docker.xauth`
> - `/tmp/gxu2026-docker-gui/xauth`
> - `/tmp/gxu2026-docker-gui/display`
> - `/tmp/gxu2026-docker-gui/displays`
>
> 容器启动时会优先读取这些提示文件并自动选择可用显示号，不再强依赖你启动 `docker compose` 时所在终端的 `DISPLAY`。

```bash
cd ~/ros2_ws
sudo apt-get install -y xauth
chmod +x scripts/systemd/install_docker_xauth_service.sh
sudo bash scripts/systemd/install_docker_xauth_service.sh
```

验证：

```bash
systemctl status gxu2026-docker-xauth.service
systemctl status gxu2026-docker-xauth.timer
ls -la /tmp/.docker.xauth   # 应为普通文件，非目录
xauth -f /tmp/.docker.xauth list | head
ls -la /tmp/gxu2026-docker-gui
cat /tmp/gxu2026-docker-gui/display
cat /tmp/gxu2026-docker-gui/displays
```

如果你有自定义路径，也可在 `docker/.env` 中设置：

```bash
DOCKER_GUI_HOST_DIR=/tmp/gxu2026-docker-gui
XAUTH_HOST_FILE=/tmp/.docker.xauth
```

> 现在即使用 VS Code 右键 `Compose Up`，容器也会在启动时自动选择一个存在 X socket 的显示号。若你想强制固定某个显示号，可在 `docker/.env` 或 compose 环境变量中额外设置 `GXU_DISPLAY_OVERRIDE=:1001`。

### 7.3 构建环境镜像

```bash
# 国内加速（默认已设为清华源，直接 build 即可）
docker compose -f docker/compose.build.yml build

# 带代理
http_proxy=http://127.0.0.1:7897 https_proxy=http://127.0.0.1:7897 \
  docker compose -f docker/compose.build.yml build

# 打版本号
IMAGE_TAG=$(date +%Y%m%d) docker compose -f docker/compose.build.yml build

# 构建完直接推送 Docker Hub
IMAGE_TAG=$(date +%Y%m%d) docker compose -f docker/compose.build.yml build
IMAGE_TAG=$(date +%Y%m%d) docker compose -f docker/compose.build.yml push
```

> 依赖没变就不需要重建镜像，一次构建长期复用。

### 7.4 启动开发容器

```bash
# 笔记本（有 NVIDIA GPU，跑仿真）
docker compose -f docker/compose.dev.yml --profile laptop up dev-laptop

# 小电脑实车（无 GPU）
docker compose -f docker/compose.dev.yml --profile robot up dev-robot

# 指定镜像 tag
IMAGE_TAG=20260228 docker compose -f docker/compose.dev.yml --profile laptop up dev-laptop

# 停止
docker compose -f docker/compose.dev.yml --profile robot down

# 进入已运行的容器（开第二个终端）
docker exec -it gxu2026-nav-robot bash
```

> Container Tools UI：右键 `docker/compose.dev.yml` → "Compose Up (Select Services)"，选对应服务即可。

| 路径 | 来源 | 说明 |
|---|---|---|
| `/ws/src` | 宿主机 `src/` | 改代码直接生效，容器内 build |
| `/ws/scripts` | 宿主机 `scripts/` | 启动脚本同步 |
| `/ws/build` | Docker 命名 volume | 持久化，重启不丢编译缓存 |
| `/ws/install` | Docker 命名 volume | 持久化 |
| `/ws/log` | Docker 命名 volume | 持久化 |
| `/ws/neupan_env` | 镜像内 | 已预置，不被 src/ 覆盖 |

### 7.5 小电脑（miniPC）systemd 自启动：容器 + 导航/建图 + 自瞄

需求覆盖：
- 开机后由 systemd 拉起容器 `dev-robot`
- 导航/建图由你手动启动（不设为开机自启）
- 默认开启 `START_RVIZ=1`（配合 HDMI 欺骗器保活图形输出）
- 自瞄二进制由 systemd 托管并默认开机自启，且启动后先等待 10 秒再进入程序

#### 1) 用 GitHub 同步到小电脑并安装

```bash
# 小电脑执行
cd ~/ros2_ws
git pull
bash scripts/systemd/install_on_minipc.sh --mode navigation
```

默认行为：
- `gxu2026-nav-stack.service`：disable + stop（你手动启动）
- `gxu2026-auto-aim.service`：enable + restart（开机自启，`ExecStartPre=/bin/sleep 10`）
- `gxu2026-nav-stack.service` 在 `navigation/mapping` 模式下默认同时启动全量录包（`AUTO_RECORD_BAG=1`, `AUTO_RECORD_BAG_MODE=full`）

#### 2) 小电脑本地手动安装（可选）

```bash
cd ~/ros2_ws
chmod +x scripts/systemd/*.sh
bash scripts/systemd/install_on_minipc.sh --mode navigation
```

#### 3) 运行时切换导航/建图

```bash
cd ~/ros2_ws
bash scripts/systemd/use_mapping.sh
bash scripts/systemd/use_navigation.sh
```

#### 4) 调试时怎么关掉（你关心的点）

临时关闭（仅本次，重启后若 service 已 enable 会自动回来）：

```bash
cd ~/ros2_ws
bash scripts/systemd/nav_stop.sh
```

恢复：

```bash
bash scripts/systemd/nav_start.sh
```

彻底关闭开机自启（调试阶段常用）：

```bash
bash scripts/systemd/service_ctl.sh disable
```

重新启用开机自启：

```bash
bash scripts/systemd/service_ctl.sh enable
bash scripts/systemd/service_ctl.sh restart
```

#### 5) 自瞄（手动敲，但交给 systemd 管）

```bash
# 手动启动自瞄（不改开机策略）
bash scripts/systemd/autoaim_start.sh

# 手动停止
bash scripts/systemd/autoaim_stop.sh

# 如果你也想让自瞄开机自启
bash scripts/systemd/service_ctl.sh enable autoaim
```

#### 6) 最常用免参数命令（少手敲）

```bash
cd ~/ros2_ws
bash scripts/systemd/use_navigation.sh   # 切导航并重启导航服务
bash scripts/systemd/use_mapping.sh      # 切建图并重启导航服务
bash scripts/systemd/nav_start.sh        # 启动导航/建图服务
bash scripts/systemd/nav_stop.sh         # 停止导航/建图服务
bash scripts/systemd/mapping_stop.sh     # 停止建图（等价于 nav_stop）
bash scripts/systemd/autoaim_start.sh    # 启动自瞄服务
bash scripts/systemd/autoaim_stop.sh     # 停止自瞄服务
bash scripts/systemd/all_stop.sh         # 全停
bash scripts/systemd/status.sh           # 看两个服务状态
```

查看状态与日志：

```bash
bash scripts/systemd/service_ctl.sh status all
journalctl --user -u gxu2026-nav-stack.service -f
journalctl --user -u gxu2026-auto-aim.service -f
```
