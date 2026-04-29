# Point-LIO PCL Processing Flow

本文档记录本仓库 `point_lio` 包中点云进入 Point-LIO 后的 PCL 处理主流程。内容以当前源码为准，主要涉及：

- `src/li_initialization.cpp`
- `src/preprocess.cpp`
- `src/IMU_Processing.cpp`
- `src/laserMapping.cpp`
- `src/Estimator.cpp`
- `include/common_lib.h`
- `include/ivox/ivox3d.h`

## 1. 总览

```text
ROS PointCloud2 / Livox CustomMsg
  -> Preprocess 转成 pcl::PointCloud<pcl::PointXYZINormal>
  -> 点过滤 / 点时间写入 curvature / 可选切帧或拼帧
  -> lidar_buffer + imu_deque 同步成 MeasureGroup
  -> feats_undistort
  -> PCL VoxelGrid 下采样为 feats_down_body
  -> 按点时间分段做 ESKF + iVox 最近邻平面匹配
  -> 转世界系得到 feats_down_world
  -> 增量写入 iVox 地图
  -> 发布 cloud_registered / cloud_registered_body / 保存 PCD
```

核心点类型定义在 `include/common_lib.h`：

```cpp
typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;
```

这里 `PointType.curvature` 被复用为点相对当前帧起点的时间，单位通常是 ms。`normal_x/y/z` 在预处理阶段基本置 0。

## 2. ROS 输入入口

主节点在 `src/laserMapping.cpp` 中启动。它读取参数后，根据 `lidar_type` 选择订阅：

- `lidar_type == AVIA`: 订阅 `livox_ros_driver2::msg::CustomMsg`，回调 `livox_pcl_cbk()`
- 其他类型: 订阅 `sensor_msgs::msg::PointCloud2`，回调 `standard_pcl_cbk()`

仿真默认配置在 `gxu2026_nav_bringup/config/simulation/nav2_params.yaml`：

```yaml
point_lio:
  ros__parameters:
    point_filter_num: 8
    space_down_sample: True
    filter_size_surf: 0.2
    filter_size_map: 0.2
    common:
      lid_topic: "velodyne_points"
      imu_topic: "livox/imu"
    preprocess:
      lidar_type: 2
      scan_line: 32
      timestamp_unit: 2
      blind: 0.6
    mapping:
      ivox_grid_resolution: 0.5
```

实车默认配置在 `gxu2026_nav_bringup/config/reality/nav2_params.yaml`，主要差异是：

- `lid_topic: "livox/lidar/pointcloud"`
- `preprocess.lidar_type: 1`
- `preprocess.scan_line: 4`
- `pcd_save.pcd_save_en: True`

## 3. 预处理: ROS 点云转 PCL 点云

外设雷达的原始点云通常是“能用但不能直接放心用”。这一步预处理和格式转换主要是在解决三个工程痛点：

- 格式不统一：ROS 话题里的 `sensor_msgs/msg/PointCloud2` 更像传输用的消息包，字段按字节打包；不同厂商又会定义不同字段，例如 Velodyne 有 `ring/time`，Livox 使用自定义消息和时间戳，Hesai/Ouster 也各有字段规则。先转成雷达专用 PCL 点类型，再统一成仓库内部的 `PointType`，可以让后续 SLAM、匹配、建图逻辑只处理一种点结构。即雷达发布msg消息包到指定话题，小电脑订阅话题接收并利用PCL库解析msg包得到实际点云坐标数据
- 数据不干净：原始点云里可能有 NaN(雷达遇到镜面反射、强光干扰时，会输出 x/y/z = NaN 的点,影响算法数学计算)、车体/雷达支架近距离点(达本身有物理盲区，离雷达太近的点比如机器人外壳、雷达支架是无效的，留着会让障碍物检测误判为 “撞车”)、远距离高噪声点(太远的点，雷达测距误差会从厘米级飙升到米级，噪声极大)、过密点(点云太多会直接占满 CPU，抽点 / 过滤线数是为了降采样，在不影响关键信息的前提下，把计算量压到系统能跑的水平)。直接送进配准或滤波，轻则增加 CPU 负担，重则让残差、矩阵求解和障碍判断全部被污染。
- 信息不完整：Point-LIO 后面按点时间推进状态，部分雷达原始消息没有直接可用的逐点时间。预处理会把时间统一写入 `curvature` 字段，缺失时用扫描角速度和水平角估算。

可以把这一步理解成“拆快递并整理工具箱”：ROS 点云消息是外包装，PCL 转换是拆包，预处理过滤是扔掉泡沫、破损件和明显不可靠的零件，最终统一的 `PointType` 才是后续算法能稳定复用的工具箱。

预处理入口在 `src/preprocess.cpp`：

- Livox: `Preprocess::process(CustomMsg, pcl_out)` -> `avia_handler()`
- PointCloud2: `Preprocess::process(PointCloud2, pcl_out)` -> 根据 `lidar_type` 调 `oust64_handler()`、`velodyne_handler()` 或 `hesai_handler()`

普通 `PointCloud2` 会先用 `pcl::fromROSMsg()` 转成雷达专用 PCL 点类型，再拷贝到统一的 `PointType`：

```cpp
pcl::PointCloud<velodyne_ros::Point> pl_orig;
pcl::fromROSMsg(*msg, pl_orig);
```

每个点会被转成：

```cpp
PointType added_pt;
added_pt.x = ...;
added_pt.y = ...;
added_pt.z = ...;
added_pt.intensity = ...;
added_pt.curvature = point_time_in_ms;
```

预处理阶段主要做这些清洗和补齐：

| 处理项 | 工程目的 |
| --- | --- |
| `pcl::fromROSMsg()` 和字段拷贝 | 把传输用 ROS 消息转换为 PCL 可处理的数据结构，再统一成内部 `PointType`，避免后续算法关心具体雷达厂商格式。 |
| NaN 过滤 | 删除 `x/y/z` 非法的点，避免后续距离计算、平面拟合、矩阵求解出现 NaN 传播。 |
| `blind` 近距离过滤 | 去掉雷达盲区、机器人外壳、支架等近距离自体点，避免被当成障碍或错误匹配点。 |
| `det_range` 远距离过滤 | 去掉远距离高噪声点，降低计算量并减少对匹配/建图精度的干扰。 |
| `point_filter_num` 抽点 | 按点序号降采样，把点量压到实时系统能承受的范围。 |
| `ring < N_SCANS` 线数过滤 | 只保留配置线数内的点，适配当前雷达线数和算法假设。 |
| `curvature` 赋值 | 统一保存逐点相对时间，供后续按点时间分段推进状态；缺失逐点时间时，用水平角和扫描角速度估算。 |

抽点和线数过滤都是直接砍数据，为了压计算量，后面的体素降采样与之不同

预处理输出写到 `pl_surf`，再赋值给回调中的 `PointCloudXYZI::Ptr ptr`。

## 4. 可选切帧和拼帧

雷达的一帧不像相机照片那样是同一瞬间采样的静态画面，而是在一个扫描周期内持续出点的集合。例如 10Hz 雷达的一帧跨度约 100ms，帧内第一个点和最后一个点之间存在明显时间差。机器人高速移动或旋转时，这段时间里的位姿已经变化，原始点云就可能出现拖影和扭曲。

`cut_frame` 和 `con_frame` 都是在围绕这个点云时间特性做工程优化：

- `cut_frame`: 把一整帧按点时间拆成更短的小段，降低单段内的时间跨度。
- `con_frame`: 把连续多帧合成一个更大的点云，提升单次处理时的点云密度。

回调位于 `src/li_initialization.cpp`：

- `standard_pcl_cbk()`
- `livox_pcl_cbk()`

如果 `cut_frame` 开启，会调用：

- `process_cut_frame_pcl2()`
- `process_cut_frame_livox()`

它们会先按点时间排序，再把一帧点云切成多个小段，推入 `lidar_buffer`。

切帧主要解决三类问题：

- 降低运动畸变：例如把 100ms 的整帧拆成两个 50ms 小段，单段内机器人位移更小，点云畸变更轻。
- 降低算法等待延迟：控制或感知周期短于雷达整帧周期时，小段点云可以更早进入 `lidar_buffer`，不用总是等完整一帧。
- 保证时间连续性：部分雷达出点顺序不严格等于时间顺序，先排序再切片能让每个小段里的点时间更连续。

如果 `con_frame` 开启，会把连续多帧点云合并到 `ptr_con`，并调整每个点的 `curvature`，让点时间仍然相对合并帧起点。

拼帧主要用于解决单帧点云太稀疏的问题。固态雷达或低线数雷达在单帧内可能点分布不均、远距离点少，SLAM 或检测算法找不到足够稳定的几何约束。把连续多帧合并后，障碍轮廓、地面结构和远距离特征会更明显，但点数量也会上升，后续匹配和建图的计算量会增加。

拼帧时必须调整 `curvature`。因为本仓库把逐点相对时间存在 `curvature` 中，合并多帧后，每个点来自不同原始帧，如果不把它们统一改成“相对合成帧起点的时间差”，后续按点时间推进状态的逻辑就会读到错乱的时间。

两者对比：

| 功能 | 核心操作 | 主要解决的问题 | 算力影响 | 典型适用场景 |
| --- | --- | --- | --- | --- |
| `cut_frame` | 一帧按时间拆成多段 | 降低单段时间跨度、减少运动畸变、缩短算法等待 | 低，主要是排序和拆分 | 高速运动、控制周期短、机械雷达畸变明显 |
| `con_frame` | 多帧合成一帧 | 提升点云密度、补充单帧特征不足 | 中等偏高，点数量增加 | 固态雷达点云稀疏、远距离特征不足、配准不稳 |

默认仿真和实车配置中，`con_frame` 与 `cut_frame` 都是 `False`。这两个开关属于可选优化项：点云密度足够、运动畸变可控时不开更简单；如果高速运动时定位漂移明显，可以优先考虑 `cut_frame`；如果单帧特征太少、远距离检测或匹配不稳，可以考虑 `con_frame`，同时留意 CPU 占用。

## 5. LiDAR 与 IMU 同步

LiDAR 与 IMU 同步解决的是“点云和惯性数据对不上时间”的问题。机械雷达一帧可能持续 100ms 左右，帧内的点是按扫描顺序陆续产生的，不是同一时刻拍下来的照片；IMU 则以更高频率(2ms)记录这段时间内的角速度和加速度。这 100ms 里，机器人的姿态早就变了。在 RoboMaster 这种高速移动、急停、快速旋转的场景里，如果不把这一帧点云对应的 IMU 数据对齐，后续就无法可靠估计扫描过程中机器人姿态的变化，点云会出现拖影、弯曲，SLAM 匹配也会明显漂移。

因此，这一步的目标是：把一帧 LiDAR 点云和它扫描期间覆盖到的 IMU 数据打包成一组 `MeasureGroup`，交给后续去畸变、状态传播、匹配和建图逻辑使用。

`sync_packages(MeasureGroup & meas)` 从 `lidar_buffer`、`time_buffer` 和 `imu_deque` 中凑出这样一组同步数据：

```cpp
struct MeasureGroup {
  double lidar_beg_time;   // 当前点云帧的起始时间
  double lidar_last_time;  // 当前点云帧的结束时间
  PointCloudXYZI::Ptr lidar; // 当前点云帧
  deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu; // 扫描期间的 IMU 数据
};
```

当前仓库把每个点相对帧起点的时间存在 `curvature` 中，单位是 ms。同步时会取当前帧点云中最大的 `curvature`，也就是这一帧最后一个点相对帧起点的时间差，然后推出点云结束时间：

```text
lidar_end_time = lidar_beg_time + max_point_curvature / 1000.0
```

这样就得到了这一帧点云的完整扫描时间区间 `[lidar_beg_time, lidar_end_time]`，后面才能知道应该从 `imu_deque` 中取哪一段 IMU 数据。

同步逻辑可以理解成三步：

1. 从 `lidar_buffer` 取出一帧点云，并从 `time_buffer` 取出它的起始时间。
2. 根据帧内最大 `curvature` 算出 `lidar_end_time`。
3. 从 `imu_deque` 中取出时间覆盖这段扫描区间的 IMU 数据，填入 `meas.imu`。

当 `imu_en` 为 true 时，`sync_packages()` 还有一个关键的“安全锁”：只有 IMU 队列中最新一条数据的时间已经大于或等于 `lidar_end_time`，才说明这帧点云扫描期间的 IMU 数据已经收全，可以返回 `true` 进入主处理循环。如果 IMU 还没覆盖到点云结束时间，就先等待后续 IMU 消息，避免拿半截惯性数据去做运动补偿和状态估计。

在 RM 场景下，这一步尤其关键。高速旋转或急停时，一帧点云内部的姿态变化会被放大；固态雷达或低线数雷达又可能存在点云分布不均的问题。同步后的 `MeasureGroup` 相当于给点云补上“扫描时间说明书”，让后续算法知道这帧点云从什么时候扫到什么时候，以及这段时间里机器人姿态如何变化。

## 6. feats_undistort: 当前版本的去畸变现状

主循环中先调用：

```cpp
p_imu->Process(Measures, feats_undistort);
```

需要注意，当前仓库版本的 `ImuProcess::Process()` 基本只是完成 IMU 初始化并把 `meas.lidar` 复制到 `feats_undistort`：

```cpp
*cur_pcl_un_ = *(meas.lidar);
```

也就是说，虽然变量名叫 `feats_undistort`，但这里没有看到常见的逐点反向运动补偿逻辑。后续的状态传播与点到平面更新是在 `laserMapping.cpp` 中按点时间分段完成的。

## 7. PCL VoxelGrid 下采样

`src/laserMapping.cpp` 中定义了两个 PCL 体素滤波器：

```cpp
pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterMap;
```

启动时设置叶子大小：

```cpp
downSizeFilterSurf.setLeafSize(filter_size_surf_min,
                               filter_size_surf_min,
                               filter_size_surf_min);
downSizeFilterMap.setLeafSize(filter_size_map_min,
                              filter_size_map_min,
                              filter_size_map_min);
```

每帧处理中，如果 `space_down_sample` 为 true：

```cpp
downSizeFilterSurf.setInputCloud(feats_undistort);
downSizeFilterSurf.filter(*feats_down_body);
sort(feats_down_body->points.begin(), feats_down_body->points.end(), time_list);
```

因此，本仓库 Point-LIO 的输入点云主路径中确实有 PCL `VoxelGrid` 体素降采样。默认仿真和实车参数中：

```yaml
space_down_sample: True
filter_size_surf: 0.2
filter_size_map: 0.2
```

## 8. 按点时间分段

下采样后的 `feats_down_body` 会按 `curvature` 排序，然后通过 `time_compressing()` 生成 `time_seq`。

`time_seq` 记录一段连续相同或非递增点时间中的点数。主循环随后按这些分段推进状态、更新滤波器并转换对应段的点云。

```cpp
time_seq = time_compressing<int>(feats_down_body);
feats_down_size = feats_down_body->points.size();
```

## 9. 地图初始化

在 `init_map == false` 时，系统先把当前点云转到世界系：

```cpp
pointBodyToWorld(&(feats_undistort->points[i]),
                 &(feats_down_world->points[i]));
```

累计点数达到 `init_map_size` 后：

- 如果 `prior_pcd.enable` 为 true，则从 PCD 加载先验地图
- 否则将当前累计点云加入 iVox

```cpp
ivox_->AddPoints(init_feats_world->points);
```

初始化后的地图也会通过 `Laser_map` 发布一次。

## 10. iVox 最近邻平面匹配

滤波更新的观测模型在 `src/Estimator.cpp` 中：

1. 将体坐标点 `feats_down_body` 转到世界系 `feats_down_world`
2. 用 iVox 查找最近邻：

```cpp
ivox_->GetClosestPoint(point_world_j, points_near, NUM_MATCH_POINTS);
```

3. `NUM_MATCH_POINTS` 当前为 5
4. 用 `esti_plane()` 拟合平面
5. 检查点到平面距离：

```cpp
if (p_norm > match_s * pd2 * pd2) {
  point_selected_surf[...] = true;
}
```

6. 对有效点构造点到平面残差 `z` 和雅可比 `h_x`
7. 交给 ESKF 的 `update_iterated_dyn_share_modified()` 做迭代更新

平面拟合函数 `esti_plane()` 使用 5 个邻近点解线性方程，得到平面参数：

```text
ax + by + cz + d = 0
```

并用 `plane_thr` 检查这些邻近点是否足够共面。

## 11. 点云坐标转换

点从雷达/body 系到世界系的转换在 `pointBodyToWorld()`：

```cpp
p_global = state.rot * (Lidar_R_wrt_IMU * p_body + Lidar_T_wrt_IMU) + state.pos;
```

如果 `extrinsic_est_en` 为 true，则使用滤波器状态中的外参；否则使用参数中的 `extrinsic_R` 和 `extrinsic_T`。

转换后的世界系点写入 `feats_down_world`。

## 12. 增量写入 iVox 地图

每帧状态更新完成后，`MapIncremental()` 将 `feats_down_world` 写入 iVox。

它并不是无条件把所有点加入地图，而是先用 `filter_size_map_min` 做一个体素中心检查：

```cpp
center = (floor(point_world / filter_size_map_min) + 0.5) * filter_size_map_min;
```

如果近邻点中已经有点落在该体素中心附近，则跳过；否则加入 `points_to_add`。

最后调用：

```cpp
ivox_->AddPoints(points_to_add);
```

iVox 内部再按 `mapping.ivox_grid_resolution` 建格，默认仿真和实车都是 `0.5`。邻域搜索范围由 `ivox_nearby_type` 控制，可选 CENTER、NEARBY6、NEARBY18、NEARBY26。

## 13. 输出

Point-LIO 主要发布：

- `aft_mapped_to_init`: 里程计
- `cloud_registered`: 世界系下采样点云，即 `feats_down_world`
- `cloud_registered_body`: 可选，body/IMU 系点云
- `Laser_map`: 初始化地图点云
- `path`: 可选路径

`cloud_registered` 发布逻辑：

```cpp
pcl::toROSMsg(*feats_down_world, laserCloudmsg);
laserCloudmsg.header.frame_id = "camera_init";
pubLaserCloudFullRes->publish(laserCloudmsg);
```

如果 `pcd_save.pcd_save_en` 为 true，会把 `feats_down_world` 累计后用 `pcl::PCDWriter` 保存到 `point_lio/PCD/`。

## 14. 调参入口速查

常用点云相关参数：

| 参数 | 作用 |
| --- | --- |
| `point_filter_num` | 预处理阶段按点序抽点 |
| `preprocess.blind` | 近距离点过滤 |
| `mapping.det_range` | 远距离点过滤 |
| `space_down_sample` | 是否启用 PCL VoxelGrid |
| `filter_size_surf` | 输入扫描点云体素降采样叶子大小 |
| `filter_size_map` | 增量入图时的地图体素去重尺度 |
| `mapping.ivox_grid_resolution` | iVox 地图网格分辨率 |
| `ivox_nearby_type` | 最近邻搜索使用的邻域格范围 |
| `mapping.plane_thr` | 最近邻平面拟合阈值 |
| `mapping.match_s` | 点到平面残差筛选尺度 |
| `publish.scan_publish_en` | 是否发布 `cloud_registered` |
| `pcd_save.pcd_save_en` | 是否保存 PCD |

## 15. 一句话结论

本仓库 `point_lio` 的 PCL 主路径是：先把 ROS 点云统一成 `pcl::PointCloud<pcl::PointXYZINormal>`，用预处理和 `point_filter_num` 做基础过滤，再通过 PCL `VoxelGrid` 做扫描级体素降采样，随后用 iVox 维护地图并做最近邻平面匹配，最后发布世界系下采样点云 `cloud_registered` 并可选保存 PCD。
