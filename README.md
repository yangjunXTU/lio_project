<!--
 * @Author: yangjun_d 295967654@qq.com
 * @Date: 2025-08-25 07:09:51
 * @LastEditors: yangjun_d 295967654@qq.com
 * @LastEditTime: 2025-10-22 02:51:07
 * @FilePath: /lio_project_wk/src/lio_project/README.md
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
-->
# LIO Project 学习练习项目

个人学习练习项目，参考高翔《自动驾驶与机器人中的SLAM技术》与 FAST-LIVO2 的工程思路，逐步实现 IMU 预测、NDT-LIO、视觉光度残差紧耦合、语义点云显示与 PCD 保存等模块。

## 参考资料

1、FAST-LVIO2：[https://github.com/hku-mars/FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2)\
2、SLAM in Autonomous Driving Book (SAD Book)：[https://github.com/gaoxiang12/slam_in_autonomous_driving](https://github.com/gaoxiang12/slam_in_autonomous_driving)  \
3、slam十四讲代码：[https://github.com/gaoxiang12/slambook2](https://github.com/gaoxiang12/slambook2)

## 一、工程定位与总体方法

1、系统类型：紧耦合 LIO/VIO（ESKF 统一后端）；\
2、状态估计后端：IESKF，统一处理 IMU 预测 + LiDAR/NDT 更新 + 视觉光度更新；\
3、LiDAR 前端：去畸变 + 体素下采样 + 增量 NDT；\
4、视觉前端：SVO 风格多层金字塔 patch 光度法，视觉约束直接写入 ESKF 的信息矩阵；\
5、地图与可视化：几何地图、RGB 点云、VIO 高亮语义点云并行维护。

## 二、坐标系与状态定义

1、坐标系采用 `T_a_b` 记号：表示将 b 系点变换到 a 系；\
2、核心坐标系：世界系 `W`、IMU 系 `I`、LiDAR 系 `L`、相机系 `C`；\
3、ESKF 名义状态（18维误差状态对应）：`p, v, R, bg, ba, g`；\
4、主状态位姿语义为 `T_w_i`，LiDAR 与视觉观测最终都约束该状态；\
5、外参链路：`T_i_l`、`T_c_l`，内部组合得到 `T_c_i`。

## 三、系统主流程（按代码执行顺序）

1、数据回调与缓存：LiDAR/IMU/图像分别入队；\
2、同步器 `sync_packages(...)` 按时间对齐生成一组 `LidarMeasureGroup`；\
3、`ProcessIMU()`：IMU 初始化完成前做静态初始化，完成后做逐帧预测；\
4、`ProcessLidar()`：先 `Undistort()` 去畸变，再 `Align()` 做 NDT+ESKF 更新；\
5、`ProcessCamera()`：视觉帧进入 VIO，构造光度残差并通过 `UpdateUsingCustomObserve(...)` 更新 ESKF；\
6、发布里程计、路径、点云；\
7、按配置执行 PCD 保存（周期或退出保存）。

## 四、核心模块逻辑

### 4.1 IMU 初始化与预测

1、初始化：统计静止阶段陀螺/加计均值与方差，估计初始 `bg, ba, g`；\
2、预测：对每个 IMU 采样执行一次状态传播。

核心公式（离散形式）：

```math
R_{k+1}=R_k\exp\left((\omega_k-b_g)\Delta t\right)
```

```math
v_{k+1}=v_k+\left(R_k(a_k-b_a)+g\right)\Delta t
```

```math
p_{k+1}=p_k+v_k\Delta t+\frac{1}{2}\left(R_k(a_k-b_a)+g\right)\Delta t^2
```

### 4.2 LiDAR 去畸变与 NDT 更新

1、去畸变：用 IMU 轨迹插值到点时间，补偿到帧末时刻；\
2、下采样：`VoxelGrid` 降采样当前 scan；\
3、NDT：在世界系体素地图中计算点到分布残差；\
4、将 NDT 残差写为信息形式 `HTVH, HTVr`，交给 ESKF 迭代更新。

核心更新形式：

```math
\delta x = \left(P^{-1}+H^TVH\right)^{-1}\left(H^TVr\right)
```

其中 LiDAR 和视觉都通过同一接口 `UpdateUsingCustomObserve(...)` 提供 `H^TVH` 与 `H^TVr`。

### 4.3 视觉光度紧耦合（VIO）

1、当前帧建图像金字塔，投影世界点筛选视觉候选；\
2、采用网格均匀采样、梯度/深度门限剔除；\
3、多层 patch 光度残差（coarse-to-fine）；\
4、解析雅可比链式求导，直接映射到 ESKF 18 维误差状态中的位置和姿态块；\
5、通过 tracking stats 做更新闸门与参考帧提升闸门。

光度残差：

```math
r = I_{ref}(u_{ref}) - I_{cur}(u_{cur})
```

雅可比链式关系：

```math
\frac{\partial r}{\partial x}
=-\frac{\partial I}{\partial u}
\cdot \frac{\partial u}{\partial P_c}
\cdot \frac{\partial P_c}{\partial x}
```

其中 `x` 为误差状态，当前实现主要填充位置与旋转对应列。

### 4.4 语义点云显示与地图保存

1、`/map/map_point_rgb`：每帧 RGB 点云（仅保留可投影且能取到颜色的点）；\
2、`/vio/map_semantic`：在 RGB 点云基础上叠加 VIO 选中点高亮；\
3、显示链路支持体素下采样和点数上限，避免 RViz 卡顿；\
4、保存链路支持几何与 RGB 地图分别保存：\
`all_raw_points.pcd`、`all_rgb_points.pcd`。

## 五、关键配置说明（MID360.yaml）

1、`extrinsics/*`：IMU-LiDAR-相机外参；\
2、`camera/*`：相机模型、分辨率、内参与畸变；\
3、`imu/*`：静态初始化与噪声相关参数；\
4、`ndt/*`：体素地图与匹配参数；\
5、`eskf/*`：迭代次数、噪声、bias 更新策略；\
6、`vio/*`：金字塔、patch、门限、信息权重与关键帧策略；\
7、`semantic/*`：语义点云发布与显示降采样参数；\
8、`pcd_save/*`：几何/RGB 点云保存开关与周期保存策略。

## 六、工程输出与可视化建议

1、里程计：`/aft_mapped_to_map`；\
2、几何点云：`/mid360/ndt_clod`；\
3、RGB 点云：`/map/map_point_rgb`；\
4、语义高亮点云：`/vio/map_semantic`；\
5、RViz 点云显示建议：`Color Transformer = RGB8`，`Fixed Frame = map`。

## 七、当前实现特点与边界

1、优势：框架清晰、模块分离，LiDAR 与视觉统一在 ESKF 信息形式下融合；\
2、优势：视觉链路已具备金字塔、解析雅可比、质量闸门，稳定性较早期方案明显提升；\
3、边界：RGB 地图目前按观测累计，尚未做完整的多视角颜色融合与遮挡一致性优化；\
4、边界：工程以学习和实验为主，实际部署前建议补充更系统的性能评估与异常工况测试。

## 八、问题记录

1、[issuse_disscuss](./doc/issuse_discuss.md)
