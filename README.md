<!--
 * @Author: yangjun_d 295967654@qq.com
 * @Date: 2025-08-25 07:09:51
 * @LastEditors: yangjun_d 295967654@qq.com
 * @LastEditTime: 2025-09-01 02:49:30
 * @FilePath: /lio_project_wk/src/lio_project/README.md
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
-->
# LIO Project 学习练习项目

个人学习练习项目，逐步实现IMU积分、特征提取、匹配（ndt、icp等）、lio里程计、紧耦合融合slam等模块；

## 参考资料：
1、FAST-LVIO2：[https://github.com/hku-mars/FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2)\
2、SLAM in Autonomous Driving Book (SAD Book)：[https://github.com/gaoxiang12/slam_in_autonomous_driving](https://github.com/gaoxiang12/slam_in_autonomous_driving)

## 一、已完成功能：\
1、框架搭建；\
2、回调函数数据读取；\
3、lidar数据custom点云类型转pcl；\
4、imu数据读取，imu数据结构构造；\
5、建立同步器结构体；\
6、同步器函数创建；\
7、imu和雷达数据时间戳对齐；\
8、imu数据初始化；\
9、ieskf结构构造；\
10、名义状态变量预测；\
11、ndt匹配；\
12、ieskf更新；\
13、地图创建；\
14、增加地图保存功能;\
15、可视化数据、path、里程计等；

## 问题记录：[issuse_disscuss](./doc/issuse_discuss.md)