import rospy
import tf2_ros
from tf2_geometry_msgs import tf2_geometry_msgs

rospy.init_node("dynamic_sub")

buffer = tf2_ros.Buffer()

sub = tf2_ros.TransformListener(buffer)

# 组织被转换的坐标点(雷达坐标系中的点)
ps = tf2_geometry_msgs.PointStamped()

# 注意这个地方要把时间戳设置为0
ps.header.stamp = rospy.Time()
ps.header.frame_id = "tag"
ps.point.x = 0
ps.point.y = 0
ps.point.z = 0


# 转化逻辑实现
rate = rospy.Rate(10)
while not rospy.is_shutdown():
    try:
        # 参数1：被转换的坐标点，参数2：目标坐标系，返回值：转换后的坐标点
        ps_out = buffer.transform(ps, "world")
        # 输出结果
        rospy.loginfo("转化后的点（%.2f, %.2f, %.2f),参考的坐标系：%s",
                    ps_out.point.x,
                    ps_out.point.y,
                    ps_out.point.z,
                    ps_out.header.frame_id)
    except Exception as e:
        rospy.logwarn("错误提示:%s", e)
    
    rate.sleep()