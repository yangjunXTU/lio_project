import rospy
import tf2_ros
from tf2_geometry_msgs import tf2_geometry_msgs

rospy.init_node("static_sub")

buffer = tf2_ros.Buffer()

sub = tf2_ros.TransformListener(buffer)

ps = tf2_geometry_msgs.PointStamped()

ps.header.stamp = rospy.Time.now()
ps.header.frame_id = "camera"

ps.point.x = 2.0
ps.point.y = 3.0
ps.point.z = 4.0

rate = rospy.Rate(10)

while not rospy.is_shutdown():
    try:
        ps_out = buffer.transform(ps,"lidar")
        rospy.loginfo("转化后的点（%.2f, %.2f, %.2f),参考的坐标系：%s",
                    ps_out.point.x,
                    ps_out.point.y,
                    ps_out.point.z,
                    ps_out.header.frame_id)
    except Exception as e:
        rospy.logwarn("错误提示:%s", e)

rate.sleep()