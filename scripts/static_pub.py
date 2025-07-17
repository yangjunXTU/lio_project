import rospy
import tf.transformations
import tf2_ros
from geometry_msgs.msg import TransformStamped,PointStamped
import tf

rospy.init_node("static_pub")
pub = tf2_ros.StaticTransformBroadcaster()

ts = TransformStamped()  # 坐标系的转换,PointStamped=坐标点的坐标转换

ts.header.stamp = rospy.Time.now()
ts.header.frame_id = "lidar"

ts.child_frame_id = "camera"

ts.transform.translation.x = 0
ts.transform.translation.y = 0.08
ts.transform.translation.z = 1.35

qtn = tf.transformations.quaternion_from_euler(0,0,0)  #(-3.14,0,-1.57)
ts.transform.rotation.x = qtn[0]
ts.transform.rotation.y = qtn[1]
ts.transform.rotation.z = qtn[2]
ts.transform.rotation.w = qtn[3]

pub.sendTransform(ts)

rospy.spin()