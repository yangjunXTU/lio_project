import apriltag_ros.msg
import rospy
import tf.transformations
from turtlesim.msg import Pose
import tf2_ros
from geometry_msgs.msg import TransformStamped
import tf
import apriltag_ros

def doPose(detection):
    # apriltag_ros.msg.AprilTagDetectionArray.detections.count()
    if(detection.detections != []):
        pub = tf2_ros.TransformBroadcaster()

        ts = TransformStamped()
        # ts.header.stamp = detection.header.stamp
        # ts.header.frame_id = "camera"
        # ts.child_frame_id = "tag"

        # if(detection.detections != []):
        ts.header.stamp = detection.header.stamp
        ts.header.frame_id = "camera"
        ts.child_frame_id = "tag"
        for d in detection.detections:
            # ts.header.stamp = rospy.Time.now()
            # ts.header.frame_id = "camera"
            # ts.child_frame_id = detection.header.frame_id
            ts.transform.translation.x = d.pose.pose.pose.position.x
            ts.transform.translation.y = d.pose.pose.pose.position.y
            ts.transform.translation.z = d.pose.pose.pose.position.z

            ts.transform.rotation.x = d.pose.pose.pose.orientation.x
            ts.transform.rotation.y = d.pose.pose.pose.orientation.y
            ts.transform.rotation.z = d.pose.pose.pose.orientation.z
            ts.transform.rotation.w = d.pose.pose.pose.orientation.w

            rospy.loginfo("转化前的点（%.2f, %.2f, %.2f),参考的坐标系：%s",
                          ts.transform.translation.x,ts.transform.translation.y,
                          ts.transform.translation.z,ts.header.frame_id)
            rospy.loginfo("转化前的pose（%.2f, %.2f, %.2f, %.2f)",
                          d.pose.pose.pose.orientation.x,d.pose.pose.pose.orientation.y,
                          d.pose.pose.pose.orientation.z,d.pose.pose.pose.orientation.w)
        # else:
        #     print("detection.detections.count() <= 0")


        pub.sendTransform(ts)
    else:
            print("detection.detections.count() <= 0")

rospy.init_node("apriltag_sub")

sub = rospy.Subscriber("/tag_detections", apriltag_ros.msg.AprilTagDetectionArray,doPose,queue_size=100)
rospy.spin()

