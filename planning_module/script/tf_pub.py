#!/usr/bin/env python3.8
import rospy
import tf
from nav_msgs.msg import Odometry
from planning_module.msg import PathData

class OdomToTF:
    def __init__(self):
        # Odometry 데이터를 구독
        self.odom_sub = rospy.Subscriber('/odometry_gt', Odometry, self.odom_callback)
        self.odom_sub = rospy.Subscriber('/path_with_odom', PathData, self.path_odom_callback)
        # tf 브로드캐스터 초기화
        self.tf_broadcaster = tf.TransformBroadcaster()

    def odom_callback(self, msg):
        # Odometry 메시지에서 위치 (position) 및 방향 (orientation) 추출
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        # position: (x, y, z) 좌표
        pos = (position.x, position.y, position.z)
        
        # orientation: (x, y, z, w) 사원수 (quaternion)
        orient = (orientation.x, orientation.y, orientation.z, orientation.w)

        # 현재 시간을 가져와서 변환을 publish
        current_time = rospy.Time.now()

        # 변환을 publish (map -> base_link)
        self.tf_broadcaster.sendTransform(
            pos,                    # 위치
            orient,                 # 방향 (사원수)
            current_time,           # 시간
            "base_link",            # 자식 프레임
            "map"                   # 부모 프레임
        )

    def path_odom_callback(self, msg):
        # Odometry 메시지에서 위치 (position) 및 방향 (orientation) 추출
        odom = msg.path_odom
        position = odom.pose.pose.position
        orientation = odom.pose.pose.orientation

        # position: (x, y, z) 좌표
        pos = (position.x, position.y, position.z)
        
        # orientation: (x, y, z, w) 사원수 (quaternion)
        orient = (orientation.x, orientation.y, orientation.z, orientation.w)

        # 현재 시간을 가져와서 변환을 publish
        current_time = rospy.Time.now()

        # 변환을 publish (map -> base_link)
        self.tf_broadcaster.sendTransform(
            pos,                    # 위치
            orient,                 # 방향 (사원수)
            current_time,           # 시간
            "path_odom",            # 자식 프레임
            "map"                   # 부모 프레임
        )


if __name__ == '__main__':
    # ROS 노드 초기화
    rospy.init_node('odom_to_tf_broadcaster')

    # 클래스 인스턴스 생성
    odom_to_tf = OdomToTF()

    # ROS 노드가 종료될 때까지 대기
    rospy.spin()
