 #!/usr/bin/env python
#coding: utf-8
import rospy # 导入ROS Python库
import actionlib #导入ROS Action库导入ROS Action相关消息
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal # 导入导航相关消息
from nav_msgs.msg import Path #导入路径消息
from geometry_msgs.msg import PoseWithCovarianceStamped #导入带协方差的姿态消息
from tf_conversions import transformations #导入坐标转换库
from math import pi #导入pi常量
from std_msgs.msg import String #导入字符串消息
from ar_track_alvar_msgs.msg import AlvarMarkers #入AR标记消息
from ar_track_alvar_msgs.msg import AlvarMarker #导入单个AR标记消息
from geometry_msgs.msg import Point #导入Point消息
import sys #导入sys模块
reload(sys) #重新加载sys模块
sys.setdefaultencoding('utf-8')# 设置默认编码为utf-8
import os # 导入os模块

music = {
    "find_1": "/home/abot/voice/find_1.mp3",
    "find_2": "/home/abot/voice/find_2.mp3",
    "find_3": "/home/abot/voice/find_3.mp3",
    "find_4": "/home/abot/voice/find_4.mp3",
    "find_5": "/home/abot/voice/find_5.mp3",
    "find_6": "/home/abot/voice/find_6.mp3",
    "find_7": "/home/abot/voice/find_7.mp3",
    "find_8": "/home/abot/voice/find_8.mp3",
    "arrive_1": "/home/abot/voice/arrive_1.mp3",
    "arrive_2": "/home/abot/voice/arrive_2.mp3",
    "arrive_3": "/home/abot/voice/arrive_3.mp3",
    "arrive_4": "/home/abot/voice/arrive_4.mp3",
    "arrive_5": "/home/abot/voice/arrive_5.mp3",
    "arrive_6": "/home/abot/voice/arrive_6.mp3",
    "arrive_7": "/home/abot/voice/arrive_7.mp3",
    "arrive_8": "/home/abot/voice/arrive_8.mp3",
    "arrive_end": "/home/abot/voice/arrive_end.mp3",
    "start": "/home/abot/voice/start.mp3"
}

# AR标记的ID
id = 255
find=0
class navigation_demo:
    def init (self):
        self.set_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=5)
        self.arrive_pub = rospy.Publisher('/voiceWords', String, queue_size=10)
        self.ar_sub = rospy.Subscriber('/object_position',Point, self.ar_cb);
        self.move_base = actionlib.SimpleActionClient("move_base".MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(60))
    def ar_cb(self, data):
        global id
        for marker in data.markers:
            id = marker.id

    def find_cb(self, data):
        global find
        point_msg = data
        if point_msg.z >= 1 and point_msg.z <= 10:
            find = 1
        elif point_msg.z <= 20:
            find = 2
        elif point_msg.z <= 30:
            find = 3
        elif point_msg.z <= 40:
            find = 4
        elif point_msg.z <= 50:
            find = 5
        elif point_msg.z <= 60:
            find = 6
        elif point_msg.z <= 70:
            find = 7
        elif point_msg.z <= 80:
            find = 8

    def set_pose(self, p):
        if seDf.move_base is None:
            return False

        x, y, th = p
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id ='map'
        pose.pose.pose.position.x = x
        pose.pose.pose.position.y = y
        q = transformations.quaternion_from_euler(0.0, 0.0, th / 180.0 * pi)
        pose.pose.pose.orientation.x = q[0]
        pose.pose.pose.orientation.y = q[1]
        pose.pose.pose.orientation.z = q[2]
        pose.pose.pose.orientation.w = q[3]
        self.set_pose_pub.publish(pose)
        return True

    def _done_cb(self, status, result):
        rospy.loginfo("navigation done! status:%d result:%s" % (status, result))
        self.arrive_pub.publish("arrived to traget point")

    def _active_cb(self):
            rospy.loginfo("[Navi] navigation has be actived")

    def _feedback_cb(self, feedback):
        msg = feedback

    def cancel(self):
        self.move_base.cancel_all_goals()
        return True
    def goto(self, p):
        rospy.loginfo("[Navi] goto %s" % p)
        goal = MoveBaseGoal()

        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = p[0]
        goal.target_pose.pose.position.y = p[1]
        q = transformations.quaternion_from_euler(0.0, 0.0, p[2] / 180.0 * pi)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.move_base.send_goal(goal, self._done_cb, self._active_cb, self._feedback_cb)
        result = self.move_base.wait_for_result(rospy.Duration(60))
        if not result:
            self.move_base.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
        else:
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("reach goal %s succeeded!" % p)
        return True

    def move_robot(linear_x,  linear_y, angular_z):
        velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rate = rospy.Rate(10)
        moving_time = 2
        start_time = time.time()
        vel_msg = Twist()
        vel_msg.linear.x = linear_x
        vel_msg.angular.z = angular_z
        vel_msg.linear.y = linear_y
        while not rospy.is_shutdown() and time.time() - start_time < moving_time:
            velocity_publisher.publish(vel_msg)
            rate.sleep()

if __name_ == "__main_":
    rospy.init_node('navigation_demo', anonymous=True)
    goalListx = rospy.get_param('~goalListx','2.0,2.0')
    goalListY = rospy.get_param('~goalListY','2.0,4.0')
    goallistYaw = rospy.get_param('~goalListYaw','0,90.0')

    goals = [[float(x),float(y), float(yaw)] for (x, y, yaw) in
    zip(goalListx.split(","),goallistY.split(","),goallistYaw.split("，"))]
    print("Please enter 1 to continue:")
    user_input = input()
    print(goals)
    r = rospy.Rate(1)
    r.sleep()
    navi = navigation_demo()
    # 区域一
    os.system('mplayer %s' % music[start])
    navi.goto(goals[0])
    rospy.sleep(1)
    if id == 1 or find == 1:
        os.system('mplayer %s' % music[find_1])
        navi.goto(goals[1])
        os.system('mplayer %s' % music[arrive_1])
        rospy.sleep(1)
    if id == 2 or find == 2:
        os.system('mplayer %s' % music[find_2])
        navi.goto(goals[2])
        os.system('mplayer %s' % music[arrive_2])
        rospy.sleep(1)

    # 区域二
    #    navi.goto(goals[13])
    navi.goto(goals[3])
    rospy.sleep(1)
    if id == 3 or find == 3:
        os.system('mplayer %s' % music[find_3])
        navi.goto(goals[4])
        os.system('mplayer %s' % music[arrive_3])
        rospy.sleep(1)
    if id == 4 or find == 4:
        os.system('mplayer %s' % music[find_4])
        navi.goto(goals[5])
        os.system('mplayer %s' % music[arrive_4])
        rospy.sleep(1)

    # 区域三
    navi.goto(goals[13])
    #    rospy.sleep(1)
    navi.goto(goals[6])
    rospy.sleep(1)
    if id == 5 or find == 5:
        os.system('mplayer %s' % music[find_5])
        navi.goto(goals[7])
        os.system('mplayer %s' % music[arrive_5])
        rospy.sleep(1)
    if id == 6 or find == 6:
        os.system('mplayer %s' % music[find_6])
        navi.goto(goals[8])
        os.system('mplayer %s' % music[arrive_6])
        rospy.sleep(1)
    #区域四
    navi.goto(goals[14])
    navi.goto(goals[9])
    rospy.sleep(1)
    if id == 7 or find == 7:
        os.system('mplayer %s' % music[find_7])
        navi.goto(goals[10])
        os.system('mplayer %s' % music[arrive_7])
        rospy.sleep(1)
    if id == 8 or find == 8:
        os.system('mplayer %s' % music[find_8])
        navi.goto(goals[11])
        os.system('mplayer %s' % music[arrive_8])
        rospy.sleep(1)

    # 终点
    navi.goto(goals[12])
    rospy.sleep(1)

    # 执行move_robot程序
    move_robot(0.20, 0.20, 0.0)
    os.system('mplayer %s' % music[arrive_end])
    while not rospy.is_shutdown():
        r.sleep()