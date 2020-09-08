#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt


class TurtleBot:

    def __init__(self):
        rospy.init_node('turtlebot_controller', anonymous=True)
        self.velocity_publisher = rospy.Publisher(
            '/turtle1/cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber(
            '/turtle1/pose', Pose, self.update_pose)

        self.pose = Pose()
        self.rate = rospy.Rate(5)

    def update_pose(self, data):
        self.pose = data
        self.pose.x = round(self.pose.x, 5)
        self.pose.y = round(self.pose.y, 5)

    def euclidean_distance(self, goal_pose):
        return sqrt(pow(float((goal_pose.x - self.pose.x)), 2) + pow(float((goal_pose.y - self.pose.y)), 2))

    def angular_vel(self, goal_pose):
    	my_theta = atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    	return my_theta - self.pose.theta
        

    def move2goal(self):
        goal_pose = Pose()
        vel_msg = Twist()

        x = 0

        while True:  
            if(x == 1):
                print("Wrong Input. Try Again: \n")
            try:
                goal_pose.x = float(input("Enter X : "))
                goal_pose.y = float(input("Enter Y : "))
            except Exception as e:
                continue
            if(goal_pose.x > 0 and goal_pose.x < 11 and goal_pose.y > 0 and goal_pose.y < 11):  
                break  
            else:
                x = 1
        

        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = self.angular_vel(goal_pose)

        self.velocity_publisher.publish(vel_msg)
        rospy.sleep(2.5)

        while self.euclidean_distance(goal_pose) > 0.40:
            vel_msg.linear.x = 1.5
            vel_msg.angular.z = 0.0
            self.velocity_publisher.publish(vel_msg)
            rospy.loginfo("euclidean_distance : " + str(self.euclidean_distance(goal_pose)) + "\n")

        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)


if __name__ == '__main__':
    try:
        x = TurtleBot()
        x.move2goal()
    except rospy.ROSInterruptException:
        pass
