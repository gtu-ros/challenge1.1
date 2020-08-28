# !/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import atan, sqrt

class Turtle:

    def __init__(self):

        rospy.init_node('Challange',anonymous=True)

        self.vel_pub = rospy.Publisher('/turtle1/cmd_vel',Twist,queue_size=10)
        self.pose_subs = rospy.Subscriber('turtle1/pose',Pose, self.update_pose)
        self.rate = rospy.Rate(10)
        self.pose = Pose()

    def update_pose(self,data):

        self.pose = data
        self.pose.x = round(self.pose.x,0)
        self.pose.y = round(self.pose.y,0)
    
    def calc_region(self, goal_pose):

        if goal_pose.y > self.pose.y:

            if goal_pose.x > self.pose.x:
                return 1
            else:
                return 2    

        elif goal_pose.y < self.pose.y:

            if goal_pose.x > self.pose.x:
                return 4
            else:
                return 3                    

    def calc_angular(self, goal_pose):
        
        m = 0.0

        if goal_pose.x != self.pose.x:
            m = (goal_pose.y - self.pose.y) / (goal_pose.x - self.pose.x)

        theta = atan(m)*57.2957795
        angular = 0.0
       
        if m != 0.0:

           if self.calc_region(goal_pose) == 2:
               theta = 90.0 - theta

           if self.calc_region(goal_pose) == 3:
               theta = -90.0 - theta

        else:

            if goal_pose.x < self.pose.x:
                theta = 180.0
            
            if goal_pose.y > self.pose.y:
                theta = 90.0

            if goal_pose.y < self.pose.y:
                theta = -90.0    

        angular = 6.24 * theta / 360
        print(angular)
        return angular - self.pose.theta
        
    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow(float((goal_pose.x - self.pose.x)), 2) + pow(float((goal_pose.y - self.pose.y)), 2))


    def move2goal(self):    

        vel_msg = Twist()
        goal_pose = Pose()

        goal_pose.x = int(input("Enter x: "))
        goal_pose.y = int(input("Enter y: "))

        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0

        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = self.calc_angular(goal_pose)

        self.vel_pub.publish(vel_msg)
        rospy.sleep(2)
        
        while self.euclidean_distance(goal_pose) > 0:

            vel_msg.linear.x = 1.0
            vel_msg.angular.z = 0.0
            self.vel_pub.publish(vel_msg)

        vel_msg.linear.x = 0
        self.vel_pub.publish(vel_msg)


        rospy.spin()


if __name__ == '__main__':
    try:
        x = Turtle()
        x.move2goal()

    except rospy.ROSInterruptException:
        pass