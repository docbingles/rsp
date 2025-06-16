#!/usr/bin/env python
import rospy
import time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from enum import Enum

class RobotState(Enum):
    IDLE = 0
    NAV_PICKUP = 1
    PICKUP = 2
    NAV_DELIVERY = 3
    DELIVERY_COMPLETE = 4

class DeliveryRobot:
    def __init__(self):
        rospy.init_node('delivery_robot_fsm')
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        self.state = RobotState.IDLE
        self.state_start_time = time.time()
        self.twist = Twist()
        self.obstacle_threshold = 0.5
        self.min_front_distance = float('inf')
        self.pickup_distance = 5.0
        self.delivery_distance = 10.0
        self.distance_travelled = 0.0
        self.linear_speed = 0.2
        self.rate = rospy.Rate(10)
        rospy.loginfo("Delivery Robot FSM started. Waiting to start...")

    def lidar_callback(self, data):
        front_angles = data.ranges[len(data.ranges)//3 : 2*len(data.ranges)//3]
        valid_ranges = [r for r in front_angles if r > 0.0 and r < float('inf')]
        if valid_ranges:
            self.min_front_distance = min(valid_ranges)
        else:
            self.min_front_distance = float('inf')

    def stop_robot(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.cmd_pub.publish(self.twist)

    def move_forward(self):
        if self.min_front_distance > self.obstacle_threshold:
            self.twist.linear.x = self.linear_speed
            self.twist.angular.z = 0.0
        else:
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.5
        self.cmd_pub.publish(self.twist)

    def fsm_loop(self):
        while not rospy.is_shutdown():
            if self.state == RobotState.IDLE:
                rospy.loginfo_throttle(5, "State: IDLE - Waiting for task...")
                if time.time() - self.state_start_time > 5:
                    self.state = RobotState.NAV_PICKUP
                    self.state_start_time = time.time()
                    self.distance_travelled = 0.0
                    rospy.loginfo("Starting navigation to pickup location.")
                self.stop_robot()

            elif self.state == RobotState.NAV_PICKUP:
                rospy.loginfo_throttle(2, "Navigating to pickup... Distance travelled: %.2f m" % self.distance_travelled)
                self.move_forward()
                self.distance_travelled += self.linear_speed * 0.1
                if self.distance_travelled >= self.pickup_distance:
                    self.state = RobotState.PICKUP
                    self.state_start_time = time.time()
                    self.stop_robot()
                    rospy.loginfo("Arrived at pickup location.")

            elif self.state == RobotState.PICKUP:
                rospy.loginfo_throttle(5, "Picking up item...")
                self.stop_robot()
                if time.time() - self.state_start_time > 3:
                    self.state = RobotState.NAV_DELIVERY
                    self.state_start_time = time.time()
                    self.distance_travelled = 0.0
                    rospy.loginfo("Item picked up. Navigating to delivery location.")

            elif self.state == RobotState.NAV_DELIVERY:
                rospy.loginfo_throttle(2, "Navigating to delivery... Distance travelled: %.2f m" % self.distance_travelled)
                self.move_forward()
                self.distance_travelled += self.linear_speed * 0.1
                if self.distance_travelled >= self.delivery_distance:
                    self.state = RobotState.DELIVERY_COMPLETE
                    self.state_start_time = time.time()
                    self.stop_robot()
                    rospy.loginfo("Arrived at delivery location.")

            elif self.state == RobotState.DELIVERY_COMPLETE:
                rospy.loginfo_throttle(5, "Delivering item...")
                self.stop_robot()
                if time.time() - self.state_start_time > 3:
                    rospy.loginfo("Delivery complete. Returning to IDLE state.")
                    self.state = RobotState.IDLE
                    self.state_start_time = time.time()

            self.rate.sleep()

if __name__ == '__main__':
    try:
        robot = DeliveryRobot()
        robot.fsm_loop()
    except rospy.ROSInterruptException:
        pass
