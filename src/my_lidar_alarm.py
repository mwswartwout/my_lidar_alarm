#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Bool


class LidarAlarm():
    # set alarm if anything is within 0.5m of the front of robot
    MIN_SAFE_DISTANCE = 0.5

    # these values to be set within the laser callback
    ping_dist_in_front_ = 3  # global var to hold length of a SINGLE LIDAR ping--in front
    ping_index_ = -1  # NOT real; callback will have to find this
    angle_min_ = 0.0
    angle_max_ = 0.0
    angle_increment_ = 0.0
    range_min_ = 0.0
    range_max_ = 0.0
    laser_alarm_ = False

    lidar_alarm_publisher_ = rospy.Publisher('lidar_alarm', Bool, queue_size=1)
    lidar_dist_publisher_ = rospy.Publisher('lidar_dist', Float32, queue_size=1)


    def laserCallback(self, laser_scan):
        if (self.ping_index_ < 0):
        # for first message received, set up the desired index of LIDAR range to
        # eval
            self.angle_min_ = laser_scan.angle_min
            self.angle_max_ = laser_scan.angle_max
            self.angle_increment_ = laser_scan.angle_increment
            self.range_min_ = laser_scan.range_min
            self.range_max_ = laser_scan.range_max
        # what is the index of the ping that is straight ahead?
        # BETTER would be to use transforms, which would reference how the LIDAR is mounted;
        # but this will do for simple illustration
            rospy.loginfo(self.angle_min_)
            rospy.loginfo(self.angle_increment_)
            self.ping_index_ = int((0 - self.angle_min_) / self.angle_increment_)
            rospy.loginfo("LIDAR setup: ping_index = %d", self.ping_index_)

        self.ping_dist_in_front_ = laser_scan.ranges[self.ping_index_]
        rospy.loginfo("ping dist in front = %f", self.ping_dist_in_front_)
        if (self.ping_dist_in_front_ < self.MIN_SAFE_DISTANCE):
            self.laser_alarm_ = True
        else:
            self.laser_alarm_ = False

        lidar_alarm_msg = Bool()
        lidar_alarm_msg.data = self.laser_alarm_
        self.lidar_alarm_publisher_.publish(lidar_alarm_msg)

        lidar_dist_msg = Float32()
        lidar_dist_msg.data = self.ping_dist_in_front_
        self.lidar_dist_publisher_.publish(lidar_dist_msg)

def main():
    rospy.init_node('my_lidar_alarm')
    alarm = LidarAlarm()

    # create a Subscriber object and have it subscribe to the lidar topic
    lidar_subscriber = rospy.Subscriber('robot0/laser_0', LaserScan, alarm.laserCallback)
    rospy.spin()
    # this is essentially a "while(1)" statement, except it
    # forces refreshing wakeups upon new data arrival
    # main program essentially hangs here, but it must stay alive to keep the
    # callback function alive

if __name__ == '__main__':
    main()
