#!/usr/bin/env python
# -*- coding: utf-8 -*-

from cProfile import label
import os
import rospy
from nav_msgs.msg import Odometry
from sensor_message_getter import SensorMessageGetter
import matplotlib.pyplot as plt
import random

class OdomTrajectoriesMap(object):
    def __init__(self, topics, points_size, msg_wait=1.0):
        self.msgs = []
        self.topics = topics
        for topic in self.topics:
            self.msgs.append(SensorMessageGetter(topic, Odometry, msg_wait))
        self.points_size = points_size
        self.colorcodes = []
        self.legend_flag = 0
    
    def get_colorcode(self):
        return ["#"+''.join([random.choice('0123456789ABCDEF') for j in range(6)])]
    
    def spin(self):
        if len(self.colorcodes) == 0:
            for i in range(len(self.msgs)):
                self.colorcodes.append(self.get_colorcode())
        get_msgs = []
        for msg in self.msgs:
            get_msgs.append(msg.get_msg())
        if None in get_msgs:
            rospy.logwarn("get None msgs. I ignore in this time.")
        else:
            j = 0
            for msg in get_msgs:
                x = msg.pose.pose.position.x
                y = msg.pose.pose.position.y
                label_name = self.topics[j]
                plt.scatter(x, y, s=self.points_size, c=self.colorcodes[j], label=label_name)
                j += 1
            if not self.legend_flag:
                self.legend_flag = 1
                plt.legend()
            plt.pause(0.05)

def main():
    script_name = os.path.basename(__file__)
    node_name = os.path.splitext(script_name)[0]
    rospy.init_node(node_name)

    # param
    odom_topics = rospy.get_param("~odom_topics")
    points_size = rospy.get_param("~points_size")

    rospy.loginfo("odom topics : "+str(odom_topics))

    rate = rospy.Rate(5)
    node = OdomTrajectoriesMap(odom_topics, points_size)
    while not rospy.is_shutdown():
        node.spin()
        rate.sleep()

if __name__ == '__main__':
    main()