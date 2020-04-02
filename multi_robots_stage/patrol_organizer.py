import re
import rospy
import random
import numpy
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose2D, PoseStamped, Quaternion
from actionlib_msgs.msg import GoalStatusArray

class Coordinator(object):
    def __init__(self):

        rospy.loginfo('coordinator initialization')

        availableRobots=['robot_0', 
                         'robot_1',
                         'robot_2', 
                         'robot_3',
                         'robot_4', 
                         'robot_5']

        listOfPositions={'robot_0':[(0,-12), (0,12)],
                         'robot_1':[(-10.392,-6), (10.392,6)],
                         'robot_2':[(-10.392,6), (10.392,-6)],
                         'robot_3':[(0,12), (0,-12)],
                         'robot_4':[(10.392,6), (-10.392,-6)],
                         'robot_5':[(10.392,-6), (-10.392,6)]}

        for robot in listOfPositions.keys():
            pos = listOfPositions[robot]
            new_pos = []
            for entry in pos:
                e = list(entry)
                e[0] += 16.1
                e[1] += 16.1
                new_pos.append(tuple(e))
            listOfPositions[robot] = new_pos

        self.nextPositionIndex = {'robot_0':0, 
                                  'robot_1':0,
                                  'robot_2':0, 
                                  'robot_3':0,
                                  'robot_4':0, 
                                  'robot_5':0}

        self.availableRobots = availableRobots
        self.listOfPositions = {}
        self.status_subscriber={}
        self.robotPublishers = {}
        self.listOfPositions = listOfPositions

        for robotName in availableRobots:
            self.status_subscriber[robotName] = rospy.Subscriber(robotName+'/move_base/status', GoalStatusArray, self.askForNextLocation)
            topic = robotName+'/move_base_simple/goal'
            self.robotPublishers[robotName] = rospy.Publisher(topic, PoseStamped, queue_size=10, latch=True)
            
    def askForNextLocation(self, msg=GoalStatusArray()):

        if (len(msg.status_list) != 0 and msg.status_list[-1].status == 3):
            callerid = msg._connection_header['callerid']
            robotName = callerid[1:-15]
            self.sendDirection(robotName)
        
    
    def sendDirection(self, robot):
        pub = self.robotPublishers[robot]
        positionIndex = self.nextPositionIndex[robot]
        position = self.listOfPositions[robot][positionIndex]

        p = PoseStamped()
        p.header.frame_id = "map"
        p.header.stamp = rospy.Time.now()

        p.pose.position.x = position[0]
        p.pose.position.y = position[1]
        p.pose.position.z = 0

        q = quaternion_from_euler(0.0, 0.0, numpy.deg2rad(90.0))
        p.pose.orientation = Quaternion(*q)

        pub.publish(p)
        self.nextPositionIndex[robot] = (self.nextPositionIndex[robot] + 1) % len(self.listOfPositions[robot])

if __name__ == '__main__':
    coordinator = Coordinator()
    rospy.init_node('coordinator')
    coordinator.sendDirection('robot_0')
    coordinator.sendDirection('robot_1')
    coordinator.sendDirection('robot_2')
    coordinator.sendDirection('robot_3')
    coordinator.sendDirection('robot_4')
    coordinator.sendDirection('robot_5')
    rospy.spin()
