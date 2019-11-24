#!/usr/bin/env python
#
# title           :hover.py
# description     :
# author          :Carlos Hansen
# date            :
# pythonVersion   :2.7.15
# ==============================================================================
import argparse             # for manipulaton of parameters
import csv                  # for saving data into a csv file
import numpy as np          # for manipulation of data

# ROS modules
import rospy
import geometry_msgs.msg
import nav_msgs.msg
import std_msgs.msg
# import sensor_msgs.msg


# Develped modules
from PID import PID         # PID controller
import support              # Useful Operations


class MyDrone:
    def __init__(self):

        #----- Modifiable Parameters
        # PID controller
        # Proportional parameters XYZrpy
        kp = np.array([0.15, 0.15, 0.2, 0, 0, 0.15])
        # Integral parameters XYZrpy
        ki = np.array([0.01, 0.01, 0.01, 0, 0, 0.01])
        # Proportional parameters XYZrpy
        kd = np.array([0.25, 0.25, 0.01, 0, 0, 0.25])

        samplingTime = 0.1  # 100 [ms]

        self.hoveringPoint = np.array([0, 0, 2, 0, 0, np.deg2rad(0)])

        self.freqTopic = 10  # Frequency of topic messages, 10HZ

        self.stableTime = 5  # Time in the vecinity of target waypoint 5[s]
        self.inReach = 0.1    # Distance to be considered in the vecinity of the waypoint

        self.fileName = "hw04_task01_pid.csv"    # Name of the file to save the data

        self.simulationNodeSubscriber = "/bebop/odom"   # For Simulation
        self.regularNodeSubscriber = "/vicon/bebop/bebop"   # For Regular mode

        # ----- General Initialization stage

        np.set_printoptions(precision=3) # to print numpy
        
        # Simulation or regular mode
        self.simulationMode = args.simulation and not args.p_node

        if args.p_node: self.regularNodeSubscriber = args.p_node    # to node passed as parameter

        # Controller
        self.controller = PID()  
        self.controller.setKp(kp)
        self.controller.setKi(ki)
        self.controller.setKd(kd)
        self.controller.setSamplingTime(samplingTime)
        self.controller.setSetPoint(self.hoveringPoint)

        # CSV
        with open(self.fileName, mode='w') as data_file:
                    file_writter = csv.writer(
                        data_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                    file_writter.writerow([0, 0, 0, 0, 0, 0])

        # ----- ROS initialization stage
        # Node
        rospy.init_node('drone_controller', anonymous=True)
        self.rate = rospy.Rate(self.freqTopic)

        # Control Drone Publisher
        # To control the position
        self.cmdMsg = geometry_msgs.msg.Twist()
        # Publisher to command the robot via the topic /bebop/cmd_vel
        self.cmdPub = rospy.Publisher(
            '/bebop/cmd_vel', geometry_msgs.msg.Twist, queue_size=10)

        # ----- Drone Operation stage
        # self.takeoff(3)

        # Control during simulation mode
        if self.simulationMode:
            print ("Simulation mode")

            print ("Subscribing to {node}".format(node=self.simulationNodeSubscriber))
            while not rospy.is_shutdown():
                # Subscriber to check if the corner had been reach and change the goal corner to go
                self.poseForPathSubscriber = rospy.Subscriber(
                    self.simulationNodeSubscriber,
                    nav_msgs.msg.Odometry,
                    self.droneController)
                self.rate.sleep()

        # Control during regular mode
        else:
            print ("Regular mode")
            print ("Subscribing to {node}".format(node=self.regularNodeSubscriber))
            while not rospy.is_shutdown():
                # Subscriber to check if the corner had been reach and change the goal corner to go
                self.poseForPathSubscriber = rospy.Subscriber(
                    self.regularNodeSubscriber,
                    geometry_msgs.msg.TransformStamped,
                    self.droneController)
                self.rate.sleep()

    
    
    # ###### Drone operation function and callbacks ######

    def takeoff(self, takeoffTime):
        """ Publication of the message to takeoff during the specified time
        
        @param takeoffTime: Time during which the topic will be publish
        @type time: Time_obj 
        """
        print("Takingoff")

        takeoffPub = rospy.Publisher(
            "bebop/takeoff", std_msgs.msg.Empty, queue_size=1)

        beginTime = rospy.Time.now()  # Starting time
        #note to work from rospy.Time a python time its a build idnfunc rospy.Time.to_time(beginTime)
        # Desired duration in terms of ros
        secondsTakeoff = rospy.Duration(takeoffTime)
        # Desired duration in reference of begininig
        endTime = secondsTakeoff + beginTime
        while rospy.Time.now() < endTime:  # publication of message while duration
            takeoffPub.publish(std_msgs.msg.Empty())
            self.rate.sleep()

    def land(self, landTime):
        """ Publication of the message to land during the specified time
        
        @param takeoffTime: Time during which the topic will be publish
        @type time: Time_obj 
        """
        print("Landing")

        landPub = rospy.Publisher(
            "bebop/land", std_msgs.msg.Empty, queue_size=1)

        beginTime = rospy.Time.now()  # Starting time

        # Desired duration in terms of ros
        secondsTakeoff = rospy.Duration(landTime)
        # Desired duration in reference of begininig
        endTime = secondsTakeoff + beginTime
        while rospy.Time.now() < endTime:  # publication of message while duration
            landPub.publish(std_msgs.msg.Empty())
            self.rate.sleep()

    def droneController(self, message):
        """"Control the drone"""

        if self.simulationMode:
            dronePosInertial = support.odometryMsg2InertialCoordinates(message)
        else:
            dronePosInertial = support.transformedStampedMsg2InertialCoordinates(message)
        # print(dronePosInertial)

        # limit if there is outside of the safe area
        if (dronePosInertial[0] < -4 or dronePosInertial[0] > 4 or dronePosInertial[1] < -4 or dronePosInertial[1] > 4 or dronePosInertial[2] > 3):
            print("outside of safe area")
            self.land(3)    # Land the drone

        # inside of the safe area
        else:
            print ("Inside of safe area")

            ctrlOutput = self.controller.controll(dronePosInertial, rospy.Time.now().to_time())

            # Create command
            if ctrlOutput is not None :
                # print("command: {0}".format(ctrlOutput))
                
                self.cmdMsg.linear.x = ctrlOutput[0] if ctrlOutput[0] < 1  else 1
                self.cmdMsg.linear.y = ctrlOutput[1] if ctrlOutput[1] < 1  else 1
                self.cmdMsg.linear.z = ctrlOutput[2] if ctrlOutput[2] < 1  else 1
                # self.cmdMsg.angular.z = ctrlOutput[5]

                print (self.cmdMsg)

                # Publish
                self.cmdPub.publish(self.cmdMsg)

            # Save the current odometry
            with open(self.fileName, mode='a') as data_file:
                file_writter = csv.writer(data_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                file_writter.writerow(dronePosInertial)        


########################################
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-s","--simulation", action="store_true", help="Enter simulation mode")
    parser.add_argument("-p","--p_node", type=str, help="Node to subscribe for the position of the frame position of the drone. If set regular operation mode will be selected")
    args = parser.parse_args()
    try:
        hover = MyDrone()
        rospy.spin()

    except rospy.ROSInterruptException:
        print("error!")
