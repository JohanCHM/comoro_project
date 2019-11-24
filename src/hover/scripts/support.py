#!/usr/bin/env python
#
# title           :support.py
# description     :
# author          :Carlos Hansen
# date            :
# pythonVersion   :2.7.15
# ==============================================================================
import numpy as np          # for manipulation of data
import tf                   # transformation between euler and quaternions


def odometryMsg2InertialCoordinates(message):
        """Convert an Odometry message to Inertial Coordinates (x, y, z, phi, theta, psi)
        
        @param message: messge to convert
        @type message: nav_msgs.Odometry
        @returns: Inertial coordinates
        @rtype: numpy.ndarray(1_6) 
        """
        # Get the pose from the message
        pos = message.pose.pose
        quat = pos.orientation

        # Transform quaternion coordinates to Euler
        angles = tf.transformations.euler_from_quaternion((quat.x, quat.y,
                                                           quat.z, quat.w))

        phi = angles[0]
        theta = angles[1]
        psi = angles[2]

        # X, Y, Z
        dronePos = np.array([pos.position.x, pos.position.y, pos.position.z])

        # Inertia into body coordinates transformation matrix
        cp = np.cos(phi)
        sp = np.sin(phi)
        ct = np.cos(theta)
        st = np.sin(theta)
        cs = np.cos(psi)
        ss = np.sin(psi)

        transMat = np.array([[ct*cs,          ct*ss,          -st],
                             [sp*st*cs-cp*ss, sp*st*ss+cp*cs, sp*ct],
                             [cp*st*cs+sp*ss, cp*st*ss-sp*cs, cp*ct]])

        dronePosInertial = transMat.dot(np.transpose(dronePos))     # Drone coordinates in body coordinates

        dronePosInertialWAng = np.concatenate([dronePosInertial,np.array([phi,theta,psi])])

        return dronePosInertialWAng

def transformedStampedMsg2InertialCoordinates(message):
        """Convert a Transformed Stamped message to Inertial Coordinates (x, y, z, phi, theta, psi)
        
        @param message: message to convert
        @type message: geometry_msgs.TransformStamped 
        @returns: Inertial coordinates
        @rtype: numpy.ndarray(1_6) 
        """
        # Get the pose from the message
        pos = message.transform
        quat = pos.rotation

        # Transform quaternion coordinates to Euler
        angles = tf.transformations.euler_from_quaternion((quat.x, quat.y,
                                                           quat.z, quat.w))

        phi = angles[0]
        theta = angles[1]
        psi = angles[2]

        # X, Y, Z, phi, theta, psi
        dronePos = np.array([pos.translation.x, pos.translation.y, pos.translation.z])

         # Inertia into body coordinates transformation matrix
        cp = np.cos(phi)
        sp = np.sin(phi)
        ct = np.cos(theta)
        st = np.sin(theta)
        cs = np.cos(psi)
        ss = np.sin(psi)

        transMat = np.array([[ct*cs,          ct*ss,          -st],
                             [sp*st*cs-cp*ss, sp*st*ss+cp*cs, sp*ct],
                             [cp*st*cs+sp*ss, cp*st*ss-sp*cs, cp*ct]])

        dronePosInertial = transMat.dot(np.transpose(dronePos))     # Drone coordinates in body coordinates

        dronePosInertialWAng = np.concatenate([dronePosInertial,np.array([phi,theta,psi])])

        return dronePosInertialWAng
