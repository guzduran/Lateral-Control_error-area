#!/usr/bin/env python
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Int16

from lane_detection.msg import Num

class ForceController:
    def __init__(self):
        
	
        self.sub_arr = rospy.Subscriber('/lane_detection/topic', Num, self.callArr, queue_size=1)

	self.pub = rospy.Publisher("/AutoNOMOS_mini/manual_control/steering", Int16, queue_size=1)
        self.pub_speed = rospy.Publisher("/AutoNOMOS_mini/manual_control/speed", Int16, queue_size=100, latch=True)
        self.sub_odom = rospy.Subscriber("/AutoNOMOS_mini/real_pose_from_gazebo", Pose2D, self.callback, queue_size=1) 

    def callArr (self,data):
	print("recibo", str(data.Num))	
	
 
    def callback(self, data):
	#print(data)
	waypoints = [[0, 0], [0, 8], [5, 8], [5, 5], [3, 3], [5 ,0], [8, 2], [12, 0]]
	####posicion robot###
	h = 0.20
	L = 0.4        
        theta = data.theta
	X_h = data.x+h*np.cos(theta)
        Y_h = data.y+h*np.sin(theta)	
	print(theta)


        #X_dh = waypoints[0]
    	#Y_dh = waypoints[1]
	k1 = 3
	Ux = -k1*(X_h-0.73)	
	Uy = -k1*(Y_h-5)
        V = np.cos(theta)*Ux + np.sin(theta)*Uy
	#V = 150
        w = -np.sin(theta)*(Ux/h) + np.cos(theta)*(Uy/h)
        #print(V)
        steering=np.arctan((w*L)/(V))
	
	r = 0.03
	V_w = V/r
	Wll = (V_w*60)/(2*np.pi)
        if (V>0):
            speed = -Wll*0
        else:
            speed = Wll*0
	

        if (steering>(np.pi)/2):
            steering = (np.pi)/2

        if (steering<-(np.pi)/2):
            steering = -(np.pi)/2
	print("V_w",V_w)
	

        steering = 90+ steering * (180/np.pi)
        self.pub.publish(Int16(steering))

        self.pub_speed.publish(Int16(speed))
	print("Steering",steering)


def main():
    rospy.init_node('ForceController')
    ForceController()  # constructor creates publishers / subscribers
    #detector.laneDetector()
    rospy.spin()

if __name__ == '__main__':
    main()
    
