#!/usr/bin/env python
import roslib
import rospy
import time
from ctypes import *
from plutodrone.srv import *
from plutodrone.msg import *
from std_msgs.msg import Int16
from geometry_msgs.msg import PoseArray
from std_msgs.msg import String
import sys, select, termios, tty


#each button is associated with a 6-dim tuple: (x,th_x,y,th_y,z,th_z)

#(linear,angular) velocity on the three axis
e_prevx = 0
ui_prevx = 0
e_prevy = 0
ui_prevy = 0
e_prevz = 0
ui_prevz = 0
currx = 0
curry = 0
currz = 0
currax = 0
curray = 0
curraz = 0
prx    = 0
pry    = 0
prz    = 0
def pid_controllerx(ya, yc, h=0.1, Ti=2.99, Td=1.6, Kp=10, u0=0, e0=0):                                                   #PID CONTROLLER
	"""Calculate System Input using a PID Controller

	Arguments:
	y  .. Measured Output of the System
	yc .. Desired Output of the System
	h  .. Sampling Time
	Kp .. Controller Gain Constant
	Ti .. Controller Integration Constant
	Td .. Controller Derivation Constant
	u0 .. Initial state of the integrator
	e0 .. Initial error

	Make sure this function gets called every h seconds!
	"""
	
	global e_prevx
        global ui_prevx
	# Step variable
	k = 0



	while 1:

		# Error between the desired and actual output
		e = yc - ya

		# Integration Input
		ui = ui_prevx + 1/Ti * h*e
		# Derivation Input
		ud = 1/Td * (e - e_prevx)/h
                
		# Adjust previous values
		e_prevx = e
		ui_prevx = ui

		# Calculate input for the system
		u = Kp * (e + ui + ud)
		print "YY error ",round(e,2)," integerator ",round(ui,2)," deffren ",round(ud,2) 
		k += 1

                return u

def pid_controllery(ya, yc, h=0.1, Ti=2.99, Td=1.6, Kp=10, u0=0, e0=0):                                                           #PID CONTROLLER
	
	
	global e_prevy
        global ui_prevy 
         
	# Step variable
	k = 0



	while 1:

		# Error between the desired and actual output
		e = yc - ya

		# Integration Input
		ui = ui_prevy + 1/Ti * h*e
		# Derivation Input
		ud = 1/Td * (e - e_prevy)/h

		# Adjust previous values
		e_prevy = e
		ui_prevy = ui

		# Calculate input for the system
		u = Kp * (e + ui + ud)
		print "XX error ",round(e,2)," integerator ",round(ui,2)," deffren ",round(ud,2)
		k += 1

                return u
def pid_controllerz(ya, yc, h=0.1, Ti=2.99, Td=1.6, Kp=10, u0=0, e0=0):                                                          #PID CONTROLLER
	
	
	global e_prevz
        global ui_prevz
	# Step variable
	k = 0
        
         
        
	while 1:

		# Error between the desired and actual output
		e = yc - ya

		# Integration Input
		ui = ui_prevz + 1/Ti * h*e
		# Derivation Input
		ud = 1/Td * (e - e_prevz)/h

		# Adjust previous values
		e_prevz = e
		ui_prevz = ui

		# Calculate input for the system
		u = Kp * (e + ui + ud)
		print "ZZ error ",round(e,2)," integerator ",round(ui,2)," deffren ",round(ud,2)
		k += 1

                return u

class send_data():
	"""docstring for request_data"""
	def __init__(self):
		rospy.init_node('drone_server')
		self.command_pub = rospy.Publisher('/drone_command', PlutoMsg, queue_size=1)

		self.currax = PlutoMsg().rcRoll
                self.curray = PlutoMsg().rcPitch
                self.curraz = PlutoMsg().rcYaw 
		self.cmd = PlutoMsg()
		self.cmd.rcRoll     =1500
		self.cmd.rcPitch    =1500
		self.cmd.rcYaw      =1500
		self.cmd.rcThrottle =1500
		self.cmd.rcAUX1     =1500
		self.cmd.rcAUX2     =1500
		self.cmd.rcAUX3     =1500
		self.cmd.rcAUX4     =1000
		
	def arm(self):
		self.cmd.rcRoll     =1500
		self.cmd.rcYaw      =1500
		self.cmd.rcPitch    =1500
		self.cmd.rcThrottle =1000
                self.cmd.rcAUX1     =1500
		self.cmd.rcAUX2     =1500
		self.cmd.rcAUX3     =1500
		self.cmd.rcAUX4     =1500
		self.command_pub.publish(self.cmd)
		rospy.sleep(.1)
	def disarm(self):
                self.cmd.rcRoll     =1500
		self.cmd.rcYaw      =1500
		self.cmd.rcPitch    =1500
                self.cmd.rcAUX1     =1500
		self.cmd.rcAUX2     =1500
		self.cmd.rcAUX3     =1500
                self.cmd.rcThrottle =1000
		self.cmd.rcAUX4     =1200
		self.command_pub.publish(self.cmd)
		rospy.sleep(0.5)
	
	
	def reset(self):
		self.cmd.rcRoll     =1500
		self.cmd.rcThrottle =1500
		self.cmd.rcPitch    =1500
		self.cmd.rcYaw      =1500
		self.command_pub.publish(self.cmd)
	

	def move(self,currax,curray,curraz):
		        self.cmd.rcRoll     =currax
			self.cmd.rcPitch    =curray
		        self.cmd.rcThrottle =curraz
                        self.cmd.rcYaw      =1500
                        self.cmd.rcAUX1     =1500
		        self.cmd.rcAUX2     =1500
		        self.cmd.rcAUX3     =1500
		        self.cmd.rcAUX4     =1500
			self.command_pub.publish(self.cmd)             
                        rospy.sleep(0.1)          
                         
       

def callback(data):
    global currx
    global curry 
    global currz 
    global currax 
    global curray 
    global curraz   
    global prx
    global pry
    global prz
    currx = data.poses[0].position.x
    curry = data.poses[0].position.y
    currz = data.poses[0].position.z
    currx = round(currx,2)
    curry = round(currx,2)
    curry = round(currx,2)
    count =0
if __name__=="__main__":
        rospy.sleep(2)
        drone = send_data();
        i=0 
        while(i <5):
             drone.disarm();
             i=i+1;
             print "Press ctrl+z to terminate" 
        while(i <10):
             drone.arm();
             i=i+1;
        xcons = 1
        ycons = 1
        rospy.sleep(1)
        zcor=23
        count=0
        
        while (1):
          cord=rospy.Subscriber('/whycon/poses', PoseArray, callback)                 #Passing Topic name and msg type
          
          if count<4 :
             prx=currx
             pry=curry
             prz=currz
             print "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX"
             
             xcor=prx
             ycor=pry
             print "LETS GO TO X=",xcor," Y=",ycor
             rospy.sleep(0.1)
          else:

            
            if count<6:
             uy=pid_controllery(currx,xcor)
             ux=pid_controllerx(curry,ycor)
             uz=pid_controllerz(currz,zcor)
             ay = 1500+(-ux)
             ax = 1500+(uy)
             az = 1500+(-uz)
             print "value X",int(ax),"coordinates",int(currx)
             print "value Y",int(ay),"coordinates",int(curry)
             print "value Z",int(az),"coordinates",int(currz)  
            else:
             ulngx=int(prx-currx)
             ulngy=int(pry-curry)
             ulngz=int(prz-currz)

             print "-----------------------------------------------------------------------------"
             if (ulngx+ulngy+ulngz > 10 or ulngx+ulngy+ulngz<-10): 
                print "ERROR Ye shubham ya Babbar gandu bich mein aa gye"
                drone.move(1500,1500,1500)
           
             else:
                prx=currx
                pry=curry
                prz=currz
             
             #if (uz>20 & u<26):
                 
                print "LETS GO TO X ",xcor,"Y ",ycor
                uy=pid_controllery(currx,xcor)
                ux=pid_controllerx(curry,ycor)
                uz=pid_controllerz(currz,zcor) 
             
                ay = 1500+(-ux)
                ax = 1500+(uy)
                az = 1500+(-uz)
          
                print "value X",int(ax),"coordinates",currx
                print "value Y",int(ay),"coordinates",curry
                print "value Z",int(az),"coordinates",currz
           
                drone.move(int(ax),int(ay),int(az))                                   
            
          count=count+1
			       

			

	
