
'''
 * Team Id: 1326
 * Author List: Arun Malik, Shubham Sharma, Dhawal Babbar, Tushar Purang
 * Filename: main.c
 * Theme: Chaser Drone
 * Functions: buzzer_pin_config, buzzer_on, buzzer_off, lcd_port_config, adc_pin_config, motion_pin_config, port_init, timer1_init, adc_init, velocity,
    ADC_Conversion, print_sensor, motion_set, forward, stop, init_devices, in_cave, main
 * Global Variables: ADC_Conversion, ADC_Value, Left_white_line , Center_white_line, Right_white_line
'''



#!/usr/bin/env python
import roslib
import rospy
import time
import math
from ctypes import *
from plutodrone.srv import *
from plutodrone.msg import *
from std_msgs.msg import Char,Int16
from geometry_msgs.msg import PoseArray
from std_msgs.msg import String
import sys, select, termios, tty
import numpy as np 
import matplotlib.pyplot as plt
import thread
import multiprocessing
from whycon.srv import SetNumberOfTargets

#graphing variables
xaxis = [0] * 160000
thry  = [0] * 160000
erry  = [0] * 160000
intiy = [0] * 160000
diffy = [0] * 160000


thrx  = [0] * 160000
errx  = [0] * 160000
intix = [0] * 160000
diffx = [0] * 160000

thrz  = [0] * 160000
errz  = [0] * 160000
intiz = [0] * 160000
diffz = [0] * 160000

#global variables intialised
e_prevx  = 0
ui_prevx = 0
e_prevy  = 0
ui_prevy = 0
ui_prevz  = 0
e_prevz  = 0
currx  = 0
curry  = 0
currz  = 0
currax = 0
curray = 0
curraz = 0
currx1  = 0
curry1  = 0
currz1  = 0
count  = 4
singlet=True
t=False

'''
 * Function Name:pid_controllers for x, y ,z
 * Input: Drone current coordinates
 * Output: Drone MOVE values
 * Logic: Pid function
 
'''
#pid controllers for x, y, z

def pid_controllery(ya, yc, h=0.1, Ti=1.78, Td=61.11, Kp=8.5, u0=0, e0=0, ki=1):                                                   #PID CONTROLLER  kp 11
	

	
	global count
        global thry 
        global erry 
        global intiy 
        global diffy 
	global e_prevy
        global ui_prevy
	k = 0
        
        while 1:
                e = yc - ya
                ui = ui_prevy + 1/Ti * h*e
		ud = Td * (e - erry[count-4])/(4*h)
                e_prevy = e
		ui_prevy = ui
                u = (Kp *e) + ki*ui + ud
                 
                print "errorY",round(e,2)," integerator ",round(ki*ui,2)," deffren ",round(ud,2)
		k += 1
                
                erry[count]=e
                intiy[count]=ki*ui
                diffy[count]=ud
                return u

def pid_controllerx(ya, yc, h=0.1, Ti=1.78, Td=60.88, Kp=8, u0=0, e0=0, ki=1):                                                           #PID CONTROLLER kp 11
	
	
	global e_prevx
        global ui_prevx 
        global thrx 
        global errx 
        global intix 
        global diffx 
	k = 0
        
        while 1:
                e = yc - ya
                ui = ui_prevx + 1/Ti * h*e
		ud = Td * (e - errx[count-4])/(4*h)
                e_prevx = e
		ui_prevx = ui
                u = (Kp *e) + ki*ui + ud
                
		print "errorX",round(e,2)," integerator ",round(ki*ui,2)," deffren ",round(ud,2)
		k += 1
                errx[count]=e
                intix[count]=ki*ui
                diffx[count]=ud
                return u

def pid_controllerz(ya, yc, h=0.1, Ti=1.21, Td=29, Kp=55, u0=0, e0=0, ki=1):                                                          #PID CONTROLLER
	
        
	global thrz 
        global errz 
        global intiz 
        global diffz 
	global e_prevz
        global ui_prevz
	k = 0
        
        while 1:
                e = yc - ya
                ui = ui_prevz + 1/Ti * h*e
		ud = Td * (e - errz[count-6])/(6*h)
                e_prevz = e
		ui_prevz = ui
                u = (Kp*e) + ki*ui + ud
                
		print "errorZ",round(e,2)," integerator ",round(ki*ui,2)," deffren ",round(ud,2)
		k += 1
                errz[count]=e
                intiz[count]=ki*ui
                diffz[count]=ud
                return u 

'''
 * Function Name:lcheck
 * Input: currunt X,Y and target x, y
 * Output: Checks if drone satisfied +-error limit
 
 
'''
			
def lcheck(x,y,z,cons):
        global t
	counterx=0
	countery=0
	
        while 1:
    	        if abs(x-currx)<1:
			counterx=counterx+1
		if abs(y-curry)<1:
			countery=countery+1
		
		if counterx>cons and countery>cons:
	    	        t=True
                        break
	        rospy.sleep(0.1)
                print "lcheck CounterX:",counterx, "Y:",countery

def service_check(target,thresh_value):
  rospy.wait_for_service('whycon/reset')
  try:
    target_value = rospy.ServiceProxy('whycon/reset', SetNumberOfTargets)
    resp1 = target_value(target,thresh_value)
    return resp1.targets
  except rospy.ServiceException, e:
    print "Service call failed: %s"%e

'''
 * Function Name:Predicts Runner future decision point
 * Input: current Runner position
 * Output: Next decision point
 * Logic : Solved by Graph problem and direction to confirm coord
 
 
'''


def gobar():
	
	global drone_cord
	
	b=((9.8,-8.6),(-.5,-9.1),(10.9,-3.54),(6.83,-3.5),(3.5,-3.4),(3.5,-6.7),(-5.2,-6.7),(-4.04,-6.7),(-7.5,-6.5),(-11.4,-6.2),(-0.5,-3.3),(-7.4,-3.2),(7.0,-0.4),(-.3,-.2),(-3.8,-0.18),(-7.5,-0.11),(11.6,2.5),(6.8,2.6),(2.4,2.7),(-.2,2.7),(-2.26,2.79),(-7.0,2.76),(-11.01,2.8),(9.5,5.4),(6.9,6.2),(2.3,5.8),(-2.2,5.9),(10,7.8),(2.4,8.3),(-2.0,8.5),(-6.4,8.2),(-9.4,7.3))	
	
	a=((50,50,1,50),(0,6,50,50),(50,50,3,50),(2,12,4,50),(3,50,50,5),(50,4,6,50),(5,10,7,1),(6,50,8,50),(7,11,9,50),(8,50,50,50),(50,13,50,6),(50,15,50,8),(50,17,13,3),(12,19,14,10),(13,50,15,50),(14,50,50,11),(50,50,17,50),(16,24,18,12),(17,25,19,50),(18,50,20,13),(19,26,21,50),(20,50,22,50),(21,50,50,50),(50,50,24,50),(23,50,50,17),(50,28,26,18),(25,50,50,20),(50,50,28,50),(27,50,29,25),(28,50,30,50),(29,50,31,50),(30,50,50,50))
	
	lastindexd=(b[9][0]-currx1)*(b[9][0]-currx1)+(b[9][1]-curry1)*(b[9][1]-curry1)
	lastindex=9
	if (b[22][0]-currx1)*(b[22][0]-currx1)+(b[22][1]-curry1)*(b[22][1]-curry1)<lastindexd:
		lastindex=22
		lastindexd=(b[22][0]-currx1)*(b[22][0]-currx1)+(b[22][1]-curry1)*(b[22][1]-curry1)
	elif (b[31][0]-currx1)*(b[31][0]-currx1)+(b[31][1]-curry1)*(b[31][1]-curry1)<lastindexd:
		lastindex=31
	drone_cord=[0,0]	
	index=[0,0,0,0] 
	print lastindex
	while 1:
		
			if a[lastindex][0] != 50 :
				buf0=(b[a[lastindex][0]][0]-currx1)*(b[a[lastindex][0]][0]-currx1)+(b[a[lastindex][0]][1]-curry1)*(b[a[lastindex][0]][1]-curry1)
			else:
				buf0=0.00
				
			if a[lastindex][1] != 50 :
				buf1=(b[a[lastindex][1]][0]-currx1)*(b[a[lastindex][1]][0]-currx1)+(b[a[lastindex][1]][1]-curry1)*(b[a[lastindex][1]][1]-curry1)
			else:
				buf1=0.00
				
			if a[lastindex][2] != 50 :
				buf2=(b[a[lastindex][2]][0]-currx1)*(b[a[lastindex][2]][0]-currx1)+(b[a[lastindex][2]][1]-curry1)*(b[a[lastindex][2]][1]-curry1)
			else:
				buf2=0.00
				
			if a[lastindex][3] != 50 :
				buf3=(b[a[lastindex][3]][0]-currx1)*(b[a[lastindex][3]][0]-currx1)+(b[a[lastindex][3]][1]-curry1)*(b[a[lastindex][3]][1]-curry1)
			else:
				buf3=0.00
				
			rospy.sleep(1)
			dist=1000
			if buf0 !=0.00 and buf0-(b[a[lastindex][0]][0]-currx1)*(b[a[lastindex][0]][0]-currx1)+(b[a[lastindex][0]][1]-curry1)*(b[a[lastindex][0]][1]-curry1)<dist:
				drone_cord[0]=b[a[lastindex][0]][0]
				drone_cord[1]=b[a[lastindex][0]][1]
				 
				dist=buf0-(b[a[lastindex][0]][0]-currx1)*(b[a[lastindex][0]][0]-currx1)+(b[a[lastindex][0]][1]-curry1)*(b[a[lastindex][0]][1]-curry1)
				yolo=0
				print "00000000000000"
			if buf1 !=0.00 and buf1-(b[a[lastindex][1]][0]-currx1)*(b[a[lastindex][1]][0]-currx1)+(b[a[lastindex][1]][1]-curry1)*(b[a[lastindex][1]][1]-curry1)<dist:
				drone_cord[0]=b[a[lastindex][1]][0]
				drone_cord[1]=b[a[lastindex][1]][1]
				dist=buf1-(b[a[lastindex][1]][0]-currx1)*(b[a[lastindex][1]][0]-currx1)+(b[a[lastindex][1]][1]-curry1)*(b[a[lastindex][1]][1]-curry1)
				yolo=1
			        print "111111111111111"
			if buf2 !=0.00 and buf2-(b[a[lastindex][2]][0]-currx1)*(b[a[lastindex][2]][0]-currx1)+(b[a[lastindex][2]][1]-curry1)*(b[a[lastindex][2]][1]-curry1)<dist:
				drone_cord[0]=b[a[lastindex][2]][0]
				drone_cord[1]=b[a[lastindex][2]][1]
				dist=buf2-(b[a[lastindex][2]][0]-currx1)*(b[a[lastindex][2]][0]-currx1)+(b[a[lastindex][2]][1]-curry1)*(b[a[lastindex][2]][1]-curry1)
				yolo=2
				print "222222222222222"
			if buf3 !=0.00 and buf3-(b[a[lastindex][3]][0]-currx1)*(b[a[lastindex][3]][0]-currx1)+(b[a[lastindex][3]][1]-curry1)*(b[a[lastindex][3]][1]-curry1)<dist:
				drone_cord[0]=b[a[lastindex][3]][0]
				drone_cord[1]=b[a[lastindex][3]][1]
				dist=buf3-(b[a[lastindex][3]][0]-currx1)*(b[a[lastindex][3]][0]-currx1)+(b[a[lastindex][3]][1]-curry1)*(b[a[lastindex][3]][1]-curry1)	
				yolo=3
				print "333333333333333"
			#index[yolo]=index[yolo]+1	
			print "+++++++++++++++++++++++++++++++++++++++++",a[lastindex][yolo]
			if (drone_cord[0]-currx1)*(drone_cord[0]-currx1)+(drone_cord[1]-curry1)*(drone_cord[1]-curry1)<3:
				lastindex=a[lastindex][yolo]
				#index=[0,0,0,0]
			
				
				
# publisher for /drone_command
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
                        rospy.sleep(0.028)          
                         
'''
 * Function Name:Updates current position of drone and runner also Equalises Z-axis using curve 
 * Input: Nothing
 * Output: Currunt coordinates in currx, curry ::::Runner coordinates in currx1,curry1
 * Logic : Used curve fitting in 3 degree polynomial using Large amount of data
 
 
'''      
#callback function for whycon
def callback(data):
    global xaxis
    global currx
    global curry 
    global currz 
    global currax 
    global curray 
    global curraz   
    global currx1
    global curry1
    global currz1
    
    
    
     
    tempz = 15.47919754423-(0.04569349161*int((currx+20)*10))-(0.0310939534*int((curry+20)*10))+(0.0000794588956*int((currx+20)*10)*int((currx+20)*10))-(0.0000890845797*int((curry+20)*10)*int((curry+20)*10))+(0.000000125473*int((currx+20)*10)*int((currx+20)*10)*int((currx+20)*10))+(0.0000007430721*int((curry+20)*10)*int((curry+20)*10)*int((curry+20)*10))-(0.000108668831*int((curry+20)*10)*int((currx+20)*10))+(0.000000376315*int((currx+20)*10)*int((currx+20)*10)*int((curry+20)*10))-(0.00000020113742*int((currx+20)*10)*int((curry+20)*10)*int((curry+20)*10))+data.poses[0].position.z
    
    if singlet: 	
        tempz1 = 15.47919754423-(0.04569349161*int((currx1+20)*10))-(0.0310939534*int((curry1+20)*10))+(0.0000794588956*int((currx1+20)*10)*int((currx1+20)*10))-(0.0000890845797*int((curry1+20)*10)*int((curry1+20)*10))+(0.000000125473*int((currx1+20)*10)*int((currx1+20)*10)*int((currx1+20)*10))+(0.0000007430721*int((curry1+20)*10)*int((curry1+20)*10)*int((curry1+20)*10))-(0.000108668831*int((curry1+20)*10)*int((currx1+20)*10))+(0.000000376315*int((currx1+20)*10)*int((currx1+20)*10)*int((curry1+20)*10))-(0.00000020113742*int((currx1+20)*10)*int((curry1+20)*10)*int((curry1+20)*10))+data.poses[1].position.z
    

        
        if tempz1<tempz:
           currx = data.poses[1].position.x
           curry = data.poses[1].position.y
           currz = tempz1
           currx1 = data.poses[0].position.x
           curry1 = data.poses[0].position.y
           currz1 = tempz
        else:
           currx = data.poses[0].position.x
           curry = data.poses[0].position.y
           currx1 = data.poses[1].position.x
           curry1 = data.poses[1].position.y
           currz = tempz
           currz1 = tempz1 
    else: 
     	currx = data.poses[0].position.x
     	curry = data.poses[0].position.y
   	currz = tempz 
   	currz = data.poses[0].position.z
       
    currx = round(currx,2)                
    curry = round(curry,2)
    currz = round(currz,2)
    currx1 = round(currx1,2)                
    curry1 = round(curry1,2)
    currz1 = round(currz1,2)

'''
 * Function Name:mydrone 
 * Input: Task to perform in queue
 * Output: Commands to drone
 * Logic : Perform any function related to drone like Landing, Going to waypoint, Disarm 
 * Example Call :q.put(DRONE_DICTIONARY)
 
''' 

def mydrone(threadName, delay):
        print "Thread started"
        global count
        count =4  
        land=False
        start=False
        
        
        
        
        while(1): 
		 
                 count=count+1
                 if q.full():
                   droneD=q.get()
                   start=True
                   if droneD['e']:
		        land=False
		        print "Disarmed"   
		        drone.disarm()
		        print "                       "
		        print "Disarmed"
		        rospy.sleep(0.5)
		        drone.disarm()
		        droneD['e']=False
		        start=False
			plt.figure(1)
       			plt.plot(thry, color="black")
       			plt.plot(erry, color="red")
       			plt.plot(intiy, color="green")
			plt.plot(diffy, color="blue")
			plt.plot(xaxis, color="purple")
			plt.show()
		
			plt.figure(2)
			plt.plot(thrx, color="black")
			plt.plot(errx, color="red")
			plt.plot(intix, color="green")
			plt.plot(diffx, color="blue")
			plt.plot(xaxis, color="purple")
			plt.show()

			plt.figure(3)
			plt.plot(thrz, color="black")
			plt.plot(errz, color="red")
			plt.plot(intiz, color="green")
			plt.plot(diffz, color="blue") 
			plt.plot(xaxis, color="purple")
			plt.show() 

                   elif droneD['t']:
                       land=False
                       print "Drone in TAKING OFF & HOLD"
                       drone.disarm()
        	       rospy.sleep(1)
                       drone.disarm()
                       print "Arming"
                       drone.arm()
                       rospy.sleep(1)
                       count=9
                       droneD['t']=False
                       xcor=currx
                       ycor=curry
                       zcor=20
                       
                   elif droneD['g']:
                        land=False
                        print "Drone in GOTO state"
                        count=501
                        droneD['g']=False
                        xcor=droneD['xcod']
                        ycor=droneD['ycod']
                        zcor=droneD['zcod']
                   elif droneD['l']:
                          print "Drone going to LAND"
                          land=True 
                          droneD['l']=False
               
                 
                 
                 if start:
                    if count<15:
                       uy=pid_controllerx(currx,xcor)
                       ux=pid_controllery(curry,ycor)
                       uz=pid_controllerz(currz,zcor)
                       ay = 1500+(-ux)
                       ax = 1500+(uy)
                       az = 1500+(-uz)
                       print "Warming up Controller and rejecting false initial values"
                       
                       print "value X",int(ax),"coordinates",currx
                       print "value Y",int(ay),"coordinates",curry
                       print "value Z",int(az),"coordinates",currz
                       print "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX"
                       
                       
                    
                    else:
                            if land:
                               zcor=zcor+0.5
 
                             
                            print "-----------------------------------------------------------------------------"
                            
                               
                            print "LETS GO TO X ",xcor,"Y ",ycor,"Z ",zcor  
                            uy=pid_controllerx(currx,xcor)
                            ux=pid_controllery(curry,ycor)
                            uz=pid_controllerz(currz,zcor) 
                            if (count>500) and (count <506):
                                
                                uy=uy-diffx[count]
                                ux=ux-diffy[count]
                                print "----------------------I M ACTIVE-------------------NEGATING:",diffx[count]," ",uy
                    
                            ay = (1500+((-ux)))
                            ax = (1500+((uy)))
                            az = 1500+(-uz)
                            
                            if az>2000:
                                az=2000
                            print "value X",int(ax),"coordinates",currx
                            print "value Y",int(ay),"coordinates",curry
                            print "value Z",int(az),"coordinates",currz
                            
                            drone.move(int(ax),int(ay),int(az))          
                            thrx[count]=uy
                            thry[count]=-ux
                            thrz[count]=-uz
                            print count
                                       
                                        
                 
                            
        



if __name__=="__main__": 
        settings = termios.tcgetattr(sys.stdin)
    	
	
        drone = send_data(); 
        cord=rospy.Subscriber('/whycon/poses', PoseArray, callback)                 #Passing Topic name and msg type
	
        q=multiprocessing.Queue(maxsize=1)       					
        mainD={'t':False,'g':False,'h':False,'l':False,'e':False,'xcod':0.00,'ycod':0.00,'zcod':25.00} #dICTIONARY to send to drone api
        thread.start_new_thread( mydrone,("Thread-1", 2, ))
	        
 	coorar=[-0.24,0.17,20,6.01,-5.01,22,5.61,4.27,22] #HOME LN1 LN5
        event =0
	drone_cord=[0,0]
	
        if event==0:
        	mainD['t']=True										
        	q.put(mainD,True,0.1)       #sending Takeoff command to drone
        	rospy.sleep(5)
            	event=1
            	mainD['t']=False

        if event==1:
        	mainD['g']=True
                mainD['xcod']=coorar[0]
        	mainD['ycod']=coorar[1]
        	mainD['zcod']=coorar[2]
        	q.put(mainD,True,0.1)
                lcheck(coorar[0],coorar[1],coorar[2],20)
        	if t:
			t=False
			event=2
        		mainD['g']=False

        if event==2:
        	mainD['g']=True
                mainD['xcod']=coorar[3]
        	mainD['ycod']=coorar[4]
        	mainD['zcod']=coorar[5]
        	q.put(mainD,True,0.1)
		lcheck(coorar[3],coorar[4],coorar[5],10)
        	if t:
			t=False
        		event=3
        		mainD['g']=False
 			
 	if event==3:
        	  mainD['g']=True
                  mainD['xcod']=coorar[6]
        	  mainD['ycod']=coorar[7]
        	  mainD['zcod']=coorar[8]
        	  q.put(mainD,True,0.1)
		  lcheck(coorar[6],coorar[7],coorar[8],10)
        	  if t:
			t=False
        		event=4
        		mainD['g']=False

        if event==4:
        	mainD['g']=True
                mainD['xcod']=coorar[0]
        	mainD['ycod']=coorar[1]
        	mainD['zcod']=coorar[2]
        	q.put(mainD,True,0.1)
		lcheck(coorar[0],coorar[1],coorar[2],10)
        	if t:
			t=False
        		event=5
        		mainD['g']=False
	drone_cord[0]=currx1
	drone_cord[1]=curry1
        while(1):
         if event==5:
	   
                mainD['g']=True
		
                mainD['xcod']=drone_cord[0]		#Go to predicted waypoint X
        	mainD['ycod']=drone_cord[1]		#Go to predicted waypoint Y
        	mainD['zcod']=21
                q.put(mainD,True,0.1)
 		#lcheck(drone_cord[0],drone_cord[1],0,10)
		
 		if t:
                     if abs(drone_cord[0]-currx1)<1 and  abs(drone_cord[1]-curry1)<1: 
			
			t=False
			event=6
			mainD['g']=False		
			
		     else:		
			continue			#wrong decesion point
 			t=False
			print "Going to try again"
                  
         if event==6:
        	mainD['l']=True
		service_check(1,0)
        	q.put(mainD,True,0.1)
		prevcurrx1=currx1
		prevcurry1=curry1
        	rospy.sleep(3.5)
	        event=7
	        mainD['l']=False

         if event==7:
        	mainD['e']=True
        	q.put(mainD,True,0.1)
		if  abs(currx1-prevcurrx1)<1 and abs(curry1-prevcurry1)<1:  #checking if Runner has moved since drone landed
		    print "Drone landed"				#Succesfull land
		    mainD['e']=False
		    rospy.sleep(20.5)
        	    exit()
		else:							#LAnding failed
		    mainD['t']=True										
		    mainD['e']=False
        	    q.put(mainD,True,0.1)
        	    rospy.sleep(5)
            	    event=5
            	    mainD['t']=False
		    print "Going to try again after that much effort"
        	    continue
		    


        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        
























