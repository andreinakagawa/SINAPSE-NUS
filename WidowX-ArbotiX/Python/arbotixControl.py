#---------------------------------------------------
# NATIONAL UNIVERSITY OF SINGAPORE - NUS
# SINGAPORE INSTITUTE FOR NEUROTECHNOLOGY - SINAPSE
# Singapore
#---------------------------------------------------
# Author: Andrei Nakagawa, MSc
# Contact: nakagawa.andrei@gmail.com
#---------------------------------------------------
# Description: This script provides a basic serial
# communication with the WidowX-ArbotiX platform so
# it can be tested for general purposes. 
# Ther user is asked to enter with the servo ID, an
# initial position (P0) and how many increments the
# servo should move from P0.
#---------------------------------------------------
#LIBRARIES
from serial import Serial #serial communication
import time #delay
from threading import Timer #timeout
#---------------------------------------------------
#CONSTS
AX_SETPOSITION = 0
AX_GETPOSITION = 1
AX_LED_ON = 4
AX_LED_OFF = 5
PKG_HEADER = 0x24
PKG_END = 0x21
#---------------------------------------------------
#FUNCTIONS
def waitBytes(_serialHandler,_numBytes):
		try:				
			numWaiting = 0
			while True:
				if(_serialHandler.is_open):
					numWaiting = _serialHandler.in_waiting					
					if numWaiting < _numBytes:						
						pass
					else:
						return True
				else:					
					return False								
		except:			
			return False
#---------------------------------------------------
def waitSTByte(_serialHandler,_startByte):	
	receivedByte = 0
	while True:
		ret = waitBytes(_serialHandler,1)			
		if ret:			
			receivedByte = ord(_serialHandler.read())						
			if receivedByte == _startByte:				
				return True
			else:
				return False
		else:
			return False
#---------------------------------------------------
def readPackage(_serialHandler):
		#print 'readPackage called'
		try:			
			ret = waitSTByte(serialRobot,PKG_HEADER)
			if ret:
				ret = waitBytes(serialRobot,3)
				if ret:					
					data = _serialHandler.read(3)					
					data = map(ord,data)				

				ret = waitBytes(serialRobot,1)								
				if ret:					
					endByte = ord(_serialHandler.read())					
					if endByte == PKG_END:						
						return data
					else:
						print('package error!')
						return False
		except:			
			print 'read error!'
			return False
#---------------------------------------------------
#---------------------------------------------------
#MAIN
#---------------------------------------------------
#serial communication parameters
serialRobot = Serial() #new object
serialRobot.port = '/dev/ttyUSB0' #Port number or ID
serialRobot.baud = 9600 #baud rate
#---------------------------------------------------
#opens the serial port
serialRobot.open() 
#---------------------------------------------------
#decides which action should be sent to the robot
action = input('0: set position, 1: get position, 4: LED on, 5: LED off | ')
#----------------------------------------------------------------------------------
#ACTION 1: Set position of a servo
if(action == AX_SETPOSITION):
	#inputs
	servoId = input('which servo: ') #servo ID
	initialPosition = input('insert an initial position: ') #initial position (P0)
	steps = input('insert how many steps: ') #how many steps from P0 should be taken
	#---------------------------------------------------
	#loop for sending the desired positions to ArbotiX from P0 with the desired
	#number of steps
	for k in range(initialPosition,initialPosition+steps):		
		print("desired position: " + str(k)) #prints the desired position on screen
		serialRobot.write(chr(PKG_HEADER)) #HEADER OF PACKAGE
		serialRobot.write(chr(AX_SETPOSITION)) #which action: 0 = set position
		serialRobot.write(chr(servoId)) #which servo
		serialRobot.write(chr(k>>8)) #Position MSB
		serialRobot.write(chr(k&0xFF)) #Position LSB
		serialRobot.write(chr(PKG_END)) #END OF PACKAGE
		time.sleep(0.1) #delay so communication and movement can take place
#----------------------------------------------------------------------------------
#ACTION 2: Get position of a servo
elif(action == AX_GETPOSITION):	
	servoId = input('which servo: ') #servo ID
	serialRobot.write(chr(PKG_HEADER)) #HEADER OF PACKAGE
	serialRobot.write(chr(AX_GETPOSITION))
	serialRobot.write(chr(servoId))
	serialRobot.write(chr(0))
	serialRobot.write(chr(0))
	serialRobot.write(chr(PKG_END)) #END OF PACKAGE
	time.sleep(0.1)
	ret = readPackage(serialRobot)
	if ret is not False:
		servoPos = (ret[1]<<8) + ret[2]
	print('servo Id: ' + str(ret[0]))
	print('servo position: ' + str(servoPos))
#----------------------------------------------------------------------------------
#ACTION 4: Turn LED on
elif(action == AX_LED_ON):	
	serialRobot.write(chr(PKG_HEADER)) #HEADER OF PACKAGE
	serialRobot.write(chr(AX_LED_ON))
	serialRobot.write(chr(0))
	serialRobot.write(chr(0))
	serialRobot.write(chr(0))
	serialRobot.write(chr(PKG_END)) #END OF PACKAGE
#----------------------------------------------------------------------------------
#ACTION 5: Turn LED off
elif(action == AX_LED_OFF):	
	serialRobot.write(chr(PKG_HEADER)) #HEADER OF PACKAGE
	serialRobot.write(chr(AX_LED_OFF))
	serialRobot.write(chr(0))
	serialRobot.write(chr(0))
	serialRobot.write(chr(0))
	serialRobot.write(chr(PKG_END)) #END OF PACKAGE
#---------------------------------------------------
#closes the serial port
serialRobot.close()
#---------------------------------------------------