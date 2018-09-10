from enum import Enum  
from mpu6050 import mpu6050
import time

g=9.81
T = 0.01
maxFallTime = 1.0
maxImpactTime = 1.0
maxRestTime = 1.0
states = Enum('states', 'wait freefall impact rest')
sensor = mpu6050(0x68)

state = states.wait
waitImpact = 0
waitRest = 0
restCounter = 0
while 1:
	data = sensor.get_accel_data()
	print(data)
	print(state)
	norm = (data['x']**2 + data['y']**2 + data['z']**2)**(1/2)
	time.sleep(T)
	if state == states.wait:
		if norm <(0.5*g):
			state = states.freefall
		else:
			state = states.wait
	elif state == states.freefall:
		if norm >(3.0*g):
			state = states.impact
			waitImpact = 0
		elif waitImpact > maxFallTime/T:
			state = states.wait
			waitImpact = 0
		else:
			waitImpact = waitImpact + 1
			state = states.freefall
	elif state == states.impact:
		if norm < (1.5*g):
			state = states.rest
			waitRest = 0
		elif waitRest > maxImpactTime/T:
			state = states.wait
			waitRest = 0
		else:
			waitRest = waitRest + 1
			state = states.impact
	#State rest
	else:
		if restCounter > maxRestTime/T:
			state = states.wait
			restCounter = 0
		else:
			state = states.rest
			restCounter = restCounter + 1



