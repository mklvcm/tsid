import math
import numpy

class Contact:
	stepHeight = 0.05

	def __init__(self):
		self._inAir = [False, False]
		self._timeDetached = 0.0
		self._startLocation = 0.0
		self._targetLocation = 0.0
		self._airDuration = 0.0
		self._horizLocation = 0.0
		self._vertLocation = 0.0
		self._shAngle = 0.0
		self._knAngle = 0.0
		self._justDetached = False
		self._justAttached = False

	# def Detach(self, time, targetLocation, airDuration):
	# 	self._inAir[0] = True
	# 	self._timeDetached = time
	# 	self._startLocation = self._horizLocation
	# 	self._targetLocation = targetLocation
	# 	self._airDuration = airDuration

	def Attach(self):
	 	self._inAir[0] = False
	 	self._vertLocation = 0
		self._justAttached = True

	def Tick(self, time, inAir, targetLocation, airDuration):
	 	self._inAir[0] = inAir
		self._targetLocation = targetLocation
		self._airDuration = airDuration

		if self._inAir[0]:
			inAirFraction = (time - self._timeDetached) / self._airDuration
			targetDistance = self._targetLocation - self._startLocation
			self._horizLocation = self._startLocation + targetDistance * inAirFraction
			self._vertLocation = Gait.STEP_HEIGHT * math.sin(inAirFraction * math.pi)
		else:
			pass # stays in contact and nothing happens

		self._justDetached = (self._inAir[1] == False) and (self._inAir[0] == True)
		self._justAttached = (self._inAir[1] == True) and (self._inAir[1] == False)

		if self._justDetached:
			self._timeDetached = time
			self._startLocation = self._horizLocation

		if self._justAttached:
			self._vertLocation = 0

		self._inAir[1] = self._inAir[0]
		

class Gait:
	LINK1 = 0.25
	LINK2 = 0.25
	COM_HEIGHT = 0.4
	MAX_STEP_LENGTH = 0.2
	SLOWEST_STEP = 1.5
	FASTEST_STEP = 0.4
	MAX_SPEED = MAX_STEP_LENGTH / FASTEST_STEP
	STEP_HEIGHT = 0.05

	##

	_prevTime = 0.0
	_cycleStartTime = 0.0
	_subCycleStartTime = 0.0
	_nContact = 0

	## public

	_comLocation = 0.0
	_contacts = [Contact() for _ in range(4)]
	_activeContact = _contacts[0]

	def calcAngles(self, comX, contactX, contactZ):
		a = comX - contactX
		b = self.COM_HEIGHT - contactZ
		c = math.sqrt(a * a + b * b)
		B1 = math.atan(a / b)

		a = self.LINK1
		b = self.LINK2
		A = math.acos((a*a + b*b - c*c) / (2 * a * b))
		B = math.acos((a*a + c*c - b*b) / (2 * a * c))
		return B + B1, A # sh and kn angles

	def Tick(self, time, speed, accel):
		if speed < 0: raise ValueError()
		if speed > self.MAX_SPEED: speed = self.MAX_SPEED

		dt = time - self._prevTime
		
		# calc step length and step cycle duration based on speed
		# (with speed increase, length grows fast first, then slow)
		
		stepLength = self.MAX_STEP_LENGTH * math.sin(speed / self.MAX_SPEED * math.pi / 4)
		stepCycleDuration = stepLength / speed
		stepAirDuration = min(self.SLOWEST_STEP, stepCycleDuration / 4)
		
		if stepAirDuration < self.FASTEST_STEP: 
			stepAirDuration = self.FASTEST_STEP
			stepCycleDuration = stepAirDuration * 4
			stepLength = speed * stepCycleDuration

		if stepCycleDuration < stepAirDuration:
			stepCycleDuration = stepAirDuration

		stepSubCycleDuration = stepCycleDuration / 4

		# how much COM progressed?

		self._comLocation += speed * dt

		# is cycle finished?

		if time - self._cycleStartTime > stepCycleDuration:
			self._cycleStartTime += stepCycleDuration

		# sub-sycle and which contact

		if time - self._subCycleStartTime > stepSubCycleDuration:
			self._subCycleStartTime += stepSubCycleDuration
			self._nContact = (self._nContact + 1) % 4
			self._activeContact.Attach()
			self._activeContact = self._contacts[self._nContact]

		# see if contact is in air or not

		airStartTime = (stepSubCycleDuration - stepAirDuration)/2
		airEndTime = airStartTime + stepAirDuration
		inAir = False
		targetLocation = 0

		if airStartTime <= time - self._subCycleStartTime and time - self._subCycleStartTime <= airEndTime:
			inAir = True
			targetLocation = self._comLocation + stepSubCycleDuration * speed + stepLength/2

		# let contact calc its position

		self._activeContact.Tick(time, inAir, targetLocation, stepAirDuration)

		# calc angles for each contact

		for i in range(4):
			c = self._contacts[i]
			sh, kn = self.calcAngles(self._comLocation, c._horizLocation, c._vertLocation)
			c._shAngle = sh
			c._knAngle = kn

		self._prevTime = time

## main

# gait = Gait()
# c0 = gait._contacts[0]
# c1 = gait._contacts[1]

# for t in numpy.arange(0, 6, 0.1):
# 	gait.Tick(t, 0.1, 0)
# 	print ("%.4f: %.2f | %.2f (%.2f %.2f), %s | %.2f (%.2f %.2f), %s" % (t, gait._comLocation, c0._horizLocation, c0._shAngle * 180 / math.pi, c0._knAngle * 180 / math.pi, "Y" if c0._inAir else "_", c1._horizLocation, c1._shAngle * 180 / math.pi, c1._knAngle * 180 / math.pi, "Y" if c1._inAir else "_"))
