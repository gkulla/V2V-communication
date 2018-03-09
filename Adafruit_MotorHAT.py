#!/usr/bin/python

from Adafruit_PWM_Servo_Driver import PWM
import time
import os
import gps
import threading
import RPi.GPIO as GPIO

DISTANCE = 0

class Adafruit_StepperMotor:
	MICROSTEPS = 8
	MICROSTEP_CURVE = [0, 50, 98, 142, 180, 212, 236, 250, 255]

	#MICROSTEPS = 16
	# a sinusoidal curve NOT LINEAR!
	#MICROSTEP_CURVE = [0, 25, 50, 74, 98, 120, 141, 162, 180, 197, 212, 225, 236, 244, 250, 253, 255]
	
	def __init__(self, controller, num, steps=200):
		self.MC = controller
		self.revsteps = steps
		self.motornum = num
		self.sec_per_step = 0.1
		self.steppingcounter = 0
		self.currentstep = 0

		num -= 1

		if (num == 0):
			self.PWMA = 8
			self.AIN2 = 9
			self.AIN1 = 10
			self.PWMB = 13
			self.BIN2 = 12
			self.BIN1 = 11
		elif (num == 1):
			self.PWMA = 2
			self.AIN2 = 3
			self.AIN1 = 4
			self.PWMB = 7
			self.BIN2 = 6
			self.BIN1 = 5
		else:
			raise NameError('MotorHAT Stepper must be between 1 and 2 inclusive')

	def setSpeed(self, rpm):
		self.sec_per_step = 60.0 / (self.revsteps * rpm)
		self.steppingcounter = 0

	def oneStep(self, dir, style):
		pwm_a = pwm_b = 255

		# first determine what sort of stepping procedure we're up to
		if (style == Adafruit_MotorHAT.SINGLE):
    			if ((self.currentstep/(self.MICROSTEPS/2)) % 2): 
				# we're at an odd step, weird
				if (dir == Adafruit_MotorHAT.FORWARD):
					self.currentstep += self.MICROSTEPS/2
				else:
					self.currentstep -= self.MICROSTEPS/2
			else:
				# go to next even step
				if (dir == Adafruit_MotorHAT.FORWARD):
					self.currentstep += self.MICROSTEPS
				else:
					self.currentstep -= self.MICROSTEPS
		if (style == Adafruit_MotorHAT.DOUBLE):
			if not (self.currentstep/(self.MICROSTEPS/2) % 2):
				# we're at an even step, weird
				if (dir == Adafruit_MotorHAT.FORWARD):
					self.currentstep += self.MICROSTEPS/2
				else:
					self.currentstep -= self.MICROSTEPS/2
			else:
				# go to next odd step
				if (dir == Adafruit_MotorHAT.FORWARD):
					self.currentstep += self.MICROSTEPS
				else:
					self.currentstep -= self.MICROSTEPS
		if (style == Adafruit_MotorHAT.INTERLEAVE):
			if (dir == Adafruit_MotorHAT.FORWARD):
				self.currentstep += self.MICROSTEPS/2
			else:
				self.currentstep -= self.MICROSTEPS/2

		if (style == Adafruit_MotorHAT.MICROSTEP):
			if (dir == Adafruit_MotorHAT.FORWARD):
				self.currentstep += 1
			else:
				self.currentstep -= 1

                	# go to next 'step' and wrap around
                	self.currentstep += self.MICROSTEPS * 4
                	self.currentstep %= self.MICROSTEPS * 4

			pwm_a = pwm_b = 0
			if (self.currentstep >= 0) and (self.currentstep < self.MICROSTEPS):
				pwm_a = self.MICROSTEP_CURVE[self.MICROSTEPS - self.currentstep]
				pwm_b = self.MICROSTEP_CURVE[self.currentstep]
			elif (self.currentstep >= self.MICROSTEPS) and (self.currentstep < self.MICROSTEPS*2):
				pwm_a = self.MICROSTEP_CURVE[self.currentstep - self.MICROSTEPS]
				pwm_b = self.MICROSTEP_CURVE[self.MICROSTEPS*2 - self.currentstep]
			elif (self.currentstep >= self.MICROSTEPS*2) and (self.currentstep < self.MICROSTEPS*3):
				pwm_a = self.MICROSTEP_CURVE[self.MICROSTEPS*3 - self.currentstep]
				pwm_b = self.MICROSTEP_CURVE[self.currentstep - self.MICROSTEPS*2]
			elif (self.currentstep >= self.MICROSTEPS*3) and (self.currentstep < self.MICROSTEPS*4):
                                pwm_a = self.MICROSTEP_CURVE[self.currentstep - self.MICROSTEPS*3]
                                pwm_b = self.MICROSTEP_CURVE[self.MICROSTEPS*4 - self.currentstep]


		# go to next 'step' and wrap around
		self.currentstep += self.MICROSTEPS * 4
		self.currentstep %= self.MICROSTEPS * 4

		# only really used for microstepping, otherwise always on!
		self.MC._pwm.setPWM(self.PWMA, 0, pwm_a*16)
		self.MC._pwm.setPWM(self.PWMB, 0, pwm_b*16)

		# set up coil energizing!
		coils = [0, 0, 0, 0]

		if (style == Adafruit_MotorHAT.MICROSTEP):
			if (self.currentstep >= 0) and (self.currentstep < self.MICROSTEPS):
				coils = [1, 1, 0, 0]
                        elif (self.currentstep >= self.MICROSTEPS) and (self.currentstep < self.MICROSTEPS*2):
				coils = [0, 1, 1, 0]
                        elif (self.currentstep >= self.MICROSTEPS*2) and (self.currentstep < self.MICROSTEPS*3):
				coils = [0, 0, 1, 1]
                        elif (self.currentstep >= self.MICROSTEPS*3) and (self.currentstep < self.MICROSTEPS*4):
				coils = [1, 0, 0, 1]
		else:
			step2coils = [ 	[1, 0, 0, 0], 
				[1, 1, 0, 0],
				[0, 1, 0, 0],
				[0, 1, 1, 0],
				[0, 0, 1, 0],
				[0, 0, 1, 1],
				[0, 0, 0, 1],
				[1, 0, 0, 1] ]
			coils = step2coils[self.currentstep/(self.MICROSTEPS/2)]

		#print "coils state = " + str(coils)
		self.MC.setPin(self.AIN2, coils[0])
		self.MC.setPin(self.BIN1, coils[1])
		self.MC.setPin(self.AIN1, coils[2])
		self.MC.setPin(self.BIN2, coils[3])

		return self.currentstep

	def step(self, steps, direction, stepstyle):
		s_per_s = self.sec_per_step
		lateststep = 0
		
		if (stepstyle == Adafruit_MotorHAT.INTERLEAVE):
			s_per_s = s_per_s / 2.0
		if (stepstyle == Adafruit_MotorHAT.MICROSTEP):
			s_per_s /= self.MICROSTEPS
			steps *= self.MICROSTEPS

		print s_per_s, " sec per step"

		for s in range(steps):
			lateststep = self.oneStep(direction, stepstyle)
			time.sleep(s_per_s)

		if (stepstyle == Adafruit_MotorHAT.MICROSTEP):
			# this is an edge case, if we are in between full steps, lets just keep going
			# so we end on a full step
			while (lateststep != 0) and (lateststep != self.MICROSTEPS):
				lateststep = self.oneStep(dir, stepstyle)
				time.sleep(s_per_s)
		
class Adafruit_DCMotor:
	def __init__(self, controller, num):
		self.MC = controller
		self.motornum = num
                pwm = in1 = in2 = 0

                if (num == 0):
                         pwm = 8
                         in2 = 9
                         in1 = 10
                elif (num == 1):
                         pwm = 13
                         in2 = 12
                         in1 = 11
                elif (num == 2):
                         pwm = 2
                         in2 = 3
                         in1 = 4
                elif (num == 3):
                         pwm = 7
                         in2 = 6
                         in1 = 5
		else:
			raise NameError('MotorHAT Motor must be between 1 and 4 inclusive')
                self.PWMpin = pwm
                self.IN1pin = in1
                self.IN2pin = in2

	def run(self, command):
		if not self.MC:
			return
		if (command == Adafruit_MotorHAT.FORWARD):
			self.MC.setPin(self.IN2pin, 0)
			self.MC.setPin(self.IN1pin, 1)
		if (command == Adafruit_MotorHAT.BACKWARD):
			self.MC.setPin(self.IN1pin, 0)
			self.MC.setPin(self.IN2pin, 1)
		if (command == Adafruit_MotorHAT.RELEASE):
			self.MC.setPin(self.IN1pin, 0)
			self.MC.setPin(self.IN2pin, 0)
	def setSpeed(self, speed):
		if (speed < 0):
			speed = 0
		if (speed > 255):
			speed = 255
		self.MC._pwm.setPWM(self.PWMpin, 0, speed*16)

class Adafruit_MotorHAT:
	FORWARD = 1
	BACKWARD = 2
	BRAKE = 3
	RELEASE = 4

	SINGLE = 1
	DOUBLE = 2
	INTERLEAVE = 3
	MICROSTEP = 4

	def __init__(self, addr = 0x60, freq = 1600):
		self._i2caddr = addr            # default addr on HAT
		self._frequency = freq		# default @1600Hz PWM freq
		self.motors = [ Adafruit_DCMotor(self, m) for m in range(4) ]
		self.steppers = [ Adafruit_StepperMotor(self, 1), Adafruit_StepperMotor(self, 2) ]
		self._pwm =  PWM(addr, debug=False)
		self._pwm.setPWMFreq(self._frequency)

	def setPin(self, pin, value):
		if (pin < 0) or (pin > 15):
			raise NameError('PWM pin must be between 0 and 15 inclusive')
		if (value != 0) and (value != 1):
			raise NameError('Pin value must be 0 or 1!')
		if (value == 0):
			self._pwm.setPWM(pin, 0, 4096)
		if (value == 1):
			self._pwm.setPWM(pin, 4096, 0)

	def getStepper(self, steps, num):
                if (num < 1) or (num > 2):
                        raise NameError('MotorHAT Stepper must be between 1 and 2 inclusive')
		return self.steppers[num-1]

	def getMotor(self, num):
		if (num < 1) or (num > 4):
			raise NameError('MotorHAT Motor must be between 1 and 4 inclusive')
		return self.motors[num-1]
	
#-------------------------------------------------------------------------------------------	
#Senior Design class implementation of Temperature_Sensor, Sensor, Gps, XBee and Car classes                
#-------------------------------------------------------------------------------------------

#1 -----------------------------------------------------------------------------------------	
class Temperature_Sensor:
        
        def __init__(self, path):
                os.system('modprobe w1-gpio')
                os.system('modprobe w1-therm')
                self.path = path
                self.temp_output = 0
                
        def temp_raw(self):
                f = open(self.path, 'r')
                lines = f.readlines()
                f.close()
                return lines
        
        def read_temp(self):
                lines = self.temp_raw()
                while lines[0].strip()[-3:] != 'YES':
                        time.sleep(0.2)
                        lines = temp_raw()
                temp_output = lines[1].find('t=')
                if temp_output != -1:
                        temp_string = lines[1].strip()[temp_output+2:]
                        temp_c = float(temp_string) / 1000.0
                        return temp_c
#2-------------------------------------------------------------------------------------------------
class Sensor:
           
        def __init__(self, trigger, echo):
                self.t_sensor = Temperature_Sensor('/sys/bus/w1/devices/28-0316744489ff/w1_slave')
                self.trigger = trigger
                self.echo = echo
                self.start = 0
                self.stop = 0
                self.dist = 0
                self.distance  = 0
                self.distance1 = 0
                self.distance2 = 0
                self.distance3 = 0
                self.speedSound = 33100 + (0.6*self.t_sensor.read_temp())
                #----------------------------------------------
                GPIO.setmode(GPIO.BCM)
                GPIO.setup(self.trigger,GPIO.OUT)     #Trigger
                GPIO.setup(self.echo,GPIO.IN)         #Echo

                
        def measure(self):
                GPIO.setwarnings(False)
                flag = False
                GPIO.setmode(GPIO.BCM)
                GPIO.output(self.trigger, GPIO.LOW)
                time.sleep(0.01)
                GPIO.output(self.trigger, True)
                time.sleep(0.00001)
                GPIO.output(self.trigger, False)

                timer_interupt = time.time()
                while GPIO.input(self.echo)==0:
                        if((time.time() - timer_interupt) > 0.01);
                                flag = True
                                break
                        else
                                pass
                if(flag == False):
                        self.start = time.time()
                if(flag == False):
                        while GPIO.input(self.echo)==1:
                                pass
                        self.stop = time.time()
                if(flag == False):
                        elapsed = self.stop - self.start
                        self.distance = (elapsed*self.speedSound)/2
                        global Distance
                        return self.distance
                else:
                        return Distance
        
        def measure_average(self):
                self.distance1= self.measure()
                self.distance2= self.measure()
                self.distance3= self.measure()
                self.distance = self.distance1 + self.distance2 + self.distance3
                self.distance = self.distance/3
                return round(self.distance, 2)
#3----------------------------------------------------------------------------------------------------------
class GPS(threading.Thread):
        def __init__(self):
		threading.Thread.__init__(self)
                # Listen on port 2947 (gpsd) of localhost
                os.system("sudo systemctl stop gpsd.socket")
                os.system("sudo systemctl disable gpsd.socket")
                os.system("sudo gpsd /dev/ttyUSB0 -F /var/run/gpsd.sock")
                time.sleep(0.5)
                self.session = gps.gps("localhost", "2947")
                self.session.stream(gps.WATCH_ENABLE | gps.WATCH_NEWSTYLE)
                self.time_var = ""
                self.speed_var = 0.0
                self.lat_var = 0.0
                self.lon_var = 0.0
                self.alt_var = 0.0
                self.report = ""
		self.running = True
               
        def run(self):
                while self.running:
                    try:
                        report = self.session.next()
                               
                        if report['class'] == 'TPV':
                            if hasattr(report, 'time'):
                                self.time_var = report.time
                            if hasattr(report, 'speed'):
                                self.speed_var = report.speed * gps.MPS_TO_KPH
                            if hasattr(report, 'lat'):
                                self.lat_var = report.lat
                            if hasattr(report, 'lon'):
                                self.lon_var = report.lon
                            if hasattr (report, 'alt'):
                                self.alt_var = report.alt
                    except KeyError:
                                pass
                    except KeyboardInterrupt:
                                quit()
                    except StopIteration:
                                session = None
#4 ---------------------------------------------------------------------------------------------------------
class Car:
        def __init__(self, addr):
                self.addr = 0x60
                mh = Adafruit_MotorHAT(self.addr)
                self.myMotor_three = mh.getMotor(3)
                self.myMotor_four = mh.getMotor(4)
                self.myMotor_one = mh.getMotor(1)
                self.myMotor_two = mh.getMotor(2)

                # set the speed to start, from 0 (off) to 255 (max speed)
                self.myMotor_one.setSpeed(150)
                self.myMotor_two.setSpeed(150)
                self.myMotor_three.setSpeed(150)
                self.myMotor_four.setSpeed(150)

                self.myMotor_one.run(Adafruit_MotorHAT.FORWARD)
                self.myMotor_two.run(Adafruit_MotorHAT.FORWARD)
                self.myMotor_three.run(Adafruit_MotorHAT.FORWARD)
                self.myMotor_four.run(Adafruit_MotorHAT.FORWARD)
                # turn on motor
                self.myMotor_one.run(Adafruit_MotorHAT.RELEASE)
                self.myMotor_two.run(Adafruit_MotorHAT.RELEASE)
                self.myMotor_three.run(Adafruit_MotorHAT.RELEASE)
                self.myMotor_four.run(Adafruit_MotorHAT.RELEASE)
                
                
        def TurnLeft(self):
                self.myMotor_one.run(Adafruit_MotorHAT.FORWARD)
                self.myMotor_two.run(Adafruit_MotorHAT.FORWARD)
                self.myMotor_three.run(Adafruit_MotorHAT.RELEASE)
                self.myMotor_four.run(Adafruit_MotorHAT.RELEASE)
                
        def TurnRight(self):
                self.myMotor_one.run(Adafruit_MotorHAT.RELEASE)
                self.myMotor_two.run(Adafruit_MotorHAT.RELEASE)
                self.myMotor_three.run(Adafruit_MotorHAT.FORWARD)
                self.myMotor_four.run(Adafruit_MotorHAT.FORWARD)
                
        def Stop(self):
                self.myMotor_one.run(Adafruit_MotorHAT.RELEASE)
                self.myMotor_two.run(Adafruit_MotorHAT.RELEASE)
                self.myMotor_three.run(Adafruit_MotorHAT.RELEASE)
                self.myMotor_four.run(Adafruit_MotorHAT.RELEASE)
                
        def MoveForward(self):
                self.myMotor_one.run(Adafruit_MotorHAT.FORWARD)
                self.myMotor_two.run(Adafruit_MotorHAT.FORWARD)
                self.myMotor_three.run(Adafruit_MotorHAT.FORWARD)
                self.myMotor_four.run(Adafruit_MotorHAT.FORWARD)

        def MoveBackward(self):
                self.myMotor_one.run(Adafruit_MotorHAT.BACKWARD)
                self.myMotor_two.run(Adafruit_MotorHAT.BACKWARD)
                self.myMotor_three.run(Adafruit_MotorHAT.BACKWARD)
                self.myMotor_four.run(Adafruit_MotorHAT.BACKWARD)
                
        def Right(self, value, speed_limit):
                for i in range(0, value):
                        self.myMotor_one.setSpeed(0)
                        self.myMotor_two.setSpeed(0)
                        self.myMotor_three.setSpeed(i)
                        self.myMotor_four.setSpeed(i)
                self.myMotor_one.setSpeed(speed_limit)
                self.myMotor_two.setSpeed(speed_limit)
                self.myMotor_three.setSpeed(speed_limit)
                self.myMotor_four.setSpeed(speed_limit)

        def Left(self, value, speed_limit):
                for i in range(0, value):
                        self.myMotor_one.setSpeed(i)
                        self.myMotor_two.setSpeed(i)
                        self.myMotor_three.setSpeed(0)
                        self.myMotor_four.setSpeed(0)
                self.myMotor_one.setSpeed(speed_limit)
                self.myMotor_two.setSpeed(speed_limit)
                self.myMotor_three.setSpeed(speed_limit)
                self.myMotor_four.setSpeed(speed_limit)

        def Accelerate(self, value):
                for i in range(0, value):
                        self.myMotor_one.setSpeed(i)
                        self.myMotor_two.setSpeed(i)
                        self.myMotor_three.setSpeed(i)
                        self.myMotor_four.setSpeed(i)

        def DeAccelerate(self, value):
                for i in range(0, value):
                        self.myMotor_one.setSpeed(value - i)
                        self.myMotor_two.setSpeed(value - i)
                        self.myMotor_three.setSpeed(value - i)
                        self.myMotor_four.setSpeed(value - i)
#5-----------------------------------------------------------------------------
class XBee():
    RxBuff = bytearray()
    RxMessages = deque()

    def __init__(self, serialport, baudrate=9600):
        
        self.serial = serial.Serial(port=serialport, baudrate=baudrate)

    def Receive(self):
        """
           Receives data from serial and checks buffer for potential messages.
           Returns the next message in the queue if available.
        """
        remaining = self.serial.inWaiting()
        while remaining:
            chunk = self.serial.read(remaining)
            remaining -= len(chunk)
            self.RxBuff.extend(chunk)

        msgs = self.RxBuff.split(bytes(b'\x7E'))
        for msg in msgs[:-1]:
            self.Validate(msg)

        self.RxBuff = (bytearray() if self.Validate(msgs[-1]) else msgs[-1])

        if self.RxMessages:
            return self.RxMessages.popleft()
        else:
            return None

    def Validate(self, msg):
        """
        Parses a byte or bytearray object to verify the contents are a
          properly formatted XBee message.

        Inputs: An incoming XBee message

        Outputs: True or False, indicating message validity
        """
        # 9 bytes is Minimum length to be a valid Rx frame
        #  LSB, MSB, Type, Source Address(2), RSSI,
        #  Options, 1 byte data, checksum
        if (len(msg) - msg.count(bytes(b'0x7D'))) < 9:
            return False

        # All bytes in message must be unescaped before validating content
        frame = self.Unescape(msg)

        LSB = frame[1]
        # Frame (minus checksum) must contain at least length equal to LSB
        if LSB > (len(frame[2:]) - 1):
            return False

        # Validate checksum
        if (sum(frame[2:3+LSB]) & 0xFF) != 0xFF:
            return False

        #print("Rx: " + self.format(bytearray(b'\x7E') + msg))
        #print ("Rx decoded:"+msg[7:-1].decode('ascii'))
        self.sensor_decode(msg)
        self.RxMessages.append(frame)
        return True

    def SendStr(self, msg, addr=0xFFFF, options=0x01, frameid=0x00):
        return self.Send(msg.encode('utf-8'), addr, options, frameid)

    def Send(self, msg=None, addr=0xFFFF, options=0x01, frameid=0x00):
        if not msg:
            return 0

        hexs = '7E 00 {:02X} 01 {:02X} {:02X} {:02X} {:02X}'.format(
            len(msg) + 5,           # LSB (length)
            frameid,
            (addr & 0xFF00) >> 8,   # Destination address high byte
            addr & 0xFF,            # Destination address low byte
            options
        )
        
        
        frame = bytearray.fromhex(hexs)
        #  Append message content
        
        frame.extend(msg)

        # Calculate checksum byte
        frame.append(0xFF - (sum(frame[3:]) & 0xFF))

        # Escape any bytes containing reserved characters
        frame = self.Escape(frame)
        
        print("Tx: " + self.format(frame))
        return self.serial.write(frame)

    def Unescape(self, msg):
        if msg[-1] == 0x7D:
            # Last byte indicates an escape, can't unescape that
            return None

        out = bytearray()
        skip = False
        for i in range(len(msg)):
            if skip:
                skip = False
                continue

            if msg[i] == 0x7D:
                out.append(msg[i+1] ^ 0x20)
                skip = True
            else:
                out.append(msg[i])
        return out

    def Escape(self, msg):
        escaped = bytearray()
        reserved = bytearray(b"\x7E\x7D\x11\x13")

        escaped.append(msg[0])
        for m in msg[1:]:
            if m in reserved:
                escaped.append(0x7D)
                escaped.append(m ^ 0x20)
            else:
                escaped.append(m)
        return escaped

    def format(self, msg):
        return " ".join("{:02x}".format(b) for b in msg)

    def sensor_decode(self,data):
        sensor_data=data[7:-1].decode('ascii').split(',')
        print ("** Rx: FC: {0} | FR: {1} | FL: {2}**".format(sensor_data[0],sensor_data[1],sensor_data[2]))
        
                
                


                
        
        
