import serial,time,re

BASE_WIDTH = 248    # millimeters
MAX_SPEED = 300     # millimeters/second

xv11_analog_sensors = [ "WallSensorInMM",
                "BatteryVoltageInmV",
                "LeftDropInMM",
                "RightDropInMM",
                "RightMagSensor",
                "LeftMagSensor",
                "XTemp0InC",
                "XTemp1InC",
                "VacuumCurrentInmA",
                "ChargeVoltInmV",
                "NotConnected1",
                "BatteryTemp1InC",
                "NotConnected2",
                "CurrentInmA",
                "NotConnected3",
                "BatteryTemp0InC" ]

xv11_digital_sensors = [ "SNSR_DC_JACK_CONNECT",
                "SNSR_DUSTBIN_IS_IN",
                "SNSR_LEFT_WHEEL_EXTENDED",
                "SNSR_RIGHT_WHEEL_EXTENDED",
                "LSIDEBIT",
                "LFRONTBIT",
                "RSIDEBIT",
                "RFRONTBIT" ]

xv11_motor_info = [ "Brush_MaxPWM",
                "Brush_PWM",
                "Brush_mVolts",
                "Brush_Encoder",
                "Brush_RPM",
                "Vacuum_MaxPWM",
                "Vacuum_PWM",
                "Vacuum_CurrentInMA",
                "Vacuum_Encoder",
                "Vacuum_RPM",
                "LeftWheel_MaxPWM",
                "LeftWheel_PWM",
                "LeftWheel_mVolts",
                "LeftWheel_Encoder",
                "LeftWheel_PositionInMM",
                "LeftWheel_RPM",
                "RightWheel_MaxPWM",
                "RightWheel_PWM",
                "RightWheel_mVolts",
                "RightWheel_Encoder",
                "RightWheel_PositionInMM",
                "RightWheel_RPM",
                "Laser_MaxPWM",
                "Laser_PWM",
                "Laser_mVolts",
                "Laser_Encoder",
                "Laser_RPM",
                "Charger_MaxPWM",
                "Charger_PWM",
                "Charger_mAH" ]

xv11_charger_info = [ "FuelPercent",
                "BatteryOverTemp",
                "ChargingActive",
                "ChargingEnabled",
                "ConfidentOnFuel",
                "OnReservedFuel",
                "EmptyFuel",
                "BatteryFailure",
                "ExtPwrPresent",
                "ThermistorPresent[0]",
                "ThermistorPresent[1]",
                "BattTempCAvg[0]",
                "BattTempCAvg[1]",
                "VBattV",
                "VExtV",
                "Charger_mAH",
                "MaxPWM" ]

Delay_Interval = 0.3

class xv11():

    def __init__(self, portnum="/dev/ttyACM0"):
        self.portnum = portnum
        self.port = serial.Serial(self.portnum,9600,timeout=0.5)
        # Storage for motor and sensor information
        self.state = {}
        for x in xv11_analog_sensors:
            self.state.update({x:0})
        for x in xv11_digital_sensors:
            self.state.update({x:0})
        for x in xv11_motor_info:
            self.state.update({x:0})
        for x in xv11_charger_info:
            self.state.update({x:0})

        self.stop_state = True
        # turn things on
        self.setTestMode("on")
        self.setLDS("on")
        
    def setTestMode(self, value):
        """ Turn test mode on/off. """
        self.port.write("Testmode " + value + "\n")
        time.sleep(Delay_Interval)

    def exit(self):
        self.setLDS("off")
        self.setTestMode("off")


    def setLDS(self, value):
        self.port.write("setldsrotation " + value + "\n")
        time.sleep(Delay_Interval)

    def requestScan(self):
        """ Ask neato for an array of scan reads. """
        """
        self.port.flushInput()
        self.port.write("getldsscan\n")
        """
        pass

    
    def getScanRanges(self):
        scan_comp = re.compile("(\d+),(\d+),(\d+),(\d+)")
        self.totalFlush()
        self.port.write("GetLDSScan\n")
        ranges = list()
	intensities = list()
        print "================scan data=============="
        for line in self.port.readlines():
	    ##print line
            m = scan_comp.search(line)
            if m:
                ranges.append(int(m.group(2))/1000.0)
                intensities.append(int(m.group(3)))
        return ranges,intensities
        """ Read values of a scan -- call requestScan first! """
        """
        ranges = list()
        angle = 0
        try:
            line = self.port.readline()
        except:
            return []
        while line.split(",")[0] != "AngleInDegrees":
            try:
                line = self.port.readline()
            except:
                return []
        while angle < 360:
            try:
                vals = self.port.readline()
            except:
                pass
            vals = vals.split(",")
            #print angle, vals
            try:
                a = int(vals[0])
                r = int(vals[1])
                ranges.append(r/1000.0)
            except:
                ranges.append(0)
            angle += 1
        return ranges
        """

    def setMotors(self, l, r, s):
        """ Set motors, distance left & right + speed """
        #This is a work-around for a bug in the Neato API. The bug is that the
        #robot won't stop instantly if a 0-velocity command is sent - the robot
        #could continue moving for up to a second. To work around this bug, the
        #first time a 0-velocity is sent in, a velocity of 1,1,1 is sent. Then, 
        #the zero is sent. This effectively causes the robot to stop instantly.
        if (int(l) == 0 and int(r) == 0 and int(s) == 0):
            if (not self.stop_state):
                self.stop_state = True
            l = 1
            r = 1
            s = 1
        else:
            self.stop_state = False

        self.port.write("setmotor "+str(int(l))+" "+str(int(r))+" "+str(int(s))+"\n")
        time.sleep(Delay_Interval)

    def getMotors(self):
        comp_motor = re.compile("([0-9a-zA-Z_]+),(\d+)")
        self.totalFlush()
        self.port.write("getmotors\n")
        for line in self.port.readlines():
            m = comp_motor.search(line)
            if m and m.group(1) in xv11_motor_info:
                self.state[m.group(1)] = int(m.group(2))

        return [self.state["LeftWheel_PositionInMM"],self.state["RightWheel_PositionInMM"]]

        """ Update values for motors in the self.state dictionary.
            Returns current left, right encoder values. """
        """
        self.port.flushInput()
        self.port.write("getmotors\n")
        line = self.port.readline()
        while line.split(",")[0] != "Parameter":
            try:
                line = self.port.readline()
            except:
                return [0,0]
        for i in range(len(xv11_motor_info)):
            try:
                values = self.port.readline().split(",")
                self.state[values[0]] = int(values[1])
            except:
                pass
        return [self.state["LeftWheel_PositionInMM"],self.state["RightWheel_PositionInMM"]]
        """

    def getAnalogSensors(self):
        comp_motor = re.compile("([0-9a-zA-Z_]+),(\d+)")
        self.totalFlush()
        """ Update values for analog sensors in the self.state dictionary. """
        self.port.write("getanalogsensors\n")
        for line in self.port.readlines():
            m = comp_motor.search(line)
            if m and m.group(1) in xv11_analog_sensors:
                self.state[m.group(1)] = int(m.group(2))
        """
        line = self.port.readline()
        while line.split(",")[0] != "SensorName":
            try:
                line = self.port.readline()
            except:
                return
        for i in range(len(xv11_analog_sensors)):
            try:
                values = self.port.readline().split(",")
                self.state[values[0]] = int(values[1])
            except:
                pass
        """

    def getDigitalSensors(self):
        comp_motor = re.compile("([0-9a-zA-Z_]+),(\d+)")
        self.totalFlush()
        """ Update values for digital sensors in the self.state dictionary. """
        self.port.write("getdigitalsensors\n")
        for line in self.port.readlines():
            m = comp_motor.search(line)
            if m and m.group(1) in xv11_analog_sensors:
                self.state[m.group(1)] = int(m.group(2))

        """
        line = self.port.readline()
        while line.split(",")[0] != "Digital Sensor Name":
            try:
                line = self.port.readline()
            except:
                return
        for i in range(len(xv11_digital_sensors)):
            try:
                values = self.port.readline().split(",")
                self.state[values[0]] = int(values[1])
            except:
                pass
        """

    def getCharger(self):
        comp_motor = re.compile("([0-9a-zA-Z_]+),(\d+)")
        self.totalFlush()
        """ Update values for charger/battery related info in self.state dictionary. """
        self.port.write("getcharger\n")
        for line in self.port.readlines():
            m = comp_motor.search(line)
            if m and m.group(1) in xv11_analog_sensors:
                self.state[m.group(1)] = int(m.group(2))

        """
        line = self.port.readline()
        while line.split(",")[0] != "Label":
            line = self.port.readline()
        for i in range(len(xv11_charger_info)):
            values = self.port.readline().split(",")
            try:
                self.state[values[0]] = int(values[1])
            except:
                pass
        """

    def setBacklight(self, value):
        if value > 0:
            self.port.write("setled backlighton")
        else:
            self.port.write("setled backlightoff")
        time.sleep(Delay_Interval)

    def setMotorTest(self):
        self.port.write("SetMotor 60 60 10\n")
        
         

    def getMotorTest(self):
        ##sio = io.TextIOWrapper(io.BufferedRWPair(self.port, self.port))
        ##sio.write(u"GetMotors\n")
        ##print sio.readline()
        self.totalFlush()
        self.port.write("GetMotors\n")
        for line in self.port.readlines():
            print line

    def getLDS(self):
        self.totalFlush()
        self.port.write("GetLDSScan\n")
        for line in self.port.readlines():
            print line
    
    def totalFlush(self):
        ##self.port.flush()
        ##self.port.flushOutput()
        ##self.port.flushInput()
        self.port.readlines()


