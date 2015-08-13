import serial
import time
import struct
import sys

ROOMBA_OPCODES = dict(
    start = 128,
    baud = 129,
    control = 130,
    safe = 131,
    full = 132,
    power = 133,
    spot = 134,
    clean = 135,
    max = 136,
    drive = 137,
    motors = 138,
    leds = 139,
    song = 140,
    play = 141,
    sensors = 142,
    force_seeking_dock = 143,
    )

def SendIntArray(bytes):
	print 'Sent : ' + str(bytes)
	ser.write(struct.pack('B' * len(bytes), *bytes))

def Avance():
	# Avance
	ser.write('\x89\x08\x00\x00\x00')
	
def Stop():
	# STOP NOW!
	ser.write('\x89\x00\x00\x00\x00')

def Recule():
	ser.write('\x89\xFE\x0C\x00\x00')

def Musique():
	# Program a five-note start song into Roomba.
	SendIntArray([140, 0, 5, 67, 16, 72, 24, 74, 8, 76, 16, 79, 32])
	time.sleep(.1)

	# Play the song we just programmed.
	SendIntArray([141, 0])
	time.sleep(1.6) # wait for the song to complete

def SafeMode():
	SendIntArray([131])
	time.sleep(.1)

def PassiveMode():
	# Leave the Roomba in passive mode; this allows it to keep
	# running Roomba behaviors while we wait for more commands.
	SendIntArray([130])
	
def WakeUp():
	SendIntArray([ROOMBA_OPCODES['start']])
	time.sleep(.1)

def DecodeSignedShort(high, low):
	return struct.unpack('>h', struct.pack('BB', high, low))[0]
	
def DecodeUnsignedShort(high, low):
	return struct.unpack('>H', struct.pack('BB', high, low))[0]
	
# STARTS HERE

if True:
# try:
	# Open a serial connection to Roomba
	ser = serial.Serial(port='/dev/ttyAMA0', baudrate=115200, timeout=0)
	
	WakeUp()
	
	# Assuming the robot is awake, start safe mode so we can hack.
	SafeMode()
	
	# Musique()
	
	while True:
		# Request all sensors data
		SendIntArray([142, 100])
		
		acc = []
		while len(acc) != 80:
			res = ser.read(1)
			if res:
				acc += struct.unpack('B', res)
		
		Infos = {}
		Infos['Bumps'] = {}
		Infos['Cliff'] = {}
		Infos['Cliff']['Presence'] = {}
		Infos['Cliff']['Signal'] = {}
		Infos['Battery'] = {}
		Infos['Motor current'] = {}
		Infos['Motor current']['Overcurrents'] = {}
		Infos['Motor current']['Signal'] = {}
		Infos['Wall'] = {}
		Infos['Wheel drop'] = {}
		Infos['Movement'] = {}
		Infos['Light bump'] = {}
		Infos['Misc'] = {}
		Infos['Useless'] = {}
		
		Infos['Bumps']['Left'] = bool(acc[0] & 0x02)
		Infos['Bumps']['Right'] = bool(acc[0] & 0x01)
		
		Infos['Wheel drop']['Right'] = bool(acc[0] & 0x04)
		Infos['Wheel drop']['Left'] = bool(acc[0] & 0x08)
		
		Infos['Cliff']['Presence']['Left'] = bool(acc[2])
		Infos['Cliff']['Presence']['Front left'] = bool(acc[3])
		Infos['Cliff']['Presence']['Front right'] = bool(acc[4])
		Infos['Cliff']['Presence']['Right'] = bool(acc[5])
		
		Infos['Cliff']['Signal']['Left'] = DecodeUnsignedShort(acc[28], acc[29])
		Infos['Cliff']['Signal']['Front left'] = DecodeUnsignedShort(acc[30], acc[31])
		Infos['Cliff']['Signal']['Front right'] = DecodeUnsignedShort(acc[32], acc[33])
		Infos['Cliff']['Signal']['Right'] = DecodeUnsignedShort(acc[34], acc[35])
		
		Infos['Motor current']['Overcurrents']['Side brush'] = bool(acc[7] & 0x01)
		Infos['Motor current']['Overcurrents']['Main brush'] = bool(acc[7] & 0x04)
		Infos['Motor current']['Overcurrents']['Right wheel'] = bool(acc[7] & 0x08)
		Infos['Motor current']['Overcurrents']['Left wheel'] = bool(acc[7] & 0x10)
		Infos['Motor current']['Signal']['Side brush'] = DecodeSignedShort(acc[76], acc[77])
		Infos['Motor current']['Signal']['Main brush'] = DecodeSignedShort(acc[74], acc[75])
		Infos['Motor current']['Signal']['Right wheel'] = DecodeSignedShort(acc[72], acc[73])
		Infos['Motor current']['Signal']['Left wheel'] = DecodeSignedShort(acc[70], acc[71])
		
		Infos['Battery']['Charging state'] = {0: 'Not charging', 1: 'Reconditioning Charging', 2: 'Full Charging', 3: 'Trickle Charging', 4: 'Waiting', 5: 'Charging Fault Condition'}[acc[16]]
		Infos['Battery']['Charge'] = DecodeUnsignedShort(acc[22], acc[23])
		Infos['Battery']['Capacity'] = DecodeUnsignedShort(acc[24], acc[25])
		Infos['Battery']['Charger available'] = {'Home base' : bool(acc[38] & 0x02), 'Internal charger' : bool(acc[38] & 0x01)}
		Infos['Battery']['Voltage'] = DecodeUnsignedShort(acc[17], acc[18])
		Infos['Battery']['Current'] = DecodeSignedShort(acc[19], acc[20])
		
		Infos['Wall']['Presence'] = bool(acc[1])
		Infos['Wall']['Signal'] = DecodeUnsignedShort(acc[26], acc[27])
		
		Infos['Movement']['Distance'] = DecodeSignedShort(acc[12], acc[13])
		Infos['Movement']['Angle'] = DecodeSignedShort(acc[14], acc[15])
		Infos['Movement']['Velocity'] = {'Global' : DecodeSignedShort(acc[43], acc[44]), 'Right' : DecodeSignedShort(acc[49], acc[50]), 'Left' : DecodeSignedShort(acc[47], acc[48])}
		Infos['Movement']['Radius'] = DecodeSignedShort(acc[45], acc[46])
		Infos['Movement']['Encoder counts'] = {'Left' : DecodeUnsignedShort(acc[51], acc[52]), 'Right' : DecodeUnsignedShort(acc[53], acc[54])}
		Infos['Movement']['Stasis'] = bool(acc[78])
		
		Infos['Light bump']['Presence'] = {'Left' : bool(acc[55] & 0x01), 'Front Left' : bool(acc[55] & 0x02), 'Center Left' : bool(acc[55] & 0x04), 'Center Right' : bool(acc[55] & 0x08), 'Front Right' : bool(acc[55] & 0x10), 'Right' : bool(acc[55] & 0x20)}
		Infos['Light bump']['Signal'] = {'Left' : DecodeUnsignedShort(acc[56], acc[57]), 'Front Left' : DecodeUnsignedShort(acc[58], acc[59]), 'Center Left' : DecodeUnsignedShort(acc[60], acc[61]), 'Center Right' : DecodeUnsignedShort(acc[62], acc[63]), 'Front Right' : DecodeUnsignedShort(acc[64], acc[65]), 'Right' : DecodeUnsignedShort(acc[66], acc[67])}
		
		Infos['Misc']['Dirt detect'] = bool(acc[8])
		Infos['Misc']['Temperature'] = acc[21]
		Infos['Misc']['Mode'] = {0: 'Off', 1: 'Passive', 2: 'Safe', 3: 'Full'}[acc[39]]
		Infos['Misc']['Song'] = {'Number' : acc[40], 'Playing' : bool(acc[41])}
		Infos['Misc']['Buttons'] = acc[11]
		
		Infos['Useless']['Packet size'] = acc[42]
		Infos['Useless']['Infrared optcode'] = {'Global' : acc[10], 'Left' : acc[68], 'Right' : acc[69]}
		
		print 'Received : ' + str(Infos)
		time.sleep(10)
	
# except:
	# print 'Error detected : ' + str(sys.exc_info()[0])

Stop()
PassiveMode()

# Close the serial port; we're done for now.
ser.close()

