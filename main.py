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
		
		print acc
		
		Infos = {}
		Infos['Bumps'] = {}
		Infos['Bumps']['Left'] = bool(acc[0] & 0x02)
		Infos['Bumps']['Right'] = bool(acc[0] & 0x01)
		Infos['Wheel drop'] = {}
		Infos['Wheel drop']['Right'] = bool(acc[0] & 0x04)
		Infos['Wheel drop']['Left'] = bool(acc[0] & 0x08)
		Infos['Cliff'] = {}
		Infos['Cliff']['Left'] = bool(acc[2])
		Infos['Cliff']['Front left'] = bool(acc[3])
		Infos['Cliff']['Front right'] = bool(acc[4])
		Infos['Cliff']['Right'] = bool(acc[5])
		Infos['Wall'] = bool(acc[1])
		Infos['Wheel overcurrents'] = {}
		Infos['Wheel overcurrents']['Side brush'] = bool(acc[7] & 0x01)
		Infos['Wheel overcurrents']['Main brush'] = bool(acc[7] & 0x04)
		Infos['Wheel overcurrents']['Right wheel'] = bool(acc[7] & 0x08)
		Infos['Wheel overcurrents']['Left wheel'] = bool(acc[7] & 0x10)
		Infos['Dirt detect'] = bool(acc[8])
		Infos['Distance'] = DecodeSignedShort(acc[12], acc[13])
		Infos['Angle'] = DecodeSignedShort(acc[14], acc[15])
		Infos['Charging state'] = {0: 'Not charging', 1: 'Reconditioning Charging', 2: 'Full Charging', 3: 'Trickle Charging', 4: 'Waiting', 5: 'Charging Fault Condition'}[acc[16]]
		Infos['Voltage'] = DecodeUnsignedShort(acc[17], acc[18])
		Infos['Current'] = DecodeSignedShort(acc[19], acc[20])
		Infos['Temperature'] = acc[21]
		
		print 'Received : ' + str(Infos)
		time.sleep(1)
	
# except:
	# print 'Error detected : ' + str(sys.exc_info()[0])

Stop()
PassiveMode()

# Close the serial port; we're done for now.
ser.close()

