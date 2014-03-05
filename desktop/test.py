#!/usr/bin/python

import struct
import serial
import sys
from math import sqrt, acos, pi
from ponycube import *

def normalize(i):
	mag = magnitude(i)
	# print	 "in " + str(i)
	#print "mag "+str(mag)
	i[0]/=mag
	i[1]/=mag
	i[2]/=mag
	i[3]/=mag
	# print "out " + str(i)
	return i

def magnitude(i):
	return sqrt(i[0]**2 + i[1]**2 + i[2]**2 + i[3]**2)

def quaternion_angle(i, j):
	dot = 0
	for k in range(4):
		dot+= i[k]*j[k]
	dot/=(magnitude(i)*magnitude(j))
	return acos(dot)


class cube_packet_viewer ():
    def __init__(self):
        self.screen = Screen(800,800,scale=1.5)
        self.cube = Cube(60,120,20)
        self.q = Quaternion(1,0,0,0)
        self.previous = None  # previous quaternion
        self.latest = None    # latest packet (get in dispatch, use in loop)

    def loop(self,event):
        packet = self.latest
        if packet:
            q = Quaternion(packet[0],packet[1],-packet[2],-packet[3])
            # print q
            self.cube.erase(self.screen)
            self.cube.draw(self.screen,q)
            pygame.display.flip()
            self.latest = None

    def dispatch(self,p):
        self.latest = p

if __name__ == "__main__":
	# with open('dump') as ser:
	ser = serial.Serial("/dev/ttyUSB0", baudrate=115200)
	while ser.read() != '\n':
		continue
	last = None

	pygame.init()
	viewer = cube_packet_viewer()

	for line in ser:
		split = [i for i in line.split()]
		for j in range(1, 5):
			split[j]=struct.unpack('>l', split[j].decode('hex'))[0]
			# split[j]/=1073741824.0
		cur = normalize(split[1:])

		viewer.dispatch(cur)

		# if last:
		# 	print quaternion_angle(last, cur)*180/pi
		# last=cur

		event = pygame.event.poll()
		# TODO: Allow exit via keystroke.
		if event.type == pygame.QUIT:
			viewer.close()
			break

		viewer.loop(event)

		pygame.time.delay(0)
