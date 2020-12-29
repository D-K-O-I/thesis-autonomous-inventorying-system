import sqlite3
import operator
import threading
import time
import math
import os
import cv2
import socket
import numpy as np
from vcm import VCM
from stats import Stats

#CONSTS: CHANGE IN CALIBRATION
SHELF_HEIGHT = 0
CURRENT_POS = (0,3)
DRONE_DISTANCE = 0.2
DRONE_DISTANCE_X = 85
DRONE_DISTANCE_Y = 120
DRONE_DISTANCE_Z = 20

#EXPERIMENTAL RATIOS FOR THE DRONE (DJI TELLO EDU)
#DRONE_DISTANCE_X :== 0.68 * DRONE_DISTANCE_Y

TYPE0_LOCK = False
TYPE1_LOCK = False
TYPE2_LOCK = False
TYPE3_LOCK = False
ENTERED_CV = False
TURN = False
TURN_FIX = 0
OFFSET = 0

RETRY = 0
MAX_TIME_OUT = 3
ANTI_AUTO_LAND = 0

YAW = 0

def empty(a):
	"""
	OpenCV empty function.
	"""
	pass

def identify_correction(frame_contour, point_origin, point_end, area, w, h, s):
	"""
	Identifies adjustments to be made to the drone's positioning and issues commands for them.
	Receives frame_contour: video frame with contour to be evaluated
	Receives point_origin: centroid of the shape
	Receives point_end: center of the video frame
	Receives area: area of the shape's bounding rectangle
	Receives w: width of the bounding rectangle
	Receives h: height of the bounding rectangle
	Receives s: the socket object with which to send commands to the drone

	Returns corr_num: once an adjustment is made, corr_num is returned
	"""
	global TYPE0_LOCK
	global TYPE1_LOCK
	global TYPE2_LOCK
	global TYPE3_LOCK
	print("[INFO] Filling empty buffer with course correction commands...")
	corr_num = 0
	
	NORM_MAX = 100
	AREA_MAX = 26000
	AREA_MIN = 23000

	#TYPE 0 CORRECTION
	#norm to center greater than expected horizontally => horizontal LEFT/RIGHT movement
	if abs(point_end[0] - point_origin[0]) > NORM_MAX:
		corr_num += 1
		if point_end[0] - point_origin[0] > 0:
			cv2.putText(frame_contour, "LEFT", (0, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
			#s.sendto('left 20'.encode('utf-8'), tello_address)
			send_command('left 20', s, 2)
			print("LEFT")
		elif point_end[0] - point_origin[0] < 0:
			cv2.putText(frame_contour, "RIGHT", (0, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
			#s.sendto('right 20'.encode('utf-8'), tello_address)
			send_command('right 20', s, 2)
			print("RIGHT")
	else:
		if TYPE0_LOCK == False:
			cv2.putText(frame_contour, "STOP", (0, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
			#s.sendto('stop'.encode('utf-8'), tello_address)
			send_command('stop', s, 0)
			TYPE0_LOCK = True
			#TYPE1_LOCK = False
			#TYPE2_LOCK = False
			print("STOP: type 0 locked!")
			return corr_num


	#TYPE 1 CORRECTION
	#norm to center greater than expected vertically => vertical UP/DOWN movement
	if abs(point_end[1] - point_origin[1]) > NORM_MAX:
		corr_num += 1
		if point_end[1] - point_origin[1] > 0:
			cv2.putText(frame_contour, "UP", (0, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
			#s.sendto('up 20'.encode('utf-8'), tello_address)
			send_command('up 20', s, 2)
			print("UP")
		elif point_end[1] - point_origin[1] < 0:
			cv2.putText(frame_contour, "DOWN", (0, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
			#s.sendto('down 20'.encode('utf-8'), tello_address)
			send_command('down 20', s, 2)
			print("DOWN")
	else:
		if TYPE1_LOCK == False:
			cv2.putText(frame_contour, "STOP", (0, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
			#s.sendto('stop'.encode('utf-8'), tello_address)
			send_command('stop', s, 0)
			#TYPE0_LOCK = False
			TYPE1_LOCK = True
			#TYPE2_LOCK = False
			print("STOP: type 1 locked!")
			return corr_num


	#TYPE 2 CORRECTION
	#area differs from expected => horizontal FRONT/BACK movement
	#@60 cm distance: 33000 => ideal area
	if area > AREA_MAX:
		corr_num += 1
		cv2.putText(frame_contour, "BACK", (0, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
		#s.sendto('back 20'.encode('utf-8'), tello_address)
		send_command('back 20', s, 2)
		print("BACK")
	elif area < AREA_MIN and area > 15000:
		corr_num += 1
		cv2.putText(frame_contour, "FWD", (0, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
		#s.sendto('forward 20'.encode('utf-8'), tello_address)
		send_command('forward 20', s, 2)
		print("FORWARD")
	else:
		if TYPE2_LOCK == False:
			cv2.putText(frame_contour, "STOP", (0, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
			#s.sendto('stop'.encode('utf-8'), tello_address)
			send_command('stop', s, 0)
			#TYPE0_LOCK = False
			#TYPE1_LOCK = False
			TYPE2_LOCK = True
			print("STOP: type 2 locked!")
			return corr_num


	#TYPE 3 CORRECTION
	#DEPRECATED: REPLACED BY get_attitude() AND SUBSEQUENT yaw_correction()
	#angle to marker greater than expected => add/subtract angle in next rotation -- HOW TO CHECK ANGLE RELATIVE TO A WALL (ANGLE OF APPROACH)?
	#attempt: if shape not a perfect square (AKA if area is not minimized compared to known calibrated value)
	#rotate cw/ccw until area is minimized
	if (w-30) < h or (w+30) > h:
		#print("TYPE 3 CORRECTION")
		cv2.putText(frame_contour, "RTT", (0, 170), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
		#thread_broker("cw 360", 3, q)
	else:
		cv2.putText(frame_contour, "STOP", (0, 170), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
		#thread_broker("stop", 3, q)
		#thread_broker("back " + str(DRONE_DIST*mult), 2, q)

	#if corr_num == 0:
	#	#s.sendto('stop'.encode('utf-8'), tello_address)
	#	send_command('stop', s)
	#	pass
	print(TYPE0_LOCK)
	print(TYPE1_LOCK)
	print(TYPE2_LOCK)
	print(corr_num)
	return corr_num

def get_contours(img, frame_contour, cap_width, cap_height, s):
	"""
	Gets Visual Marker contour. 
	Sets Bounding Rectangle.
	Calculates distance to shape centorid and shape area.
	Receives img: treated video frame
	Receives frame_contour: clean video frame to draw contours on
	Receives cap_width: video capture width constant
	Receives cap_height: video capture height constant
	Receives s: the socket object with which to send commands to the drone

	Returns corr_num: once an adjustment is made, corr_num is returned
	"""
	contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:]
	
	corr_num = None

	for cnt in contours:
		area = cv2.contourArea(cnt)
		#areaMin = cv2.getTrackbarPos("Area", "Parameter")
		if area >= 5000:
			cv2.drawContours(frame_contour, contours, -1, (255,0,255), 7)

			perimeter = cv2.arcLength(cnt, True)
			approx = cv2.approxPolyDP(cnt, 0.02 * perimeter, True)

			x,y,w,h = cv2.boundingRect(approx)
			#cv2.rectangle(frame_contour, (x,y), (x+w, y+h), (0, 255, 0), 5)
			
			cv2.rectangle(frame_contour,(x,y),(x+w,y+h),(0,255,0),2)
			cv2.circle(frame_contour, (x+(w/2), y+(h/2)), 10, (0, 255, 0), -1)
			cv2.putText(frame_contour, "centroid", (x+(w/2) - 25, y+(h/2) - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

			line_p1 = (x+(w/2), y+(h/2))
			line_p2 = (cap_width/2,cap_height/2)

			line_vector = cv2.line(frame_contour, line_p1, line_p2, (0,255,0), 2)
			line_p1_np = np.asarray(line_p1)
			line_p2_np = np.asarray(line_p2)
			distance_vector_norm = np.linalg.norm(line_p2_np - line_p1_np)

			cv2.putText(frame_contour, "MOVE:", (0, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

			cv2.putText(frame_contour, str(distance_vector_norm), line_p2, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
			cv2.putText(frame_contour, str(area), (line_p2[0], line_p2[1]+25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

			cv2.line(frame_contour, (0,0), (cap_width,cap_height), (255,255,255), 1)
			cv2.line(frame_contour, (cap_width,0), (0,cap_height), (255,255,255), 1)

			#cv2.imshow("framecontour", frame_contour)

			corr_num = identify_correction(frame_contour, line_p1, line_p2, area, w, h, s)
			#identify_correction(frame_contour, line_p1, line_p2, area, w, h, s)

	return corr_num

def rfid_check(drone_position):
	"""
	Gets buffer from live rfid interface.
	If the desired tag is in the buffer: break
	Otherwise, if timeout reached: carry on
	Receives drone_position: current drone position to check against
	"""

	print("opening...")
	f = open(r'tag.txt', 'r')
	print("reading...")
	tag = f.readline()
	print(tag)
	f.close()

	try:
		if tag[0] == '0':
			tagx = int(tag[1:3])
		else:
			tagx = -int(tag[1:3])
		if tag[3] == '0':
			tagy = int(tag[4:6])
		else:
			tagy = -int(tag[4:6])

		print(tagx, tagy, drone_position)
		if (tagx,tagy) == drone_position:
			print("[INFO] Position confirmed with RFID!")
			return True
		else:
			print("[INFO] Found wrong tag. The buffer was not updated / updated in time")
			return False
	except:
		pass

def split_move(mov):
	"""
	Sends movement commands for the drone to traverse the warehouse.
	Receives mov: distance to move.
	"""
	send_command('forward ' + str(mov), s, 5)
	print("[INFO] Finished movement.")

def correct_yaw(yaw_t, yaw_c, s):
	"""
	Corrects drone yaw to the expected heading by sending rotation commands.
	Receives yaw_t: its target (or expected) yaw
	Receives yaw_c: its current yaw
	Receives s: the socket object with which to send commands to the drone

	"""
	#NORTH-FACING
	if yaw_t == 0 and yaw_c < 0:
		print("[INFO] FIXING YAW...")
		send_command('cw ' + str(abs(yaw_c)), s, 2)
	elif yaw_t == 0 and yaw_c > 0:
		print("[INFO] FIXING YAW...")
		send_command('ccw ' + str(abs(yaw_c)), s, 2)
	#SOUTH-FACING
	elif yaw_t == 180 and yaw_c < 0:
		print("[INFO] FIXING YAW...")
		send_command('ccw ' + str(yaw_t - abs(yaw_c)), s, 2)
	elif yaw_t == 180 and yaw_c > 0:
		print("[INFO] FIXING YAW...")
		send_command('cw ' + str(yaw_t - abs(yaw_c)), s, 2)
	#EAST-FACING
	elif yaw_t == 90 and abs(yaw_c) < 90:
		print("[INFO] FIXING YAW...")
		send_command('cw ' + str(yaw_t - yaw_c), s, 2)
	elif yaw_t == 90 and abs(yaw_c) > 90:
		print("[INFO] FIXING YAW...")
		send_command('ccw ' + str((abs(yaw_t - yaw_c))%180), s, 2)
	#WEST-FACING
	elif yaw_t == 270 and abs(yaw_c) < 90:
		print("[INFO] FIXING YAW...")
		send_command('ccw ' + str((yaw_t + yaw_c)%180), s, 2)
	elif yaw_t == 270 and abs(yaw_c) > 90:
		print("[INFO] FIXING YAW...")
		send_command('cw ' + str((yaw_t - yaw_c)%180), s, 2)

	else:
		print("[INFO] YAW OK")

	global TURN
	global TURN_FIX

	TURN = True
	time.sleep(0.5)
	send_command('ccw ' + str(TURN_FIX), s, 5)
	TURN_FIX = 0


def get_attitude():
	"""
	Polls drone for roll, pitch, yaw.
	Returns attitude: response from drone.
	"""
	attitude = send_command('attitude?', s, 0)
	return attitude

def movement_controller(vcm, nodemap, s):
	"""
	Controls the drone's movements.
	Receives vcm: the virtual coordinates map object
	Receives nodemap: the dictionary of nodes and their coordinate sets
	Receives s: the socket object with which to send commands to the drone

	Polls drone for status.
	Determines each movement to be sent according to the 3D-route.
	Whenever a Hybrid Checkpoint is reached, engages computer vision algorithm.

	"""
	print("-----------------------------")
	print("BEGINNING MOVEMENT CONTROL...")
	print("-----------------------------")
	#start position
	drone_position = nodemap[vcm.traverse_order[0][0]]
	#North Reference
	drone_heading_current = 0

	#UNCOMMENT FOR DRONE TEST FLIGHT
	#get_attitude()
	send_command('battery?', s, 0)
	send_command('speed 20', s, 0)

	send_command('takeoff', s, 5)
	#get_attitude()
	send_command('up ' + str(int(DRONE_DISTANCE_Z*2.5)), s, 5)

	#print(vcm.traverse_order)
	for shelf_level in vcm.traverse_order:
		pair_stop = shelf_level[0]

		#break down each shelf level into a list of ( ORIGIN , DESTINATION ) 1-node hops
		node_pair_list = zip(shelf_level, shelf_level[1:] + [shelf_level[0]])
		print(node_pair_list)
		#print(pair_stop)
		for node, next_node in node_pair_list:
			#check if drone isn't at the end of the level
			if (next_node != pair_stop or node_pair_list[-1][0] == node_pair_list[-1][1]) and (node != next_node):
				print("NEXT MOVE: " + str(node) + " --> " + str(next_node))
				
				#make MOVEMENT command
				movement_vector = (nodemap[next_node][0] - nodemap[node][0], nodemap[next_node][1] - nodemap[node][1])
				#print("Movement vector: " + str(movement_vector))

				print("[INFO] DRONE POSITION: " + str(drone_position))

				#CHECK IF AT THE NORTHERN/SOUTHERN EDGE OF THE NODEMAP (AKA IF CURRENTLY AT EDGE AND PREVIOUS NODE WAS NOT AT EDGE)
				if (drone_position[1] == nodemap[min(nodemap, key=lambda k: nodemap[k][1])][1] or drone_position[1] == nodemap[max(nodemap, key=lambda k: nodemap[k][1])][1]):

					#MAKE ROTATION CORRECTIONS IF NEEDED
					global YAW
					global OFFSET

					yaw_target = None
					yaw_current = None		
					get_attitude()
					#ideal + offset
					yaw_target = drone_heading_current
					#actual + correction
					yaw_current = YAW + OFFSET
					if yaw_current > 180:
						yaw_current = -180 + OFFSET - (180 - YAW)
					print("[INFO] Drone Current Heading: " + str(drone_heading_current))
					print("[INFO] Drone Yaw: " + str(yaw_current))
						
					if yaw_current != None and yaw_target != None:
						correct_yaw(yaw_target, yaw_current, s)
					else:
						print("[WARN] Unable to correct yaw!")

					if drone_heading_current == 0 or drone_heading_current == 180:
						print("[INFO] Found end of aisle: performing course correction...")

						#GUARANTEE FULL DRONE STOP
						#send_command("stop",s)

						##COMPUTER VISION MODULE START##
						send_command('speed 10', s, 0)

						print ("[INFO] Start streaming")
						fw = 960
						fh = 720

						#PC WEBCAM FOR TESTING
						#capture = cv2.VideoCapture(0)

						#DRONE CAM FOR EXPERIMENTS PART1
						capture = cv2.VideoCapture ('udp://0.0.0.0:11111',cv2.CAP_FFMPEG)

						capture.set(3, fw)
						capture.set(4, fh)
						capture.set(10,300)

						#DRONE CAM FOR EXPERIMENTS PART2
						if not capture.isOpened():
							capture.open('udp://0.0.0.0:11111')


						global ENTERED_CV
						ENTERED_CV = True

						computer_vision_toggle_thread = threading.Thread(target=_computer_vision_toggle_thread)
						computer_vision_toggle_thread.daemon = True
						computer_vision_toggle_thread.start()


						#DRONE CAMERA CALIBRATION
						cv2.namedWindow("Calibration")
						cv2.resizeWindow("Calibration", 640, 400)
						cv2.createTrackbar("Hue Min", "Calibration", 0, 179, empty)
						cv2.createTrackbar("Hue Max", "Calibration", 11, 179, empty)
						cv2.createTrackbar("Sat Min", "Calibration", 158, 255, empty)
						cv2.createTrackbar("Sat Max", "Calibration", 255, 255, empty)
						cv2.createTrackbar("Val Min", "Calibration", 0, 255, empty)
						cv2.createTrackbar("Val Max", "Calibration", 255, 255, empty)
						cv2.createTrackbar("Threshold1","Calibration",90, 255, empty)
						cv2.createTrackbar("Threshold2","Calibration",255, 255, empty)


						cap_width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
						cap_height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
						cap_fps =  int(capture.get(cv2.CAP_PROP_FPS))
						#print(cap_width)
						#print(cap_height)
						#print(cap_fps)

						time.sleep(5)

						while ENTERED_CV == True:
							capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
							capture.set( cv2.CAP_PROP_FPS, 2 )
							corr_num = None

							cv2.VideoCapture.grab(capture)
							ret, frame = capture.read()
							#print(ret)


							if(ret):
								#CREATE CALIBRATION TRACKBARS
								h_min = cv2.getTrackbarPos("Hue Min", "Calibration")
								h_max = cv2.getTrackbarPos("Hue Max", "Calibration")
								s_min = cv2.getTrackbarPos("Sat Min", "Calibration")
								s_max = cv2.getTrackbarPos("Sat Max", "Calibration")
								v_min = cv2.getTrackbarPos("Val Min", "Calibration")
								v_max = cv2.getTrackbarPos("Val Max", "Calibration")

								#PROCESS FRAME INTO RESULT
								lower = np.array([h_min,s_min,v_min])
								upper = np.array([h_max,s_max,v_max])

								frame_HSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
								frame_mask = cv2.inRange(frame_HSV, lower, upper)
								frame_result = cv2.bitwise_and(frame, frame, mask=frame_mask)

								#CONTOUR DETECTION
								frame_gray = cv2.cvtColor(frame_result,cv2.COLOR_BGR2GRAY)
								frame_blurred = cv2.GaussianBlur(frame_gray,(7,7),1)
								#frame_blurred = cv2.bilateralFilter(frame_gray, 5, 10, 10)
								ret,thresh = cv2.threshold(frame_blurred,20,255,cv2.THRESH_BINARY)

								frame_blurred_bilateral = cv2.bilateralFilter(frame_result,9,75,75)
								frame_gray = cv2.cvtColor(frame_blurred_bilateral,cv2.COLOR_BGR2GRAY)
								threshold1 = cv2.getTrackbarPos("Threshold1","Calibration")
								threshold2 = cv2.getTrackbarPos("Threshold2","Calibration")
								frame_canny = cv2.Canny(frame_gray, threshold1, threshold2)
		
								kernel = np.ones((15,15))
								frame_dil = cv2.dilate(frame_canny, kernel, iterations=1)
								frame_contour = frame.copy()

								corr_num = get_contours(frame_dil, frame_contour, cap_width, cap_height, s)
								#get_contours(frame_dil, frame_contour, cap_width, cap_height, s)
								cv2.imshow("framecontour", frame_contour)

								
								
							if corr_num == 0:
								#send_command('stop', s)
								time.sleep(1)
								print("[INFO] COURSE CORRECTION DONE! EXITING...")
								break

							if cv2.waitKey (1)&0xFF == ord ('q'):
								break

						capture.release()
						cv2.destroyAllWindows()

						global TYPE0_LOCK
						global TYPE1_LOCK
						global TYPE2_LOCK
						global TYPE3_LOCK
						TYPE0_LOCK = False
						TYPE1_LOCK = False
						TYPE2_LOCK = False
						TYPE3_LOCK = False

						send_command('speed 20', s, 0)

						##COMPUTER VISION MODULE END##

						#RFID VALIDATION after position correction
						i = 0
						while i < 3:
							print("[INFO] Checking for RFID tag...")
							time.sleep(0.5)
							check = rfid_check(drone_position)
							i += 1
							if check == True:
								break
						if i == 3:
							print("[WARNING] Tag not found. Cannot confirm position. Continuing...")



				#DRONE MOVEMENT COMMANDS
				if (movement_vector[0] != movement_vector[1] and (movement_vector[0] == 0 or movement_vector[1] == 0)):
					#ORTHOGONAL CASE
					#NORTH-SOUTH movement
					if movement_vector[0] == 0:
						if movement_vector[1] < 0:
							#SOUTH
							if drone_heading_current != 180:
								rotation = (180 - drone_heading_current) % 360
								if rotation <= 180:
									send_command('cw ' + str(rotation), s, 5)
								else:
									send_command('ccw ' + str(rotation%180), s, 5)
								drone_heading_current = 180

						else:
							#NORTH
							if drone_heading_current != 0:
								rotation = (0 - drone_heading_current) % 360
								if rotation <= 180:
									send_command('cw ' + str(rotation), s, 5)
								else:
									send_command('ccw ' + str(rotation%180), s, 5)
								drone_heading_current = 0
						split_move(abs(movement_vector[1])*DRONE_DISTANCE_Y)
						
				
					#EAST-WEST movement
					elif movement_vector[1] == 0:
						if movement_vector[0] < 0:
							#WEST
							if drone_heading_current != 270:
								rotation = (270 - drone_heading_current) % 360
								if rotation <= 180:
									send_command('cw ' + str(rotation), s, 5)
								else:
									send_command('ccw ' + str(rotation%180), s, 5)
								drone_heading_current = 270

						else:
							#EAST
							if drone_heading_current != 90:
								rotation = (90 - drone_heading_current) % 360
								if rotation <= 180:
									send_command('cw ' + str(rotation), s, 5)
								else:
									send_command('ccw ' + str(rotation%180), s, 5)
								drone_heading_current = 90
						split_move(abs(movement_vector[0])*DRONE_DISTANCE_X)
						


				else:
					#DIAGONAL CASE
					#turn heading to North
					if drone_heading_current != 0:
						rotation = (0 - drone_heading_current) % 360
						send_command('cw ' + str(rotation), s, 5)
						drone_heading_current = 0

					coverage = vcm.traverse_order.index(shelf_level)
					send_command('up ' + str(((SHELF_HEIGHT - coverage) + 1)*DRONE_DISTANCE_Z), s, 5)

					#move diagonally to point
					angle_degrees = int(round(math.degrees(math.atan2(movement_vector[0]*DRONE_DISTANCE_Y, movement_vector[1]*DRONE_DISTANCE_X)),2))
					print(movement_vector[0])
					print(movement_vector[1])
					print(angle_degrees)
					hypotenuse = abs(math.hypot(movement_vector[0]*DRONE_DISTANCE_X, movement_vector[1]*DRONE_DISTANCE_Y))

					if abs(angle_degrees) >= 180:
						#cw angle
						if angle_degrees >= 0:
							send_command('ccw ' + str(abs(angle_degrees)), s, 5)
						else:
							send_command('cw ' + str(abs(angle_degrees)), s, 5)
						#forward hypotenuse
						split_move(int(hypotenuse))
						#ccw angle
						if angle_degrees >= 0:
							send_command('cw ' + str(abs(angle_degrees)), s, 5)
						else:
							send_command('ccw ' + str(abs(angle_degrees)), s, 5)

					else:
						#ccw 360 - angle
						if angle_degrees >= 0:
							send_command('cw ' + str(abs(angle_degrees)), s, 5)
						else:
							send_command('ccw ' + str(abs(angle_degrees)), s, 5)
						#forward hypotenuse
						split_move(int(hypotenuse))
						#cw 360 - angle
						if angle_degrees >= 0:
							send_command('ccw ' + str(abs(angle_degrees)), s, 5)
						else:
							send_command('cw ' + str(abs(angle_degrees)), s, 5)
			
					send_command('down ' + str(((SHELF_HEIGHT - coverage) + 1)*DRONE_DISTANCE_Z), s, 5)
				drone_position = nodemap[next_node]
		
		#move up one level
		send_command('up ' + str(DRONE_DISTANCE_Z), s, 5)

	send_command('land', s, 0)
	send_command('battery?', s, 0)

def send_command(command, s, t):
	"""
	Send a command to the ip address. Will be blocked until
	the last command receives an 'OK'.
	If the command fails (either b/c time out or error),
	will try to resend the command
	:param command: (str) the command to send
	:param ip: (str) the ip of Tello
	:return: The latest command response
	"""
	global RETRY
	log.append(Stats(command, len(log)))

	s.sendto(command.encode('utf-8'), tello_address)
	print ('[INFO] Sending command: %s => %s' % (command, tello_ip))

	start = time.time()
	while not log[-1].got_response():
		now = time.time()
		diff = now - start
		if diff > MAX_TIME_OUT:
			print ('[WARN] Max timeout exceeded for command: %s' % command)

			#land due to failure
			if RETRY >= 5:
				RETRY = 0
				print("[ERROR] NO RESPONSE AFTER RETRYING! LANDING DRONE FOR SAFETY...")
				send_command('land', s, 0)

			#repeat last command
			else:
				RETRY += 1
				print("[WARN] Retrying... " + str(RETRY))
				send_command(command, s, t)
			return

	print ('Done!!! sent command: %s to %s\n' % (command, tello_ip))
	RETRY = 0
	
	slist = ['forward', 'back', 'up', 'down', 'left', 'right', 'cw', 'ccw', 'takeoff', 'land']
	tlist = ['cw 1', 'ccw 1']
	#wait for drone to move before sending the next commmand
	for i in slist:
		if i == command.rsplit()[0]:
			time.sleep(t)
			pass

	#otherwise dont't wait for commands that don't require movement
	time.sleep(0.5)

def _receive_thread():
	"""Listen to responses from the Tello.

	Runs as a thread, sets self.response to whatever the Tello last returned.

	"""
	while True:
		try:
			response, ip = s.recvfrom(1024)
			print('from %s: %s' % (ip, response))
			if response == "error Not joystick":
				time.sleep(2)
				continue
			log[-1].add_response(response)
			if 'yaw' in response:
				global YAW
				YAW = int(response.split(";")[-2].split(":")[-1])
		except:
			print ("Connection ended.")
			break

def _anti_auto_land_thread():
	"""
	Stops the drone from timing out and automatically landing when processing
	the computer vision algorithm.
	"""
	global ENTERED_CV
	global ANTI_AUTO_LAND

	while True:
		if ENTERED_CV == True:
			if ANTI_AUTO_LAND == 0:
				send_command('ccw 1', s, 0)
				if ENTERED_CV == False:
					continue
				send_command("stop", s, 0)
				if ENTERED_CV == False:
					continue
				time.sleep(9)
				ANTI_AUTO_LAND = 1
			elif ANTI_AUTO_LAND == 1:
				send_command('ccw 1', s, 0)
				if ENTERED_CV == False:
					continue
				send_command("stop", s, 0)
				if ENTERED_CV == False:
					continue
				time.sleep(9)
				ANTI_AUTO_LAND = 0
		#time.sleep(9)

def _computer_vision_toggle_thread():
	"""
	Regulates the timeout for the computer vision logic to operate under.
	"""
	#after interval, stop CV
	global ENTERED_CV
	if ENTERED_CV == True:
		time.sleep(30)
		ENTERED_CV = False

def _drone_yaw_fix_thread():
	"""
	Partially corrects the drone rear-right motor issue causng involuntary clockwise yawing.
	"""

	#COMMENT THIS THREAD AND ITS INITIALIZATION FOR YOUR DRONE IN PARTICULAR.
	start = time.time()
	global TURN
	global TURN_FIX
	global OFFSET

	#	0.208 degrees per second clockwise rotation error
	#	might be overcorrecting!!!
	#	try values between 1.800 <-> 0.208?
	while True:
		#if turn isn't issued
		if TURN == True:
			#add a turn by x amount to the next turn command
			TURN = False
			print("TURN FIX BEFORE: " + str(TURN_FIX))
			TURN_FIX = int(0.15 * (time.time() - start))
			OFFSET += TURN_FIX
			print("TURN FIX AFTER: " + str(TURN_FIX))
			print("OFFSET: " + str(OFFSET))
			start = time.time()


#NAVIGATION COMPONENT INITIALIZATIONS--------------------

"""
This is the second component to be started. Python 2 is required.
Once the message "THE DRONE CAN NOW BE LAUNCHED!" is printed, the navigation component can be started.
"""

vcm = VCM(SHELF_HEIGHT, CURRENT_POS)

os.chdir('..')
connection = sqlite3.connect('tags.db')
cursor = connection.cursor()

node_list = cursor.execute("SELECT * FROM nodes")
for node in node_list.fetchall():
	vcm.add_node(node[0].encode('utf-8'), node[1], node[2])

#ADJUST PERIMETER BY UNCOMMENTING AND ADJUSTING THIS FUNCTION
#vcm.set_perimeter((-1,-1),(1,1))

#movement variable initializations
nodemap = vcm.get_nodemap()
vcm.route_3d()
print(vcm.traverse_order)
#NAVIGATION COMPONENT INITIALIZATIONS--------------------


#TEST WITH DRONE: INITIALIZATIONS------------------------
tello_ip = '192.168.10.1'
tello_port = 8889
tello_address = (tello_ip, tello_port)
mypc_address = ('', 8889)
s = socket.socket (socket.AF_INET, socket.SOCK_DGRAM)
s.bind (mypc_address)
s.sendto ('command'.encode (' utf-8 '), tello_address)
s.sendto ('streamon'.encode (' utf-8 '), tello_address)

response = None
log = []

receive_thread = threading.Thread(target=_receive_thread)
receive_thread.daemon = True
receive_thread.start()

anti_auto_land_thread = threading.Thread(target=_anti_auto_land_thread)
anti_auto_land_thread.daemon = True
anti_auto_land_thread.start()

#EXPERMENTAL DRONE MALFUNCTION YAW CORRECTION
#COMMENT FOR REGULAR USE WITH YOUR PARTICULAR DRONE
#drone_yaw_fix_thread = threading.Thread(target=_drone_yaw_fix_thread)
#drone_yaw_fix_thread.daemon = True
#drone_yaw_fix_thread.start()



#drone_camera_thread = threading.Thread(target=_drone_camera_thread)
#drone_camera_thread.daemon = True
#drone_camera_thread.start()
#TEST WITH DRONE: INITIALIZATIONS------------------------



movement_controller(vcm, nodemap, s)
	
print("JOB'S DONE!")
	
#if __name__ == "__main__":
#	main()












##IMPORTANT: IF RESPONSE == r"error Not joystick"
##SEND AGAIN
##https://github.com/dji-sdk/Tello-Python/blob/master/Single_Tello_Test/tello.py


#def split_move(mov):
#	send_command('forward ' + str(mov), s)
	#try:
	#	t = get_attitude()
	#	print("Attitude:")
	#	print(int(t.split(";")[-2].split(":")[-1]))
	#except:
	#	print("Failed to get attitude")
	#print(mov)
	#while mov > 0:
	#	send_command('forward ' + str(100), s)
	#	mov -= 100
	#	if mov < 100:
	#		send_command('forward ' + str(mov), s)
	#		mov = 0


