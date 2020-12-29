import pyshark
import time
import sqlite3

translation_table = {}

def insert_tl(tt, hexcode, symbol):
	tt[hexcode] = symbol

insert_tl(translation_table,r'e5:04',r'A')
insert_tl(translation_table,r'e5:05',r'B')
insert_tl(translation_table,r'e5:06',r'C')
insert_tl(translation_table,r'e5:07',r'D')
insert_tl(translation_table,r'e5:08',r'E')
insert_tl(translation_table,r'e5:09',r'F')
insert_tl(translation_table,r'1e:00',r'1')
insert_tl(translation_table,r'1f:00',r'2')
insert_tl(translation_table,r'20:00',r'3')
insert_tl(translation_table,r'21:00',r'4')
insert_tl(translation_table,r'22:00',r'5')
insert_tl(translation_table,r'23:00',r'6')
insert_tl(translation_table,r'24:00',r'7')
insert_tl(translation_table,r'25:00',r'8')
insert_tl(translation_table,r'26:00',r'9')
insert_tl(translation_table,r'27:00',r'0')
insert_tl(translation_table,r'00:00',r'')


#tshark -r test.pcap -2 -R "usb.data_len==8 and usb.src!=host and frame.time_relative > 0.0 and usb.usbd_status==0x00000000"
def get_cap():
	"""
	Filters livebuffer.pcap for USB packets with tag information.
	"""
	time.sleep(5)
	c = pyshark.FileCapture(input_file = r'livebuffer.pcap', display_filter='usb.data_len==8 and usb.src!=host and frame.time_relative > 0.0 and usb.usbd_status==0x00000000')
	print(len(c))
	return c

def update_frame_count(c):
	"""
	Keeps track of the frame counter.
	"""
	fc = 0
	for fr in c:
		fc += 1
	return fc


def main(rfid_q, sig_stop):
	"""
	Main logic: instances the inventorying component, reads livebuffer.pcap and parses new USB packets.
	Packets are decoded using the translation table.
	"""
	from component_inventorying import comp_inv
	#filter for livebuffer.pcap:
	cap = get_cap()
	frame_number = 0
	frame_count = update_frame_count(cap)
	print(cap)
	#print(frame_count)

	tags = []
	st = ''
	#print("waiting for true")
	interface = sig_stop.get()
	#print("got a " + str(interface))
	
	print("THE DRONE CAN NOW BE LAUNCHED!")
	
	while interface == True:
		#print("updating...")
		try:
			interface = sig_stop.get(block=False)
		except:
			pass
			#print("queue empty, keep going...")
		print(interface)
		#print("LENGTH: " + str(len(cap)))
		#print(frame_number)
		print(tags)
		if (frame_number + 1 > frame_count):
			print("[INFO] Waiting for more tags...")
			
			try:
				cap.close()
			except:
				print("[INFO] During TRY:cap.close() - Cap already closed!")
			
			time.sleep(1)

			try:
				cap = get_cap()
			except:
				print("[ERROR] During TRY: get_cap()")
				continue

			frame_count = update_frame_count(cap)
			print("Frame count: " + str(frame_count))
		
			if len(cap) == 0:
				continue
		st += translation_table[cap[frame_number].layers[1].usb_capdata[6:11]]
		frame_number += 1

		if frame_number%96 == 0:
			if st[-1] == 'A':
				#if item
				comp_inv.update_local_db(st[-96:])
			if st[-1] == 'B':
				#if node
				rfid_q.put(st[-16:])
				pass
			if st[-16:] not in tags:
				tags.append(st[-16:])
			st = ''
		
	#process_livebuffer.terminate()
	print(tags)

#A
#ITEM TAGS
#0000000000000000
#CCCCXXXYYYZZZF T

#B
#NODE TAGS
#0000000000000000
#XXXYYY         T