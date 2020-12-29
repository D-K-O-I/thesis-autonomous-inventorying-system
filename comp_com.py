#NODE FUNCTIONS
def insert_node(epc, x, y):
	sel = cursor.execute("SELECT * FROM nodes WHERE epc = :epc_sel", {'epc_sel':epc,})
	with connection:
		if len(sel.fetchall()) == 0:
			cursor.execute("INSERT INTO nodes VALUES (:epc, :x, :y)", {'epc': epc, 'x': x, 'y': y})
			print("[INFO] Added new node to DB.")
		else:
			print("[INFO] Node already in DB, updating its record.")
			cursor.execute("UPDATE nodes SET x = :x, y = :y WHERE epc = :epc = epc_sel", {'x': x, 'y': y, 'epc_sel': epc})

def remove_node(epc_rem):
	sel = cursor.execute("SELECT * FROM nodes WHERE epc = :epc_sel", {'epc_sel':epc_rem,})
	with connection:
		if not len(sel.fetchall()) == 0:
			cursor.execute("DELETE FROM nodes WHERE epc = :epc_rem", {'epc_rem':epc_rem,})
			print("[INFO] Node removed.")
		else:
			print("[ERROR] Node not found.")

def get_nodes():
	cursor.execute("SELECT * FROM nodes")
	print(cursor.fetchall())


#ITEM FUNCTIONS
def insert_item(epc, item_count, item_posx, item_posy, item_posz, flag):
	sel = cursor.execute("SELECT * FROM items WHERE epc = :epc_sel", {'epc_sel':epc,})
	with connection:
		if len(sel.fetchall()) == 0:
			cursor.execute("INSERT INTO items VALUES (:epc, :item_count, :item_posx, :item_posy, :item_posz, :flag)",
				 {'epc': epc, 'item_count': item_count, 'item_posx': item_posx, 'item_posy': item_posy, 'item_posz': item_posz, 'flag': flag})
			print("[INFO] Added new item to DB.")
		else:
			print("[INFO] Item already in DB, updating its record.")
			cursor.execute("UPDATE items SET item_count = :item_count, item_posx = :item_posx, item_posy = :item_posy, item_posz = :item_posz, flag = :flag WHERE epc = :epc_sel",
				  {'item_count': item_count, 'item_posx': item_posx, 'item_posy': item_posy, 'item_posz': item_posz, 'flag': flag, 'epc_sel':epc})


def remove_item(epc_rem):
	sel = cursor.execute("SELECT * FROM items WHERE epc = :epc_sel", {'epc_sel':epc_rem,})
	with connection:
		if not len(sel.fetchall()) == 0:
			cursor.execute("DELETE FROM items WHERE epc = :epc_rem", {'epc_rem':epc_rem,})
			print("[INFO] Item removed.")
		else:
			print("[ERROR] Item not found.")

def get_items():
	cursor.execute("SELECT * FROM items")
	print(cursor.fetchall())

def get_item_by_epc(epc):
	cursor.execute("SELECT * FROM items WHERE epc = :epc", {'epc':epc})
	print(cursor.fetchall())

def get_items_in_perimeter(lower, upper):
	cursor.execute("""SELECT * FROM items WHERE
						(	item_posx >= :lowerx
						AND item_posy >= :lowery
						AND item_posx <= :upperx
						AND item_posy <= :uppery
						)""", {'lowerx':lower[0],'lowery':lower[1],'upperx':upper[0],'uppery':upper[1]})
	print(cursor.fetchall())

def get_items_highest_count():
	cursor.execute("SELECT * FROM items ORDER BY item_count DESC")
	print(cursor.fetchall())

def get_flagged_items():
	cursor.execute("SELECT * FROM items WHERE flag = 1")
	print(cursor.fetchall())

def flush_items():
	cursor.execute("DELETE FROM items")

#get_nodes()
#get_items()
#get_item_by_epc('1')
#get_items_in_perimeter((0,1),(1,1))
#get_items_highest_count()

if __name__ == '__main__':
	"""
	This is the first component to be started. Python 3 is required.
	It has a sample warehouse layout loaded onto the node table. Please adjust it according to your necessity.
	The FTP is currently disabled. Re-enable it by uncommenting its instancing.
	Once the message "THE DRONE CAN NOW BE LAUNCHED!" is printed, the navigation component can be started.
	"""

	import sqlite3
	import os
	import time
	import subprocess
	import multiprocessing
	import rfid_interface

	#retval = os.getcwd()
	#print ("Current working directory %s" % retval)
	#os.chdir('.')
	#retval = os.getcwd()
	#print ("Current working directory %s" % retval)

	connection = sqlite3.connect('tags.db')
	cursor = connection.cursor()

	#TABLE INITIALIZATION
	cursor.execute("SELECT COUNT(*) FROM sqlite_master WHERE type='table' AND name='nodes'")
	l=cursor.fetchall()
	if l[0][0] == 0:
		cursor.execute("CREATE TABLE nodes (epc TEXT PRIMARY KEY NOT NULL, x INT NOT NULL, y INT NOT NULL)")
	else:
		print("[INFO] Using existing table for nodes")

	cursor.execute("SELECT COUNT(*) FROM sqlite_master WHERE type='table' AND name='items'")
	l=cursor.fetchall()
	if l[0][0] == 0:
		cursor.execute("CREATE TABLE items (epc TEXT PRIMARY KEY NOT NULL, item_count INT NOT NULL, item_posx INT NOT NULL, item_posy INT NOT NULL, item_posz INT NOT NULL, flag INT NOT NULL)")
	else:
		print("[INFO] Using existing table for items")



	#START USB CAPTURE------------|
	process_livebuffer = subprocess.Popen('tshark -i 12 -w livebuffer.pcap', shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

	#UNCOMMENT FOR FTP
	#import ftptest
	#mprocess_ftp = multiprocessing.Process(target=ftptest.main)
	#mprocess_ftp.daemon = True
	#mprocess_ftp.start()

	#CONTROL CENTER---------------|
	while True:
		time.sleep(1)
		inp = input("Select an option (to view options, enter [h]): ")

		if inp == 'h':
			print("OPTIONS:")
			print("[0] Launch Drone")
			print("[1] Get Nodes")
			print("[2] Get Items")
			print("[3] Add Item <EPC> <COUNT> <POSX> <POSY> <POSZ> <FLAG>")
			print("[4] Rem Item <EPC>")
			print("[5] Add Node <EPC> <POSX> <POSY>")
			print("[6] Rem Node <EPC>")
			print("[7] Get flagged items")
			print("[8] Flush local item database")
			print("[x] Exit")

		elif inp == '0':
			print("Initializing...")
			##rfid_interface.interface = True
			rfid_q = multiprocessing.Queue(maxsize=1)
			sig_stop = multiprocessing.Queue(maxsize=1)
			mprocess_interface = multiprocessing.Process(target=rfid_interface.main, args=(rfid_q, sig_stop))
			mprocess_interface.daemon = True
			mprocess_interface.start()


			print("alive(): " + str(mprocess_interface.is_alive()))
			sig_stop.put(True)
			#time.sleep(30)

			#mprocess_compnav = multiprocessing.Process(target=os.system('py -2 component_navigation/comp_nav.py'))
			#mprocess_compnav.start()
			try:
				while True:#mprocess_compnav.is_alive():
					time.sleep(2)
					try:
						tag = rfid_q.get(block=False)
						f = open('tag.txt', 'w')
						f.write(str(tag))
						f.close()
					except:
						pass
			except KeyboardInterrupt:
				print('interrupted!')
			print("t4")
			sig_stop.put(False)
			print("t5")
			mprocess_interface.join()
			print("t6")
			print("alive(): " + str(mprocess_interface.is_alive()))
			
		elif inp == '1':
			get_nodes()
		elif inp == '2':
			get_items()
		elif inp == '3':
			x = input("Add Item <EPC> <COUNT> <POSX> <POSY> <POSZ> <FLAG>:")
			i0,i1,i2,i3,i4,i5 = x.split(' ')
			insert_item(i0,int(i1),int(i2),int(i3),int(i4),int(i5))
		elif inp == '4':
			x = input("Rem Item <EPC>:")
			remove_item(x)
		elif inp == '5':
			x = input("Add Node <EPC> <POSX> <POSY>:")
			i0,i1,i2 = x.split(' ')
			insert_node(i0,int(i1),int(i2))
		elif inp == '6':
			x = input("Rem Node <EPC>:")
			remove_node(x)
		elif inp == '7':
			get_flagged_items()
		elif inp == '8':
			flush_items()
		elif inp == 'x':
			break

	#CONTROL CENTER---------------|	

	process_livebuffer.terminate()
	time.sleep(2)
	connection.commit()
	connection.close()
