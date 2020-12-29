import sqlite3
import os

global buffer_tags
buffer_tags = []
#LOAD DATABASE
print("loading item DB...")
connection = sqlite3.connect('tags.db')
cursor = connection.cursor()

def insert_item(epc, item_count, item_posx, item_posy, item_posz, flag):
	"""
	Inserts of updates item table records.
	"""
	cursor.execute("SELECT * FROM items WHERE epc = :epc_sel", {'epc_sel':epc,})
	sel = cursor.fetchall()
	with connection:
		if len(sel) == 0:
			cursor.execute("INSERT INTO items VALUES (:epc, :item_count, :item_posx, :item_posy, :item_posz, :flag)",
				 {'epc': epc, 'item_count': item_count, 'item_posx': item_posx, 'item_posy': item_posy, 'item_posz': item_posz, 'flag': flag})
			print("[INFO] Added new item to DB.")
		else:
			print("[INFO] Item already in DB, updating its record.")
			if ((sel[0][2], sel[0][3], sel[0][4]) != (item_posx, item_posy, item_posz)) and flag != 1:
				flag = 1
			cursor.execute("UPDATE items SET item_count = :item_count, item_posx = :item_posx, item_posy = :item_posy, item_posz = :item_posz, flag = :flag WHERE epc = :epc_sel",
				  {'item_count': item_count, 'item_posx': item_posx, 'item_posy': item_posy, 'item_posz': item_posz, 'flag': flag, 'epc_sel':epc})

def update_local_db(st):
	"""
	Parses a string containing information on an item tag.
	"""
	epc = st[-96:-16]
	item_count = int(st[-16:-12])
	if st[-12] == '0':
		item_posx = int(st[-11:-9])
	else:
		item_posx = -int(st[-11:-9])
	if st[-9] == '0':
		item_posy = int(st[-8:-6])
	else:
		item_posy = -int(st[-8:-6])
	item_posz = int(st[-6:-3])
	flag = int(st[-3])

	insert_item(epc, item_count, item_posx, item_posy, item_posz, flag)

	cursor.execute("SELECT * FROM items WHERE epc = :epc_sel", {'epc_sel':epc,})
	print(cursor.fetchall())