import ftplib
import time

def main():
	while True:
		#init session
		#EDIT TO SET A HOST
		session = ftplib.FTP("localhost", "test", "1", timeout=5.0)
		#open file to send (READ BINARY mode)
		file = open("tags.db", 'rb')

		try:
			#send file in binary transfer mode
			session.storbinary("STOR tags.db", file)
			print("Successfully connected to Warehouse DB, sent local DB!")

			file.close()
			session.quit()
			
			time.sleep(10)
		except:

			file.close()
			session.quit()

			print("Failed to connect to Warehouse DB, retrying in 10 seconds ...")
			time.sleep(10)

if __name__ == '__main__':
	"""
	Main logic: whenever possible, sends the tags.db file through FTP
	"""
	main()