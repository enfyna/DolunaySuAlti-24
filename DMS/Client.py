from cv2 import imdecode, IMREAD_COLOR
from numpy import frombuffer, uint8
from json import loads, dumps
from base64 import b64decode
from time import sleep
import socket

class SimClient():
	def __init__(self):
		self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.client_socket.connect(("127.0.0.1", 12345))
		print('Connected')

	def recv(self):
		buffer : str = ''
		while True:
			recv = self.client_socket.recv(1024)
			if recv == b'':
				self.close()
				raise ConnectionAbortedError()
			recv = recv.decode('ASCII')
			buffer += recv
			try:
				data : dict = loads(f"{{{buffer.split('{')[1].split('}')[0]}}}")
				keys = data.keys()
				# print(f'Received data : {keys}')

				if 'cam_1' in keys:
					im_bytes = b64decode(data['cam_1'])
					im_arr = frombuffer(im_bytes, dtype=uint8)
					data['cam_1'] = imdecode(im_arr, flags=IMREAD_COLOR)
				
				if 'cam_2' in keys:
					im_bytes = b64decode(data['cam_2'])
					im_arr = frombuffer(im_bytes, dtype=uint8)
					data['cam_2'] = imdecode(im_arr, flags=IMREAD_COLOR)

				buffer = ''
				return data
			except:
				# print(f"Received buffer from Godot")
				continue				

	def SendData(self, data : dict):
		data_to_send = dumps(data)
		self.client_socket.send(data_to_send.encode('ASCII'))
		return

	def close(self):
		self.client_socket.close()