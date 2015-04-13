import socket 
from sh import nc


"""while True:
	output = nc('-ulv','192.168.17.150','61557') 
	print output"""


UDP_IP = "192.168.16.57"
BOT_IP = "192.168.17.150"
#BOT_IP = "192.168.16.52"
UDP_PORT = 61557

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))

while True:
	data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
	print "received message:", data
