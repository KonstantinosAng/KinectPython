import socket
import sys

target = socket.gethostbyname(sys.argv[1])
port = int(sys.argv[2])
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
socket.setdefaulttimeout(1)
result = s.connect_ex((target, port))
if result == 0:
	print("Port {} is open".format(port))
	s.close()
else:
	print("Port {} is closed".format(port))