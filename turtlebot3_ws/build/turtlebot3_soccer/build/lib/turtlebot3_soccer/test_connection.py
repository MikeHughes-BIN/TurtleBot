import socket

# Create a simple client on Raspberry Pi
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect(('10.1.1.1', 12345))  # Jetson Nano's IP

client_socket.sendall(b'Hello, Jetson Nano!')
client_socket.close()
