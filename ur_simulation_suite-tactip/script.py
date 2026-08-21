# Script to send string over port 127.0.0.1:29999

import socket

HOST = "127.0.0.1"  # The server's hostname or IP address
PORT = 29999  # The port used by the server

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))
    s.sendall(b"load robot_interaction_lesson_using_file.urp ")
    data = s.recv(1024)
    print(f"Received {data!r}")

    s.sendall(b"play")
    data = s.recv(1024)
    print(f"Received {data!r}")
    
