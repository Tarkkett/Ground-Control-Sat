import socket
from time import sleep

HOST = '127.0.0.1'  # Standard loopback interface address (localhost)
PORT = 65432        # Port to listen on (non-privileged ports are > 1023)

# Create a socket object
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
    # Bind the socket to the address and port
    server_socket.bind((HOST, PORT))

    # Listen for incoming connections
    server_socket.listen()
    print(f"Server is listening on {HOST}:{PORT}")

    # Accept incoming connections
    client_socket, client_address = server_socket.accept()
    print(f"Connection from {client_address} has been established.")

    # Receive and print data from client continuously
    while True:
        data = client_socket.recv(1024)
        if not data:
            print("no data")
            sleep(1)
            break
            
        print(f"Received data from client: {data.decode()}")

    # Close the connection
    client_socket.close()
