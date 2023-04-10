import socket
from socket import gethostbyname
import select
import time

#HOST = 'localhost'
HOST = gethostbyname('0.0.0.0')
PORT = 65439

ACK_TEXT = 'text_received'

def main():
    print('Initializing Socket...')
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)    # instantiate a socket object
    print('Binding To Socket...')
    sock.bind((HOST, PORT))                                     # bind the socket
    sock.listen()                                               # start the socket listening
    print('Listening To Socket...')

    # accept the socket response from the client, and get the connection object
    conn, addr = sock.accept()      # Note: execution waits here until the client calls sock.connect()
    print('Socket Connection Accepted, Received Connection Object')

    while True:
        message = input("Message to send: ")
        print('Sending: ' + message)
        sendTextViaSocket(message, conn)
    return

def sendTextViaSocket(message, sock):
    encodedMessage = bytes(message, 'utf-8')        # encode the text message
    sock.sendall(encodedMessage)                    # send the data via the socket to the server
    encodedAckText = sock.recv(1024)                # receive acknowledgment from the server
    ackText = encodedAckText.decode('utf-8')

    # log if acknowledgment was successful
    if ackText == ACK_TEXT:
        print('Server acknowledged reception')
    else:
        print('Error: Server returned ' + ackText)
    
    return

if __name__ == '__main__':
    main()