import socket
from socket import gethostbyname
import select
import time

#HOST = 'localhost'
HOST = gethostbyname('0.0.0.0')
PORT = 65439

ACK_TEXT = 'text_received'

def main():
    # instantiate a socket object
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    print('socket instantiated')

    # bind the socket
    sock.bind((HOST, PORT))
    print('socket binded')

    # start the socket listening
    sock.listen()
    print('socket now listening')

    # accept the socket response from the client, and get the connection object
    conn, addr = sock.accept()      # Note: execution waits here until the client calls sock.connect()
    print('socket accepted, got connection object')

    myCounter = 0
    while True:
        message = 'message ' + str(myCounter)
        print('sending: ' + message)
        sendTextViaSocket(message, conn)
        myCounter += 1
        time.sleep(1)
    # end while
# end function

def sendTextViaSocket(message, sock):
    # encode the text message
    encodedMessage = bytes(message, 'utf-8')

    # send the data via the socket to the server
    sock.sendall(encodedMessage)

    # receive acknowledgment from the server
    encodedAckText = sock.recv(1024)
    ackText = encodedAckText.decode('utf-8')

    # log if acknowledgment was successful
    if ackText == ACK_TEXT:
        print('server acknowledged reception of text')
    else:
        print('error: server has sent back ' + ackText)
    # end if
# end function

if __name__ == '__main__':
    main()