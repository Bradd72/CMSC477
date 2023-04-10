import socket
import select
import time

#HOST = 'localhost'
HOST = '10.104.68.209'
PORT = 65439

ACK_TEXT = 'text_received'

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)        # instantiate a socket object
    print('Socket Instantiated')

    # connect the socket
    connectionSuccessful = False
    while not connectionSuccessful:
        try:
            sock.connect((HOST, PORT))    # Note: if execution gets here before the server starts up, this line will cause an error, hence the try-except
            print('Socket Connected')
            connectionSuccessful = True
        except:
            pass

    socks = [sock]
    while True:
        readySocks, _, _ = select.select(socks, [], [], 5)
        for sock in readySocks:
            message = receiveTextViaSocket(sock)
            print('Received: ' + str(message))

    return

def receiveTextViaSocket(sock):
    encodedMessage = sock.recv(1024)        # get the text via the scoket

    # if we didn't get anything, log an error and bail
    if not encodedMessage:
        print('Error: encodedMessage was received as None')
        return None

    message = encodedMessage.decode('utf-8')        # decode the received text message

    # send acknowledgement
    encodedAckText = bytes(ACK_TEXT, 'utf-8')       # encode the acknowledgement text
    sock.sendall(encodedAckText)                    # send the encoded acknowledgement text

    return message

if __name__ == '__main__':
    main()