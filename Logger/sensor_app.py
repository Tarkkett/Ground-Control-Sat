from flask import Flask, render_template
from flask_socketio import SocketIO
from random import random
from threading import Lock, Thread
from datetime import datetime
import socket
from time import sleep
import ast


app = Flask(__name__)
app.config['SECRET_KEY'] = 'donsky!'
socketio = SocketIO(app, cors_allowed_origins='*')

HOST = '127.0.0.1'
PORT = 25002

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket = None

threading = True

def get_current_datetime():
    now = datetime.now()
    return now.strftime("%m/%d/%Y %H:%M:%S")

@app.route('/')
def index():
    return render_template('index.html')

@socketio.on('connect')
def connect():
    global threading
    tMain = Thread(target=background_thread, daemon=True)
    if tMain is not None:
        threading = True
        tMain.start()

@socketio.on('disconnect')
def disconnect():
    global threading
    threading = False
    print('Client disconnected')

def background_thread():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        server_socket.bind((HOST, PORT))
        server_socket.listen()
        client_socket, client_address = server_socket.accept()
        print("Socket accepted!")
    except Exception as e:
        print(e)
    
    with client_socket:
        while threading:
            print("Threading")
            sleep(0.1)
            
            
            sensorMsg = client_socket.recv(1024).decode()
            parsed_message = ast.literal_eval(sensorMsg)


            message_array = list(parsed_message)
            print(message_array)
            #Temp,                       pressure,                  humidity,                   servo x,                servo y,                v vel,                    altitude,                   g,                          UVB
            #{self.data.parsedMsg[5]},{self.data.parsedMsg[7]},{self.data.parsedMsg[6]},{self.data.parsedMsg[10]},{self.data.parsedMsg[11]},{self.data.parsedMsg[12]},{self.data.parsedMsg[9]},{self.data.parsedMsg[14]},{self.data.parsedMsg[13]}
            socketio.emit('updateTempSensorData', {'value': message_array[0], "date": get_current_datetime()})
            socketio.emit('updatePressureSensorData', {'value': message_array[1], "date": get_current_datetime()})
            socketio.emit('updateHumiditySensorData', {'value': message_array[2], "date": get_current_datetime()})
            socketio.emit('updateServoSensorData', {'valuex': message_array[3], 'valuey': message_array[4], "date": get_current_datetime()})
            socketio.emit('updateVerticalVelSensorData', {'value': message_array[5], "date": get_current_datetime()})
            socketio.emit('updateAltitudeSensorData', {'value': message_array[6], "date": get_current_datetime()})
            socketio.emit('updateGravSensorData', {'value': message_array[7], "date": get_current_datetime()})
            socketio.emit('updateUVBSensorData', {'value': message_array[8], "date": get_current_datetime()})



        

if __name__ == '__main__':
    socketio.run(app)