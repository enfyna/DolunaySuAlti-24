import socket
import os
import base64
import json
import cv2
import time
import JoystickController
class ClientConn():
    def __init__(self,port,vehicle,host=''):
        """
        host : (Name of computer or ip adress computer)("localhost")
        port : (Port used by the server)("65432")
        """
        if host == '':
            host = self.readIP()
        print(host)
        self.vehicle = vehicle
        self.host = host
        self.port = port
        self.data = {}
        self.isConnected=True
        self.isSended=True
        self.isFrontCamSended=True
        self.isUnderCamSended=True
        self.isPixhawkSended=True
        self.isDistanceSended=True
        self.isHydrophoneSended=True
        self.connect()
    def readIP(self):
        try:
            with open(os.path.expanduser('~')+'/ipaddress.txt') as file:
                ip = file.readline()
                ip = ip.strip()
                return ip
        except Exception as e:
            print("IP not found")
            return 'localhost'
    def connect(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
           self.sock.settimeout(1)
           self.sock.connect((self.host, self.port))
           print("Connected.")
           self.isConnected=True
        except Exception:
            if(self.isConnected):
                print("Connection close.")
            self.isConnected=False

    def codeImage(self,frame):
        _, encoded_frame = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 60])
        framebase64 = base64.b64encode(encoded_frame.tobytes()).decode()
        return framebase64

    def addData2Json(self,key,value):
        self.data[key] = value

    def setHydrophoneData(self,hydrodata):
        try:
            json_data = json.dumps(hydrodata)
            self.addData2Json("hydrophone", json_data)
            self.isHydrophoneSended=True
        except:
            if self.isHydrophoneSended:
                print("Error: Hydrophone data not added.Check hydrophone sensor.")
                self.isHydrophoneSended=False

    def setDistanceData(self,disdata):
        try:
            json_data = json.dumps(disdata)
            self.addData2Json("distance", json_data)
            self.isDistanceSended=True
        except:
            if self.isDistanceSended:
                print("Error: Distance data not added.Check distance sensor.")
                self.isDistanceSended=False


    def setPixhawkData(self,pixhawkdata):
        try:
            self.addData2Json("pixhawkdata", pixhawkdata)
            self.isPixhawkSended=True
        except:
            if self.isPixhawkSended:
                print("Error: Pixhawk data not added.Check pixhawk.")
                self.isPixhawkSended=False


    def setFrontCam(self,frame):
        try:
            self.addData2Json("cam1",self.codeImage(frame))
            self.isFrontCamSended=True
        except:
            if self.isFrontCamSended:
                print("Error: Front Cam not added.Check camera.")
                self.isFrontCamSended=False

    def setUnderCam(self,frame):
        try:
            self.addData2Json("cam2",self.codeImage(frame))
            self.isUnderCamSended=True
        except:
            if self.isUnderCamSended:
                print("Error: Under Cam not added.Check camera.")
                self.isUnderCamSended=False

    def sendAllData(self):
        json_data = json.dumps(self.data)
        try:
            self.sock.send(json_data.encode())
            self.isSended=True
        except (BrokenPipeError, ConnectionResetError):
            if(self.isSended):
                print("Server closed.")
                self.isConnected=False
                self.isSended=False
            else:
                self.connect()
        except Exception as e:
                print(e)
                self.connect()

    def receiveData(self):
        response = self.sock.recv(1024)
        if not response:
            return
        try:

            if(len(response)<100 and response.decode('utf-8').split("(")[1].split(")")[0]=="received"):
                pass
            else:
                JoystickController.joystickControl(response.decode(), self.vehicle)
        except Exception as e:
           pass
           #print("HATA :"+ str(e))

    def close(self):
        self.sock.close()