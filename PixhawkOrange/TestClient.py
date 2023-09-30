import cv2
import numpy as np
import GetDataFromPixhawk as dataPX
import ClientGroundStation as cgs


cap1 = cv2.VideoCapture(0)
cap1.set(3, 480)
cap1.set(4, 320)
cap1.set(cv2.CAP_PROP_FPS, 60) 
# if you have second camera ,uncomment
#cap2 = cv2.VideoCapture(1)
#cap2.set(3, 480)
#cap2.set(4, 320)
#cap2.set(cv2.CAP_PROP_FPS, 60) 

#if connected pixhawk ,uncomment
#port="/dev/ttyACM0" or different end number for linux
#port="COM4" or different end number for windows
#master = mavutil.mavlink_connection(port)

# socket settings
connStation = cgs.ClientConn("localhost", 65432)

while cap1.isOpened():
        try:
            ret1, frame1 = cap1.read()
            #ret2, frame2 = cap2.read() 
            assert ret1
            #assert ret2
            connStation.setFrontCam(frame1)
            #connStation.setUnderCam(frame2)
            #connStation.setPixhawkData(dataPX.getData(master))
            connStation.sendAllData()
        except KeyboardInterrupt:
            cap1.release()
            #cap2.release()
            connStation.close()
        except Exception as e:
            #cap2.release()
            connStation.close()
            print(e)
            break
            #cap2.release()     
        if cv2.waitKey(1) == ord('q'):
            break   
cap1.release()
#cap2.release()
connStation.close()

