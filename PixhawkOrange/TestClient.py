import cv2
import numpy as np
import ClientGroundStation as cgs
import PixhawkOrange



arac = PixhawkOrange.Dolunay()

cap1 = cv2.VideoCapture(0)
cap2 = cv2.VideoCapture(1)

# socket settings
connStation = cgs.ClientConn("169.254.17.41", 65432,arac)
arac.set_arm(1)

while cap1.isOpened() and cap2.isOpened():
        try:
            ret1, frame1 = cap1.read()
            ret2, frame2 = cap2.read() 
            assert ret1
            assert ret2
            frame1 = cv2.resize(frame1, (200,200))
            connStation.setFrontCam(frame1)
            frame2 = cv2.resize(frame2, (200,200))
            connStation.setUnderCam(frame2)
            connStation.setPixhawkData(arac.getData())
            connStation.sendAllData()
            connStation.receiveData()
        except KeyboardInterrupt:
            cap1.release()
            cap2.release()
            connStation.close()
        except Exception as e:
            cap1.release()
            cap2.release()
            connStation.close()
            print(e)
            break  
        if cv2.waitKey(1) == ord('q'):
            break   
cap1.release()
cap2.release()
connStation.close()

