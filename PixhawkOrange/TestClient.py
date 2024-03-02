import cv2
import numpy as np
import ClientGroundStation as cgs
import Dolunay

arac = Dolunay.Dolunay()
arac.Pixhawk.set_arm(True)

# socket settings
connStation = cgs.ClientConn(65432, arac)

while True:
	try:
		if arac.Camera.is_front_cam_open():
			ret1, frame1 = arac.Camera.get_front_cam()
			frame1 = cv2.resize(frame1, (200,150))
			connStation.setFrontCam(frame1)

		if arac.Camera.is_bottom_cam_open():
			ret2, frame2 = arac.Camera.get_bottom_cam()
			frame2 = cv2.resize(frame2, (200,150))
			connStation.setUnderCam(frame2)

		connStation.setPixhawkData(arac.Pixhawk.getData())
		connStation.sendAllData()
		connStation.receiveData()
	except KeyboardInterrupt:
		arac.Camera.release_cams()
		connStation.close()
		break
	except Exception as e:
		arac.Camera.release_cams()
		break
	if cv2.waitKey(1) == ord('q'):
		break

arac.Camera.release_cams()
connStation.close()