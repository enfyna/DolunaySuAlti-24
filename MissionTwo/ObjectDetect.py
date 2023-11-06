import cv2
import numpy as np

class ObjectDetect():
        
    def __init__(self):
        self.net = cv2.dnn.readNet("Data/yolov4-tiny.weights", "Data/yolov4-tiny-custom.cfg")

        self.classes = []
        with open("Data/obj.names", 'r') as f:
            self.classes = [line.strip() for line in f.readlines()]
        self.layer_name = self.net.getLayerNames()
        self.output_layer = [self.layer_name[i - 1] for i in self.net.getUnconnectedOutLayers()]
        self.width=0
        self.height=0
        self.FIND=0

    def ResizeImage(self,frame):
        scale_percent = 100  # percent of original size
        self.width = int(frame.shape[1] * scale_percent / 100)
        self.height = int(frame.shape[0] * scale_percent / 100)
        dim = (self.width, self.height)
        frame = cv2.resize(frame, dim, interpolation=cv2.INTER_AREA)
        return frame
        
    def UnderWaterColorCorrection(self,frame):
        img_lab = cv2.cvtColor(frame, cv2.COLOR_BGR2Lab)
        L, a, b = cv2.split(img_lab)
        a = cv2.add(a, 5)
        b = cv2.subtract(b, 8)
        L = cv2.add(L,20)
        img_corrected_lab = cv2.merge((L, a, b))
        img_corrected = cv2.cvtColor(img_corrected_lab, cv2.COLOR_Lab2BGR)
        return frame
    
    def DetectObject(self,frame):
        blob = cv2.dnn.blobFromImage(
            frame, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
        self.net.setInput(blob)
        outs = self.net.forward(self.output_layer)
        class_ids = []
        confidences = []
        boxes = []
        self.FIND=0
        for out in outs:
                for detection in out:
                    scores = detection[5:]
                    class_id = np.argmax(scores)
                    confidence = scores[class_id]
                    if confidence > 0.5:  # dogruluk
                        # Object detection
                        center_x = int(detection[0] * self.width)
                        center_y = int(detection[1] * self.height)
                        w = int(detection[2] * self.width)
                        h = int(detection[3] * self.height)
                        # Reactangle Cordinate
                        x = int(center_x - w/2)
                        y = int(center_y - h/2)
                        boxes.append([x, y, w, h])
                        confidences.append(float(confidence))
                        class_ids.append(class_id)
        indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
        font = cv2.FONT_HERSHEY_PLAIN
        for i in range(len(boxes)):
                if i in indexes:
                    x, y, w, h = boxes[i]
                    label = str(self.classes[class_ids[i]])
                    p=50
                    cv2.rectangle(frame, (x-p, y-p), (x + w+p, y + h+p), (0,0,255), 2)
                    #frame = frame[max(y-p,0):min(y+h+p,self.height),max(x-p,0):min(x+w+p,self.width)]
                    cv2.putText(frame, label, (x, y + 30), font, 3, 0xFFF, 3)
                    self.FIND=2
        return frame
        
    def DetectColor(self,frame):
        frame_copy=frame.copy()
        #detect yellow
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.medianBlur(gray, 5)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        yellow_lower = np.array([10, 120, 0])
        yellow_upper = np.array([70, 255, 255])
        mask_yellow = cv2.inRange(hsv, yellow_lower, yellow_upper)
        yellow_output = cv2.bitwise_and(frame, frame, mask=mask_yellow)
        yellow_output = cv2.cvtColor(yellow_output, cv2.COLOR_BGR2GRAY)
        kernel = np.ones((5, 5), np.uint8)
        yellow_output = cv2.dilate(yellow_output, kernel)
        # thresh
        th = cv2.threshold(yellow_output, 40, 255,
                           cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]
        return th,frame_copy
        
    def DetectCircle(self,frame,frame_copy):
        detected_circles = cv2.HoughCircles(frame,
                                            cv2.HOUGH_GRADIENT, 1.96, 80)
        if(self.FIND!=2):
            self.FIND=0
        # Draw circles that are detected.
        if detected_circles is not None:
            # Convert the circle parameters a, b and r to integers.
            detected_circles = np.uint16(np.around(detected_circles))
            for pt in detected_circles[0, :]:
                a, b, r = pt[0], pt[1], pt[2]
                # Draw the circumference of the circle.
                cv2.circle(frame_copy, (a, b), r, (0, 255, 0), 2)
                self.x_circle=int(a-self.width/2)
                self.y_circle=int(b-self.height/2)
                # Draw a small circle (of radius 1) to show the center.
                cv2.circle(frame_copy, (a, b), 1, (0, 0, 255), 3)
                self.FIND=1
        return frame_copy
    
    def getFIND(self):
        return self.FIND
        
    def getPointCenterCircle(self):
        return self.x_circle,self.y_circle
    
    def Detect(self,frame):
        frame = self.ResizeImage(frame)
        frame = self.UnderWaterColorCorrection(frame)
        frame = self.DetectObject(frame)
        frame,frame_copy = self.DetectColor(frame)
        frame = self.DetectCircle(frame,frame_copy)
        return frame