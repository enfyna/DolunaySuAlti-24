import cv2
import numpy as np

net = cv2.dnn.readNet("Data/yolov4-tiny.weights", "Data/yolov4-tiny-custom.cfg")

clasees = []
with open("Data/obj.names", 'r') as f:
    clasees = [line.strip() for line in f.readlines()]


cap = cv2.imread("Data/imagesOld/circle (28).jpg")
scale_percent = 60  # percent of original size
width = int(cap.shape[1] * scale_percent / 100)
height = int(cap.shape[0] * scale_percent / 100)
dim = (width, height)
cap = cv2.resize(cap, dim, interpolation=cv2.INTER_AREA)
height, width, channel = cap.shape

cv2.imshow('unCorrected Image', cap)
# underwater correction color 
average_color = cap.mean(axis=0).mean(axis=0)
scale_blue = average_color[0] / 255.0
scale_green = average_color[1] / 255.0
scale_red = average_color[2] / 255.0
corrected_image = cv2.convertScaleAbs(cap, alpha=scale_blue, beta=0)
corrected_image = cv2.convertScaleAbs(corrected_image, alpha=scale_green, beta=0)
corrected_image = cv2.convertScaleAbs(corrected_image, alpha=scale_red, beta=0)
color_correction_matrix = np.array([[1.5, 0.0, 0.0],
                                   [0.0, 1.2, 0.0],
                                   [0.0, 0.0, 0.8]])
corrected_image = cv2.transform(cap, color_correction_matrix)
cv2.imshow('Corrected Image', corrected_image)
cap = corrected_image

#machine learning side
layer_name = net.getLayerNames()
output_layer = [layer_name[i - 1] for i in net.getUnconnectedOutLayers()]
blob = cv2.dnn.blobFromImage(
    cap, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
net.setInput(blob)
outs = net.forward(output_layer)
class_ids = []
confidences = []
boxes = []


#detect circle
for out in outs:
        for detection in out:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]
            if confidence > 0.5:  # dogruluk
                # Object detection
                center_x = int(detection[0] * width)
                center_y = int(detection[1] * height)
                w = int(detection[2] * width)
                h = int(detection[3] * height)
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
            label = str(clasees[class_ids[i]])
            p=50
            cv2.rectangle(cap, (x-p, y-p), (x + w+p, y + h+p), (0,0,255), 2)
            cap = cap[max(y-p,0):min(y+h+p,height),max(x-p,0):min(x+w+p,width)]
            cv2.putText(cap, label, (x, y + 30), font, 3, 0xFFF, 3)


#detect yellow
gray = cv2.cvtColor(cap, cv2.COLOR_BGR2GRAY)
hsv = cv2.cvtColor(cap, cv2.COLOR_BGR2HSV)
yellow_lower = np.array([10, 120, 0])
yellow_upper = np.array([70, 255, 255])
mask_yellow = cv2.inRange(hsv, yellow_lower, yellow_upper)
yellow_output = cv2.bitwise_and(cap, cap, mask=mask_yellow)
yellow_output = cv2.cvtColor(yellow_output, cv2.COLOR_BGR2GRAY)
kernel = np.ones((5, 5), np.uint8)
yellow_output = cv2.dilate(yellow_output, kernel)
# thresh
th = cv2.threshold(yellow_output, 40, 255,
                   cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]
detected_circles = cv2.HoughCircles(th,
                                    cv2.HOUGH_GRADIENT, 1.96, 80)
# Draw circles that are detected.
if detected_circles is not None:

    # Convert the circle parameters a, b and r to integers.
    detected_circles = np.uint16(np.around(detected_circles))

    for pt in detected_circles[0, :]:
        a, b, r = pt[0], pt[1], pt[2]

        # Draw the circumference of the circle.
        cv2.circle(cap, (a, b), r, (0, 255, 0), 2)

        # Draw a small circle (of radius 1) to show the center.
        cv2.circle(cap, (a, b), 1, (0, 0, 255), 3)

cv2.imshow('final', th)
cv2.imshow("yellow", yellow_output)
cv2.imshow("image", cap)
cv2.waitKey(0)
cv2.destroyAllWindows()
