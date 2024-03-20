from ultralytics import YOLO
import cv2
import cvzone
import math
import urllib.request
import numpy as np
import requests
from http.client import IncompleteRead

# Replace with the IP address of your ESP32 device
esp32_ip = '172.20.10.2'  

# URL for the WebServer
url = f"http://{esp32_ip}:81/"

# URL for the AsyncWebServer
stream_url = f"http://{esp32_ip}:80/cam-lo.jpg"

cv2.namedWindow("live Cam Testing", cv2.WINDOW_AUTOSIZE)

# Create a VideoCapture object
cap = cv2.VideoCapture(stream_url)

# Check if the IP camera stream is opened successfully
if not cap.isOpened():
    print("Failed to open the IP camera stream")
    exit()

model = YOLO("yolov8l.pt")

classNames = ["person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", 
              "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
              "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
              "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat",
              "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork",
              "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog",
              "pizza", "donut", "cake", "chair", "couch", "potted plant", "bed", "dining table", "toilet", "tv",
              "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
              "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush",
             ]

prev_message_str = None

car_detected = False

while True:
    img_resp = urllib.request.urlopen(stream_url)

    try:
        imgnp = np.array(bytearray(img_resp.read()), dtype=np.uint8)
    except IncompleteRead as e:
        imgnp = np.array(bytearray(e.partial), dtype=np.uint8)

    img = cv2.imdecode(imgnp,-1)

    results = model(img, stream=True)
    for r in results:
        boxes = r.boxes
        for box in boxes:

            # Bounding Box
            x1,y1,x2,y2 = box.xyxy[0]
            x1,y1,x2,y2 = int(x1), int(y1), int(x2), int(y2)
            # cv2.rectangle(img, (x1,y1), (x2,y2), (0,255,0), 2)
            # print(x1,y1,x2,y2)

            w, h = x2-x1, y2-y1
            

            # Confidence
            conf = math.ceil((box.conf[0] *100))/100   #round confidence to 2 decimal places

            # Class Name
            cls = int(box.cls[0])
            currentClass = classNames[cls]

            # Draw only vehicles
            if currentClass == "car" and conf > 0.3:
                cvzone.putTextRect(img, f'{currentClass} {conf}', (max(0, x1), max(35, y1)), 3, 3, (255,255,255), (0,255,0))
                cvzone.cornerRect(img, (x1,y1,w,h), 30, 5, 1, (0, 255, 0), (0, 255, 0))
                message_str = "True"
                car_detected = True
                #break 

    if not car_detected:
        message_str = "False"

    if message_str != prev_message_str:
        data = {"value": message_str}
        response = requests.post(url, data=data)
        prev_message_str = message_str

    car_detected = False

    cv2.imshow("Image", img)
    cv2.waitKey(1)