from ultralytics import YOLO
import cv2
import cvzone
import math
import requests

# cap = cv2.VideoCapture(0) # For webcam
# cap.set(3, 640)
# cap.set(4, 480)
url = "http://192.168.1.195/"


cap = cv2.VideoCapture("videos/veh2.mp4") # For video

cap.set(3, 640)
cap.set(4, 480)

model = YOLO("yolov8n.pt")

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

while True:
    success, img = cap.read()
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