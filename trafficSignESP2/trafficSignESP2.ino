/*
  Two way communication example: https://RandomNerdTutorials.com/esp-now-two-way-communication-esp32/
  
*/

//for esp 1

#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>

#define onBoard 2
#define echoPin2 22
#define trigPin2 23 
#define RXp2 16
#define TXp2 17  

// REPLACE WITH THE MAC Address of your receiver 
uint8_t broadcastAddress1[] = {0xA0, 0xA3, 0xB3, 0xED, 0xA5, 0x60}; // esp1
uint8_t broadcastAddress2[] = {0xA0, 0xA3, 0xB3, 0xED, 0xB7, 0x1C}; // esp3

//define Servo object
Servo myServo;
int Servo = 13;

//local variables
bool prevRedRed = false;
int carsInLane = 0;
bool prevIsSlow = false;
bool isTimerGoing = false;
bool isTimerYellowGoing = false;
bool isTimerRedGoing = false;
bool prevCarCheck3 = false;       //car entering
bool currentCarCheck3 = false;
bool prevCarCheck4 = false;       // car leaving
bool currentCarCheck4 = false;
unsigned long prevMillis1 = 0;
unsigned long prevMillis2 = 0;
unsigned long prevMillis3 = 0;
unsigned long timer = 0; // for when the lane is congested
unsigned long timerYellow = 0; // for when the light just turned yellow
unsigned long timerRed = 0;  // for when the light just turned red
unsigned long timerBeforeYellow = 0; // for before the light can turn yellow
const long interval1 = 250; // .25 sec for car count
const long interval2 = 250; // .25 sec for car subtract
const long interval3 = 50; // .05 sec for sending radio (50 for fastest) (1000 for testing)

const long interval4 = 3000; // timer for when there is constant traffic flow on both sides
const long interval5 = 5000; // timer for when the light just turned yellow
const long interval6 = 3000; // timer for when the light just turned red
const long interval7 = 3000; // timer for before the light can turn yellow incase a car runs the opposite red

// Define variables to be sent
int carsEntering;
int carsLeaving;
int signCount;
bool isSlow = false;
bool isCarWaiting = false;
bool isCarApproaching = false;
bool isGoToRed = false;

// Define variables to store incoming readings
int incomingCarsEntering;
int incomingCarsLeaving;
bool incomingIsSlow;
bool incomingIsCarWaiting;
bool incomingIsCarApproaching;
bool incomingIsGoToRed;
bool incomingIsRedRed;

// Variable to store if sending data was successful
String success;

//Structure example to send data
//Must match the receiver structure
typedef struct struct_message {
    int carsinlane;
    int id;
    int carsentering;
    int carsleaving;
    int signCount;
    bool slw;
    bool iscarwaiting;
    bool iscarapproaching;
    bool isgotored;
} struct_message;

typedef struct struct_message2 {
  bool redred;
} struct_message2;

// Create a struct_message to hold sensor readings
struct_message trafficSignReadings;

// Create a struct_message to hold incoming sensor readings
struct_message incomingReadings;

// a struct_message2 to hold incoming readings
struct_message2 incomingRemote;

esp_now_peer_info_t peerInfo;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //Serial.print("\r\nLast Packet Send Status:\t");
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0){
    //success = "Delivery Success :)";
  }
  else{
    //success = "Delivery Fail :(";
  }
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  //Serial.print("Bytes received: ");
  //Serial.println(len);
   if(len >= 2){
   incomingCarsEntering = incomingReadings.carsentering;
   incomingCarsLeaving = incomingReadings.carsleaving;
   incomingIsSlow = incomingReadings.slw;
   incomingIsCarWaiting = incomingReadings.iscarwaiting;
   incomingIsCarApproaching = incomingReadings.iscarapproaching;
   incomingIsGoToRed = incomingReadings.isgotored;
  }
  else {
    memcpy(&incomingRemote, incomingData, sizeof(incomingRemote));
    incomingIsRedRed = incomingRemote.redred;
  }
}
 
void setup() {
  // Init Serial Monitor
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RXp2, TXp2);

  pinMode(onBoard, OUTPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);

  /* servo */
  myServo.attach(Servo);
  myServo.write(0);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    //Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // register peer
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  // register first peer  
  memcpy(peerInfo.peer_addr, broadcastAddress1, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  // register second peer  
  memcpy(peerInfo.peer_addr, broadcastAddress2, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
       
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
}
 
void loop() {
  unsigned long currentMillis = millis();

  if(incomingIsRedRed){

    carsInLane = 0;
    // Set values to send
    trafficSignReadings.id = 2;
    trafficSignReadings.slw = isSlow;
    trafficSignReadings.carsentering = carsEntering;
    trafficSignReadings.carsleaving = carsLeaving;
    trafficSignReadings.iscarapproaching = isCarApproaching;
    trafficSignReadings.iscarwaiting = isCarWaiting;
    trafficSignReadings.isgotored = isGoToRed;
    trafficSignReadings.carsinlane = carsInLane;

    // Send message via ESP-NOW
    if (currentMillis - prevMillis3 >= interval3){

      esp_err_t result = esp_now_send(0, (uint8_t *) &trafficSignReadings, sizeof(trafficSignReadings));
      prevMillis3 = currentMillis;
        if (result == ESP_OK) {
          //Serial.println("Sent with success");
        }
        else {
          //Serial.println("Error sending the data");
        }
    }
  }

  if(!incomingIsRedRed && prevRedRed && isSlow){
    moveServoSlow();
  }

  if(!incomingIsRedRed){
    // If the sign is yellow/slow
    if(isSlow){

      //Outside sensor
      if (currentMillis - prevMillis1 >= interval1) {
        prevMillis1 = currentMillis;
        carDetection1();     
      }
      //Inside sensor
      if (currentMillis - prevMillis2 >= interval2) {
        UltrasonicRead3();
        prevMillis2 = currentMillis;
      }
      carsInLane = carsEntering - incomingCarsLeaving;
    }
    
    // If the sign is red/stop and the light did not just turn red
    else if ((currentMillis - timerRed >= interval6) && !isSlow && (!prevIsSlow || incomingIsSlow) ) { // added && (!prevIsSlow || incomingIsSlow)

      //Outside sensor
      if (currentMillis - prevMillis1 >= interval1) {
        prevMillis1 = currentMillis;
        carDetection2();     
      }
      //Inside sensor
      if (currentMillis - prevMillis2 >= interval2) {
        UltrasonicRead4();
        prevMillis2 = currentMillis;
      }
      carsInLane = incomingCarsEntering - carsLeaving;
        if(prevIsSlow && (!incomingIsSlow) ){
          carsInLane =  carsEntering - incomingCarsLeaving;
        }
      isTimerRedGoing = false;
    }

    // If the light just turned red/stop continue to count cars entering the lane for 3 seconds
    // In case a car runs the red light.
    else if(!isSlow && prevIsSlow && !incomingIsSlow){  //was (!isSlow && isTimerRedGoing)

      if (currentMillis - prevMillis2 >= interval2) {
        UltrasonicRead3();
        prevMillis2 = currentMillis;
      }
      carsInLane =  carsEntering - incomingCarsLeaving;
    }

    // Set values to send
    trafficSignReadings.id = 2;
    trafficSignReadings.slw = isSlow;
    trafficSignReadings.carsentering = carsEntering;
    trafficSignReadings.carsleaving = carsLeaving;
    trafficSignReadings.iscarapproaching = isCarApproaching;
    trafficSignReadings.iscarwaiting = isCarWaiting;
    trafficSignReadings.isgotored = isGoToRed;
    trafficSignReadings.carsinlane = carsInLane;
    
    // Send message via ESP-NOW
    if (currentMillis - prevMillis3 >= interval3){

      esp_err_t result = esp_now_send(0, (uint8_t *) &trafficSignReadings, sizeof(trafficSignReadings));
      prevMillis3 = currentMillis;
        if (result == ESP_OK) {
          //Serial.println("Sent with success");
        }
        else {
          //Serial.println("Error sending the data");
        }
    }
    
    updateSign();
    delay(100);
    prevRedRed = false;
  }

  // If the light is yellow, turn to red and reset carsInLane
  if (incomingIsRedRed && !prevRedRed && isSlow) {
      moveServoStop();
      prevRedRed = incomingIsRedRed;
      carsEntering = 0;
      carsLeaving = 0;
      carsInLane = 0;
  }

  // If the light is already red reset carsInLane
  else if (incomingIsRedRed && !prevRedRed && !isSlow) {
      prevRedRed = incomingIsRedRed;
      carsEntering = 0;
      carsLeaving = 0;
      carsInLane = 0;
  }
}


/* When yellow "car is approaching" */  
void carDetection1(){
    String incomingMessage = Serial2.readString();
    incomingMessage.trim();
    Serial.print("Message Received: ");
    Serial.println(incomingMessage);
    if(incomingMessage == "True")
    {
      isCarApproaching = true;
      digitalWrite(2, HIGH);
    }
    else if (incomingMessage == "False")
    {
      isCarApproaching = false;
      digitalWrite(2, LOW);
    }
}
/* When red say "car is waiting"*/
void carDetection2(){
    String incomingMessage = Serial2.readString();
    incomingMessage.trim();
    Serial.print("Message Received: ");
    Serial.println(incomingMessage);
    if(incomingMessage == "True")
    {
      isCarWaiting = true;
      digitalWrite(2, HIGH);
    }
    else if (incomingMessage == "False")
    {
      isCarWaiting = false;
      digitalWrite(2, LOW);
    }
}

/*When yellow "car entered lane"*/
void UltrasonicRead3(){
    long duration, distance;
    digitalWrite(trigPin2, LOW);  
    delayMicroseconds(2); 
    digitalWrite(trigPin2, HIGH);
    delayMicroseconds(10); 
    digitalWrite(trigPin2, LOW);
    duration = pulseIn(echoPin2, HIGH);
    distance = (duration/2) / 29.1;
    //Serial.println(distance);
    if ((distance < 20) && (distance > 0)) { 
      digitalWrite(onBoard, HIGH);
      currentCarCheck3 = true;                   
      if (currentCarCheck3 && !prevCarCheck3) {
        signCount++;
        carsEntering++;
        Serial.print("Cars Entering Lane: ");
        Serial.println(carsEntering);
      }
    } 
    else {
      digitalWrite(onBoard, LOW);
      currentCarCheck3 = false;
    }
    prevCarCheck3 = currentCarCheck3;
}

/* When red say "car is leaving"*/
void UltrasonicRead4(){
    long duration, distance;
    digitalWrite(trigPin2, LOW);  
    delayMicroseconds(2); 
    digitalWrite(trigPin2, HIGH);
    delayMicroseconds(10); 
    digitalWrite(trigPin2, LOW);
    duration = pulseIn(echoPin2, HIGH);
    distance = (duration/2) / 29.1;
    //Serial.println(distance);
    if ((distance < 20) && (distance > 0)) { 
      digitalWrite(onBoard, HIGH);
      currentCarCheck4 = true; 
      //Serial.print("Cars entering Lane: ");
      //Serial.println(incomingCarsEntering);                  
      if (currentCarCheck4 && !prevCarCheck4 && (incomingCarsEntering > carsLeaving)) {
        carsLeaving++;
        Serial.print("Cars Leaving: ");
        Serial.println(carsLeaving);
      }
    } 
    else {
      digitalWrite(onBoard, LOW);
      currentCarCheck4 = false;
    }
    prevCarCheck4 = currentCarCheck4;
}

void moveServoStop() {
     myServo.write(0);
    delay(500);
    myServo.detach();
}

void moveServoSlow() {
    myServo.write(180);
    delay(500);
    myServo.detach();
}

void updateSign(){

unsigned long currentMillis = millis();


// If a car is waiting at the red light and the other light is yellow
  if(isCarWaiting && incomingIsSlow){

    prevIsSlow = isSlow;

    // If the lane is occupied
    if((carsInLane > 0) && !isTimerGoing){
      timer = millis();
      isTimerGoing = true;
      Serial.println("Timer Started");
      Serial.println("Waiting for car to exit lane");
    }

    // If the timer ended or the cars left the lane
    if((currentMillis - timer >= interval4) || (carsInLane == 0)){
      isTimerGoing = false;
      isGoToRed = true;
      timerBeforeYellow = millis();
    }
  }

  // If the timer ends and there is still a car in the lane
  // Then the other light is red and a car is waiting here till all the cars leave the lane
  // Must wait 3 seconds because a car at the other end might run the light
  // Need the !prevIsSlow because This state can only happen during prev red and current red
  if((currentMillis - timerBeforeYellow >= interval7) && isCarWaiting && !incomingIsSlow && !prevIsSlow && (carsInLane == 0) && !isSlow){
    moveServoSlow();
    carsEntering = 0;
    isGoToRed = false;
    isSlow = true;
    timerYellow = millis();
    Serial.print("Timer started at: ");
    Serial.println(timerYellow);
    isCarWaiting = false;
  }

  //In order for the light to go to red, 5 seconds must have passed from when the light first turned yellow
  if((currentMillis - timerYellow >= interval5) && incomingIsGoToRed && isSlow){
    moveServoStop();
    //Start timer for turning red, This is because we will continue to red cars entering the lane
    //In case cars run the red light.
    timerRed = millis();
    isTimerRedGoing = true;
    
    prevIsSlow = isSlow;
    isSlow = false;
    carsLeaving = 0;
    Serial.println("going to red");
  }
}