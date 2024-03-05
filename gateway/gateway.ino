/*
  
  Inspiration: https://RandomNerdTutorials.com/esp32-web-bluetooth/
  
  BLE Remote
  
*/
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <esp_now.h>
#include <WiFi.h>

#define onBoard 2
// See the following for generating UUIDs:
// https://www.uuidgenerator.net/
#define SERVICE_UUID        "6c744422-08a1-40c7-807a-576b64b52437"
#define SOUTH_CHARACTERISTIC_UUID "2bd7866c-14ca-4191-a09f-c4985352cc96"
#define REDRED_CHARACTERISTIC_UUID "de420cdd-4085-4066-8ee7-a6c5e28316d5"
#define NORTH_CHARACTERISTIC_UUID "dc51766b-fe18-4d8f-bb31-d49e82e59e18"
#define CARCOUNT_CHARACTERISTIC_UUID "c28e246d-5632-44d7-8fdb-124f4243eb10"
#define SIGN1COUNT_CHARACTERISTIC_UUID "65037f44-f9f5-48a4-8205-e9d3dc574316"
#define SIGN2COUNT_CHARACTERISTIC_UUID "55a12381-bb3c-4731-9cde-99fcbc13fca2"

uint8_t broadcastAddress1[] = {0xA0, 0xA3, 0xB3, 0xED, 0xA5, 0x60};  // Node 1
uint8_t broadcastAddress2[] = {0xD4, 0x8A, 0xFC, 0x9E, 0x1C, 0xB0};  // Node 2

BLEServer* pServer = NULL;
BLECharacteristic* pSouthCharacteristic = NULL;
BLECharacteristic* pNorthCharacteristic = NULL;
BLECharacteristic* pCarCountCharacteristic = NULL;
BLECharacteristic* pRedRedCharacteristic = NULL;
BLECharacteristic* pSign1CountCharacteristic = NULL;
BLECharacteristic* pSign2CountCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// local variables
unsigned long prevMillis2 = 0;
unsigned long prevMillis3 = 0;
const long interval2 = 3000;
const long interval3 = 50; // .05 sec for sending radio (50 for fastest) (1000 f

// Variable to be sent
bool isRedRed = false;

// Variable to store if sending data was successful
String success;

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

// Holds current data
struct_message myData;

// for sending data
struct_message2 RemoteReadings;

// readings from each baord
struct_message board1;
struct_message board2;

struct_message boardsStruct[2] = {board1, board2};

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

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
    //Serial.print("Bytes received: ");
    //Serial.println(len);
    memcpy(&myData, incomingData, sizeof(myData));
    boardsStruct[myData.id-1].carsinlane = myData.carsinlane;
    boardsStruct[myData.id-1].carsentering = myData.carsentering;
    boardsStruct[myData.id-1].carsleaving = myData.carsleaving;
    boardsStruct[myData.id-1].signCount = myData.signCount;
    boardsStruct[myData.id-1].slw = myData.slw;
    boardsStruct[myData.id-1].iscarwaiting = myData.iscarwaiting;
    boardsStruct[myData.id-1].iscarapproaching = myData.iscarapproaching;
    boardsStruct[myData.id-1].isgotored = myData.isgotored;
}

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

class MyCharacteristicCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pRedRedCharacteristic) {
        std::string value = pRedRedCharacteristic->getValue();
        if (value.length() > 0) {
            Serial.print("Characteristic event, written: ");
            Serial.println(static_cast<int>(value[0])); // Print the integer value

            int receivedValue = static_cast<int>(value[0]);
            if (receivedValue == 1) {
                isRedRed = true;
                digitalWrite(onBoard, HIGH);
            } else {
                isRedRed = false;
                digitalWrite(onBoard, LOW);
            }
        }
    }
};

void setup() {
  Serial.begin(115200);
  pinMode(onBoard, OUTPUT);

  // Create the BLE Device
  BLEDevice::init("ESP32");

  //Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  //Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
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
    return ;
  }
  // register second peer  
  memcpy(peerInfo.peer_addr, broadcastAddress2, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(BLEUUID(SERVICE_UUID), 30, 0);

  // Create a BLE Characteristic
  pSouthCharacteristic = pService->createCharacteristic(
                      SOUTH_CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );
  pNorthCharacteristic = pService->createCharacteristic(
                      NORTH_CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );
  pCarCountCharacteristic = pService->createCharacteristic(
                      CARCOUNT_CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );
  pSign1CountCharacteristic = pService->createCharacteristic(
                      SIGN1COUNT_CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );
  pSign2CountCharacteristic = pService->createCharacteristic(
                      SIGN2COUNT_CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );

  // Create the REDRED button Characteristic
  pRedRedCharacteristic = pService->createCharacteristic(
                      REDRED_CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_WRITE
                    );

  // Register the callback for the ON button characteristic
  pRedRedCharacteristic->setCallbacks(new MyCharacteristicCallbacks());

  // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
  // Create a BLE Descriptor
  
  pSouthCharacteristic->addDescriptor(new BLE2902());
  pNorthCharacteristic->addDescriptor(new BLE2902());
  pCarCountCharacteristic->addDescriptor(new BLE2902());
  pSign1CountCharacteristic->addDescriptor(new BLE2902());
  pSign2CountCharacteristic->addDescriptor(new BLE2902());
  pRedRedCharacteristic->addDescriptor(new BLE2902());

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  Serial.println("Waiting a client connection to notify...");
}

void loop() {
    unsigned long currentMillis = millis();

    int board1CarsEntering = boardsStruct[0].carsentering;
    int board1CarsLeaving = boardsStruct[0].carsleaving;
    int board1SignCount = boardsStruct[0].signCount;
    bool board1IsSlow = boardsStruct[0].slw;
    bool board1IsCarWaiting = boardsStruct[0].iscarwaiting;
    bool board1IsCarApproaching = boardsStruct[0].iscarapproaching;
    bool board1IsGoToRed = boardsStruct[0].isgotored;

    int board2CarsEntering = boardsStruct[1].carsentering;
    int board2CarsLeaving = boardsStruct[1].carsleaving;
    int board2SignCount = boardsStruct[1].signCount;
    bool board2IsSlow = boardsStruct[1].slw;
    bool board2IsCarWaiting = boardsStruct[1].iscarwaiting;
    bool board2IsCarApproaching = boardsStruct[1].iscarapproaching;
    bool board2IsGoToRed = boardsStruct[1].isgotored;

    int carsInLane = boardsStruct[1].carsinlane;

     // Set values to send
     RemoteReadings.redred = isRedRed;

      // Send message via ESP-NOW
    if (currentMillis - prevMillis3 >= interval3){

        esp_err_t result = esp_now_send(0, (uint8_t *) &RemoteReadings, sizeof(RemoteReadings));
        prevMillis3 = currentMillis;
      if (result == ESP_OK) {
        //Serial.println("Sent with success");
      }
      else {
        //Serial.println("Error sending the data");
      }
    }

    

    // notify changed value
    if (deviceConnected) {
        pSouthCharacteristic->setValue(String(board1IsSlow).c_str());
        pSouthCharacteristic->notify();
        Serial.print("New value notified: ");
        Serial.println(board1IsSlow);

        pNorthCharacteristic->setValue(String(board2IsSlow).c_str());
        pNorthCharacteristic->notify();
        Serial.print("New value notified: ");
        Serial.println(board2IsSlow);

        pCarCountCharacteristic->setValue(String(carsInLane).c_str());
        pCarCountCharacteristic->notify();
        Serial.print("New value notified: ");
        Serial.println(carsInLane);

        pSign1CountCharacteristic->setValue(String(board1SignCount).c_str());
        pSign1CountCharacteristic->notify();
        Serial.print("New value notified: ");
        Serial.print(board1SignCount);

        pSign2CountCharacteristic->setValue(String(board2SignCount).c_str());
        pSign2CountCharacteristic->notify();
        Serial.print("New value notified: ");
        Serial.print(board2SignCount);

        delay(500); // bluetooth stack will go into congestion, if too many packets are sent, in 6 hours test i was able to go as low as 3ms
    }
    // disconnecting
    if (!deviceConnected && oldDeviceConnected) {
        Serial.println("Device disconnected.");
        delay(500); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        Serial.println("Start advertising");
        oldDeviceConnected = deviceConnected;
    }
    // connecting
    if (deviceConnected && !oldDeviceConnected) {
        // do stuff here on connecting
        oldDeviceConnected = deviceConnected;
        Serial.println("Device Connected");
    }
}