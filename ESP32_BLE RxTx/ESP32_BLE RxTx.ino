#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <ESP32Time.h>


#define BUTTON_PIN 27
#define LED 2
#define ISR_PIN 26
#define SHORT_PRESS 1000
#define LONG_PRESS 2500
#define BAUD_RATE 115200
#define STRUCT_LEN 5
#define customService BLEUUID((uint16_t)0x1700)

ESP32Time rtc(3600);

//Variables Declaration
int currentState;
int lastState = HIGH;
unsigned long pressedTime = 0;
unsigned long releasedTime = 0;
bool deviceConnected = false, flag = false;
bool BLE_flag = 1,run_once = 1;
bool ISR_flag = false;

struct Data {
  float Gforce;
  char RTC[35];   //Thursday, October 13 2022 16:24:34
  char send[43];  //016.55G Thursday, October 13 2022 16:24:34
} D[L];

char value[49] = "NULL";  //String read from the BLE
char input[49] = "NULL";  //String that needs to be parsed

void IRAM_ATTR isr() {
  ISR_flag = true;
}

BLECharacteristic *pCharacteristic;
BLEServer *pServer;
BLEService *pService;
BLECharacteristic customCharacteristic(BLEUUID((uint16_t)0x1A00), BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);

class MyCharacteristicCallbacks : public BLECharacteristicCallbacks {

  void onWrite(BLECharacteristic *customCharacteristic) {

    std::string rcvString = customCharacteristic->getValue();

    if (rcvString.length() > 0) {
      Serial.println("Value Received from BLE: ");
      for (int i = 0; i < rcvString.length(); ++i) {
        Serial.print(rcvString[i]);
        value[i] = rcvString[i];
      }
      for (int i = rcvString.length(); i < 50; ++i) {
        value[i] = NULL;
      }
      customCharacteristic->setValue((char *)&value);
    } 
    else {
      Serial.println("Empty Value Received!");
    }
  }
};

class MyServerCallbacks : public BLEServerCallbacks {

  void onConnect(BLEServer *pServer) {
    deviceConnected = true;
  };

  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
  }
};

void setup() {
  Serial.begin(BAUD_RATE);
  rtc.setTime(30, 24, 15, 17, 1, 2021);
  gpioInit();       //GPIO init for LED or Button, for degugging
  bleInit();        //BLE server initiation function call
  while (BLE_flag)  //flag which is true and becomes false when service is terminated and moves to loop()
  {
    bleServiceTask();
  }
}

void loop() {
  Serial.println("In The Loop");
  // for (int i = 0; i < STRUCT_LEN; i++) {
  //   //strcpy(D[i].RTC, rtc.getTime("RTC: %A, %B %d %Y %H:%M:%S").c_str());
  //   strcpy(D[i].RTC,"016.55G Thursday, October 13 2022 16:24:34" );
  //   strcpy(D[i].send, D[i].RTC);
  //   delay(1000);
  // }
  if (ISR_flag) {
    bleSend();
  } 
  else {
    ISR_flag = false;
  }
}

void bleInit(void) {
  // Create the BLE Device
  BLEDevice::init("Shipment Tracker :)");
  // Create the BLE Server
  pServer = BLEDevice::createServer();
  // Create the BLE Service
  pService = pServer->createService(customService);
  // Create a BLE Characteristic
  pService->addCharacteristic(&customCharacteristic);
  customCharacteristic.setCallbacks(new MyCharacteristicCallbacks());
  pServer->getAdvertising()->addServiceUUID(customService);
  customCharacteristic.setValue((char *)&value);
}

void gpioInit(void) {
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED, OUTPUT);
  attachInterrupt(ISR_PIN, isr, HIGH);
}

void bleServiceTask(void) {
  currentState = digitalRead(BUTTON_PIN);
  if (lastState == HIGH && currentState == LOW)
    pressedTime = millis();
  else if (lastState == LOW && currentState == HIGH) {
    releasedTime = millis();
    long pressedDuration = releasedTime - pressedTime;
    if (pressedDuration < SHORT_PRESS) {
      pService->start();
      pServer->getAdvertising()->start();  //short press to start the BLE service
      Serial.println("BLE SERVICE STARTED");
    }

    if (pressedDuration > LONG_PRESS) {
      pService->stop();
      pServer->getAdvertising()->stop();  //long press to stop the BLE service i.e >2.5sec
      Serial.println("BLE SERVICE STOPPED");
      BLE_flag = 0;
      delay(1000);
    }
  }
  lastState = currentState;
}

void bleSend(void) {
  
  if (run_once) {
    pService->start();
    Serial.println("BLE SERVICE STARTED");
    pServer->getAdvertising()->start();  //start the BLE service
    Serial.println("BLE TRANSFER STARTED");
    run_once = 0;
  }

  for (int i = 0; i < STRUCT_LEN; i++) {

    customCharacteristic.setValue((char *)&D[i].send);
    customCharacteristic.notify();
    delay(1000);
  }
}
