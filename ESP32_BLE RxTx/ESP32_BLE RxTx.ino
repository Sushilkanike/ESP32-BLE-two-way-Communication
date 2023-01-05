#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <ESP32Time.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>

//#define BUTTON_PIN 26
#define LED 2
#define ISR_PIN 26
#define SHORT_PRESS 1000
#define LONG_PRESS 2500
#define BAUD_RATE 115200
#define STRUCT_LEN 5
#define customService BLEUUID((uint16_t)0x1700)

ESP32Time rtc(3600);

Adafruit_MPU6050 mpu;

//Variables Declaration
int currentState, i, k, count = 0;
int lastState = HIGH;
unsigned long pressedTime = 0;
unsigned long releasedTime = 0;
bool deviceConnected = false, flag = false;
bool BLE_flag = 1, run_once = 1, run_once_1 = 1;
bool ISR_flag = false;
float X, Y, Z, L, W, H, M, CBM, G;
char temp[8];
String rcvString;

struct Data {
  float Gforce;
  char RTC[35];   //Thursday, October 13 2022 16:24:34
  String send;  //016.55G Thursday, October 13 2022 16:24:34
} D[STRUCT_LEN];

char value[40] = "NULL";  //String read from the BLE
char input[40] = "NULL";
char STRING_ERROR[50];  //String that needs to be parsed

void IRAM_ATTR isr() {
  ISR_flag = true;
  //restartAdvertising();
}

BLECharacteristic *pCharacteristic;
BLEServer *pServer;
BLEService *pService;
BLECharacteristic customCharacteristic(BLEUUID((uint16_t)0x1A00), BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);

class MyCharacteristicCallbacks : public BLECharacteristicCallbacks {

  void onWrite(BLECharacteristic *customCharacteristic) {

    std::string rcvString = customCharacteristic->getValue();

    if (rcvString.length() == 3) {
      Serial.println("BLE SERVICE STOPPED");
      BLE_flag = 0;
      for (int i = 0; i < 7; ++i) {
        rcvString[i] = 'F';
      }
      // pService->stop();
      pServer->getAdvertising()->stop();  //long press to stop the BLE service i.e >2.5sec
    } else if (rcvString.length() > 5) {
      Serial.println("Recieved from BLE: ");
      for (int i = 0; i < rcvString.length(); ++i) {
        Serial.print(rcvString[i]);
        value[i] = rcvString[i];
      }
    }
      if (run_once_1) {
    Serial.println("BLE TRANSFER STARTED");
    for (int i = 0; i < STRUCT_LEN; i++) {
      customCharacteristic->setValue((char *)&D[i].send);
      customCharacteristic->notify();
      delay(2000);
      run_once_1 = 0;
    }
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

  pService->start();
  pServer->getAdvertising()->start();
  Serial.println("BLE SERVICE STARTED");
}

void gpioInit(void) {
  //pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED, OUTPUT);
  attachInterrupt(ISR_PIN, isr, CHANGE);
}

// void bleServiceTask(void) {
//   currentState = digitalRead(BUTTON_PIN);
//   if (lastState == HIGH && currentState == LOW)
//     pressedTime = millis();
//   else if (lastState == LOW && currentState == HIGH) {
//     releasedTime = millis();
//     long pressedDuration = releasedTime - pressedTime;
//     if (pressedDuration < SHORT_PRESS) {
//       pService->start();
//       pServer->getAdvertising()->start();  //short press to stop the BLE service i.e >2.5sec
//       Serial.println("BLE SERVICE STARTED");
//       BLE_flag = 0;
//       delay(1000);
//     }

//     if (pressedDuration > LONG_PRESS) {
//       pService->stop();
//       pServer->getAdvertising()->stop();  //long press to stop the BLE service i.e >2.5sec
//       Serial.println("BLE SERVICE STOPPED");
//       BLE_flag = 0;
//       delay(1000);
//     }
//   }
//   lastState = currentState;
// }

void bleSend(void) {

  if (deviceConnected) {
    Serial.println("BLE TRANSFER STARTED");
    for (int i = 0; i < STRUCT_LEN; i++) {

      customCharacteristic.setValue((char *)&D[i].send);
      customCharacteristic.notify();
      delay(2000);
    }
  }
}

void get_IMU() {

  /* Get new sensor events with the readings */
  sensors_event_t a, g, Temp;
  mpu.getEvent(&a, &g, &Temp);

  /* Print out the values */
  Serial.print("AccelX:");
  Serial.print(a.acceleration.x);
  Serial.print(",");
  Serial.print("AccelY:");
  Serial.print(a.acceleration.y);
  Serial.print(",");
  Serial.print("AccelZ:");
  Serial.print(a.acceleration.z);
  Serial.print(", ");
  Serial.print("GyroX:");
  Serial.print(g.gyro.x);
  Serial.print(",");
  Serial.print("GyroY:");
  Serial.print(g.gyro.y);
  Serial.print(",");
  Serial.print("GyroZ:");
  Serial.print(g.gyro.z);
  Serial.println("");

  delay(10);
}

void recieveBLE() {
  strcpy(input, value);
  for (i = 0; i < 40; i++) {
    Serial.print(input[i]);
  }

  Serial.println();
  if ((input[0] == 'S') && (input[38] == 'E')) {
    for (k = 0, i = 1; k < 3, i < 4; k++, i++) {
      temp[k] = input[i];
    }
    X = atof(temp);
    destroytemp();

    for (k = 0, i = 5; k < 3, i < 8; k++, i++) {
      temp[k] = input[i];
    }
    Y = atof(temp);
    destroytemp();

    for (k = 0, i = 9; k < 3, i < 12; k++, i++) {
      temp[k] = input[i];
    }
    Z = atof(temp);
    destroytemp();

    for (k = 0, i = 12; k < 5, i < 18; k++, i++) {
      temp[k] = input[i];
    }
    L = atof(temp);
    destroytemp();

    for (k = 0, i = 19; k < 5, i < 24; k++, i++) {
      temp[k] = input[i];
    }
    W = atof(temp);
    destroytemp();

    for (k = 0, i = 25; k < 5, i < 30; k++, i++) {
      temp[k] = input[i];
    }
    H = atof(temp);
    destroytemp();
    parameters();

    for (k = 0, i = 31; k < 7, i < 38; k++, i++) {
      temp[k] = input[i];
    }
    M = atof(temp);
    destroytemp();

  } else {
    Serial.println("The String recieved is broken, Resend the data...");
    char STRING_ERROR[50] = "The String recieved is broken, Resend the data...";
    delay(500);
  }
}
void parameters() {
  Serial.println();
  Serial.print("Orientation Along X, Y, Z: ");
  Serial.print(X);
  Serial.print(" ");
  Serial.print(Y);
  Serial.print(" ");
  Serial.println(Z);
  Serial.print("Dimentions Length, Width, Height: ");
  Serial.print(L);
  Serial.print(" ");
  Serial.print(W);
  Serial.print(" ");
  Serial.println(H);
  Serial.print("Weight: ");
  Serial.println(M);
  CBM = L * W * H;  //cubic meters
  G = getGforce(CBM, M);
  Serial.print("Gforce: ");
  Serial.println(G);
}


int getGforce(float D, float M) {
  int i, j;

  //for 100G
  if ((D > 0 && D <= 0.03) && (M > 0 && M <= 4.56) || (D > 0.03 && D <= 0.14) && (M > 0 && M <= 4.56) || (D > 0.03 && D <= 0.14) && (M > 4.56 && M <= 11.34))
    return 100;

  //for 75G
  else if ((D > 0 && D <= 0.03) && (M > 11.34 && M <= 22.68) || (D > 0 && D <= 0.03) && (M > 22.68 && M <= 45.36) || (D > 0.03 && D <= 0.14) && (M > 4.56 && M <= 11.34) || (D > 0.03 && D <= 0.14) && (M > 11.34 && M <= 22.68) || (D > 0.14 && D <= 0.42) && (M > 0 && M <= 4.56) || (D > 0.14 && D <= 0.42) && (M > 4.56 && M <= 11.34) || (D > 0.42 && D <= 1.42) && (M > 0 && M <= 4.56))
    return 75;

  //for 50G
  else if ((D > 0 && D <= 0.03) && (M > 45.36 && M <= 113.40) || (D > 0 && D <= 0.03) && (M > 113.40 && M <= 453.59) || (D > 0.03 && D <= 0.14) && (M > 22.68 && M <= 45.36) || (D > 0.03 && D <= 0.14) && (M > 45.36 && M <= 113.40) || (D > 0.03 && D <= 0.14) && (M > 113.40 && M <= 453.59) || (D > 0.14 && D <= 0.42) && (M > 11.34 && M <= 22.68) || (D > 0.14 && D <= 0.42) && (M > 22.68 && M <= 45.36) || (D > 0.42 && D <= 1.42) && (M > 4.56 && M <= 11.34) || (D > 0.42 && D <= 1.42) && (M > 11.34 && M <= 22.68) || (D > 1.42) && (M > 0 && M <= 4.56) || (D > 1.42) && (M > 4.56 && M <= 11.34))
    return 50;

  //for 37G
  else if ((D > 0.03 && D <= 0.14) && (M > 453.59) || (D > 0.14 && D <= 0.42) && (M > 45.36 && M <= 113.40) || (D > 0.14 && D <= 0.42) && (M > 113.40 && M <= 453.59) || (D > 0.42 && D <= 1.42) && (M > 22.68 && M <= 45.36) || (D > 0.42 && D <= 1.42) && (M > 45.36 && M <= 113.40) || (D > 1.42) && (M > 11.34 && M <= 22.68) || (D > 1.42) && (M > 22.68 && M <= 45.36))
    return 37;

  //for 25G
  else if ((D > 0.14 && D <= 0.42) && (M > 453.59) || (D > 0.42 && D <= 1.42) && (M > 113.40 && M <= 453.59) || (D > 0.42 && D <= 1.42) && (M > 453.59) || (D > 1.42) && (M > 45.36 && M <= 113.40) || (D > 1.42) && (M > 113.40 && M <= 453.59) || (D > 1.42) && (M > 453.59))
    return 25;

  else return 0;
}

void destroytemp() {
  for (int j = 0; j < 8; j++)
    temp[j] = 0;
}
void restartAdvertising() {

  if (run_once) {
    Serial.println("interrupt triggered!");
    pServer->getAdvertising()->start();
    //pService->start();
    Serial.println("Restarted advertising");
    run_once = 0;
  }
}

void setup() {
  Serial.begin(BAUD_RATE);
  rtc.setTime(30, 24, 15, 17, 1, 2021);
  gpioInit();       //GPIO init for LED or Button, for degugging
  bleInit();        //BLE server initiation function call
  while (BLE_flag)  //flag which is true and becomes false when service is terminated and moves to loop()
  {
    //bleServiceTask();
    Serial.println("In the While");
    Serial.println(BLE_flag);
    recieveBLE();
    delay(1000);
  }
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("");
  delay(100);
  run_once = 1;

  //temp declarations
  D[0].send = "016.55G Thursday, October 13 2022 16:24:34";
  D[1].send = "022.25G Thursday, October 13 2022 16:24:34";
  D[2].send = "035.17G Thursday, October 13 2022 16:24:34";
  D[3].send = "025.45G Thursday, October 13 2022 16:24:34";
  D[4].send = "032.23G Thursday, October 13 2022 16:24:34";
}

void loop() {

  if (!ISR_flag) {
    get_IMU();
  } else if (ISR_flag) {
    restartAdvertising();
    // bleSend();
    // Serial.println("Resending the data");
    delay(500);
  } else {
    //Do Nothing
  }
}
