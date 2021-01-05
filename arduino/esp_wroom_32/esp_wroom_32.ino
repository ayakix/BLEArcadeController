#include <Arduino.h>
#include "BLEDevice.h"
//#include "BLEScan.h"

#define PIN_SWITCH_1 35
#define PIN_SWITCH_2 32

#define PIN_BUTTON_A 16
#define PIN_BUTTON_B 17
#define PIN_BUTTON_X 18
#define PIN_BUTTON_Y 19

#define PIN_BUZZER 14

#define PIN_STICK_R 27
#define PIN_STICK_D 25
#define PIN_STICK_L 26
#define PIN_STICK_U 33

#define LEDC_CHANNEL 0      // チャンネル
#define LEDC_TIMER_BIT 13
#define LEDC_BASE_FREQ 5000

#define C3 130.8
#define CS3 138.5
#define D3 146.8
#define DS3 155.5
#define E3 164.8
#define F3 174.6
#define FS3 184.9
#define G3 195.9
#define GS3 207.6
#define A3 220
#define AS3 233
#define B3 246.9
#define C4 261.6
#define CS4 277.18
#define D4 293.665
#define DS4 311.127
#define E4 329.63
#define F4 349.228
#define FS4 369.994
#define G4 391.995
#define GS4 415.305
#define A4 440
#define AS4 466.164
#define B4 493.883
#define C5 523.251
#define D5 587.330
#define DS5 622.254
#define E5 659.255
#define F5 698.456
#define FS5 739.989
#define G5 783.991
#define GS5 830.609
#define A5 880.000
#define AS5 932.328
#define B5 987.767
#define C6 1046.502
#define CS6 1108.731
#define D6 1174.659
#define DS6 1244.508
#define E6 1318.510

#define MODE_CAR 0
#define MODE_TRAIN 1

int lastMode = -1;

#define INDEX_A 0
#define INDEX_B 1
#define INDEX_X 2
#define INDEX_Y 3
#define INDEX_R 4
#define INDEX_D 5
#define INDEX_L 6
#define INDEX_U 7

int pins[8] = {PIN_BUTTON_A, PIN_BUTTON_B, PIN_BUTTON_X, PIN_BUTTON_Y, PIN_STICK_R, PIN_STICK_D, PIN_STICK_L, PIN_STICK_U};
int lastPinValues[] = {1, 1, 1, 1, 1, 1, 1, 1};

#define TRAIN_DEVICE_NAME_HAYABUSA "Scratch12445"
#define TRAIN_DEVICE_NAME_KOMACHI "Scratch12444"
static BLEUUID TRAIN_SERVICE_UUID("7028FF00-0716-4982-A44C-F4961B5FC950");
static BLEUUID TRAIN_CHARACTERISTIC_UUID("70283006-0716-4982-A44C-F4961B5FC950");

#define CAR_DEVICE_NAME "Bluefruit"
static BLEUUID CAR_SERVICE_UUID("EE02AC5B-32A0-0CDD-DB39-5D3AB4336C6D");
static BLEUUID CAR_CHARACTERISTIC_UUID("EE0234A7-32A0-0CDD-DB39-5D3AB4336C6D");

byte lastCmd[] = {0x00, 0x00, 0x00, 0x00, 0x00};
byte CMD_TRAIN_START[]  = {0x01, 0x64, 0x00, 0x00, 0x00};
byte CMD_TRAIN_STOP[]  = {0x01, 0x00, 0x00, 0x00, 0x00};
byte CMD_CAR_BUZZER[]  = {0x00, 0x00, 0x00, 0x00, 0x01};
byte CMD_CAR_FORWARD[] = {0x01, 0xFF, 0x01, 0xFF, 0x00};
byte CMD_CAR_BACK[]    = {0x00, 0xFF, 0x00, 0xFF, 0x00};
byte CMD_CAR_STOP[]    = {0x00, 0x00, 0x00, 0x00, 0x00};
byte CMD_CAR_LEFT[]    = {0x00, 0xFF, 0x01, 0xFF, 0x00};
byte CMD_CAR_RIGHT[]   = {0x01, 0xFF, 0x00, 0xFF, 0x00};

static BLERemoteCharacteristic* pRemoteCharacteristics[2];
static BLEAdvertisedDevice* myDevices[2];
static int scannedCnt = 0;
static int deviceCnt = 0;

static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
    Serial.print("Notify callback for characteristic ");
    Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
    Serial.print(" of data length ");
    Serial.println(length);
    Serial.print("data: ");
    Serial.println((char*)pData);
}

class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
    Serial.println("onConnected");
  }

  void onDisconnect(BLEClient* pclient) {
    Serial.println("onDisconnect");
    lastMode = -1;
    esp_restart();
  }
};

bool connectToServer(int i) {
    Serial.print("Forming a connection to ");
    Serial.println(myDevices[i]->getAddress().toString().c_str());

    int mode = getMode();
    
    BLEClient*  pClient  = BLEDevice::createClient();
    Serial.println(" - Created client");
    
    pClient->setClientCallbacks(new MyClientCallback());
    
    // Connect to the remove BLE Server.
    pClient->connect(myDevices[i]);  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
    Serial.println(" - Connected to server");

    // Obtain a reference to the service we are after in the remote BLE server.

    BLEUUID serviceUUID = (mode == MODE_CAR) ? CAR_SERVICE_UUID : TRAIN_SERVICE_UUID;
    BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
    if (pRemoteService == nullptr) {
      Serial.print("Failed to find our service UUID: ");
      Serial.println(serviceUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our service");

    // Obtain a reference to the characteristic in the service of the remote BLE server.
    BLEUUID CharacteristicUUID = (mode == MODE_CAR) ? CAR_CHARACTERISTIC_UUID : TRAIN_CHARACTERISTIC_UUID;
    pRemoteCharacteristics[i] = pRemoteService->getCharacteristic(CharacteristicUUID);
    if (pRemoteCharacteristics[i] == nullptr) {
      Serial.print("Failed to find our characteristic UUID: ");
      Serial.println(CharacteristicUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our characteristic");

    // Read the value of the characteristic.
    if (pRemoteCharacteristics[i]->canRead()) {
      std::string value = pRemoteCharacteristics[i]->readValue();
      Serial.print("The characteristic value was: ");
      Serial.println(value.c_str());
    }

    if (pRemoteCharacteristics[i]->canNotify())
      pRemoteCharacteristics[i]->registerForNotify(notifyCallback);

    return true;
}
/**
 * Scan for BLE servers and find the first one that advertises the service we are looking for.
 */
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
 /**
   * Called for each advertising BLE server.
   */
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertisedDevice.toString().c_str());

    if ((advertisedDevice.haveName() && advertisedDevice.getName() == CAR_DEVICE_NAME) || (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(TRAIN_SERVICE_UUID))) {
      if (scannedCnt < deviceCnt) {
        myDevices[scannedCnt] = new BLEAdvertisedDevice(advertisedDevice);
        scannedCnt++;
      }
      
      if (scannedCnt >= deviceCnt) {
        BLEDevice::getScan()->stop();
      }
    } // Found our server
  } // onResult
}; // MyAdvertisedDeviceCallbacks

void playFF(){
  int beat = 500;
  ledcWriteTone(LEDC_CHANNEL, D5);
  delay(beat / 3);
  ledcWriteTone(LEDC_CHANNEL, D5);
  delay(beat / 3);
  ledcWriteTone(LEDC_CHANNEL, D5);
  delay(beat / 3);
  ledcWriteTone(LEDC_CHANNEL, D5);
  delay(beat);
  ledcWriteTone(LEDC_CHANNEL, AS4);
  delay(beat);
  ledcWriteTone(LEDC_CHANNEL, C5);
  delay(beat);
  ledcWriteTone(LEDC_CHANNEL, D5);
  delay(beat / 3);
  ledcWriteTone(LEDC_CHANNEL, 0);
  delay(beat / 3);
  ledcWriteTone(LEDC_CHANNEL, C5);
  delay(beat / 3);
  ledcWriteTone(LEDC_CHANNEL, D5);
  delay(beat * 1.5);
  ledcWriteTone(LEDC_CHANNEL, 0);
}

void playZelda() {
  int beat = 150;
  
  ledcWriteTone(LEDC_CHANNEL, 3136); // ソ
  delay(beat);
  ledcWriteTone(LEDC_CHANNEL, 2960); // ♯ファ
  delay(beat);
  ledcWriteTone(LEDC_CHANNEL, 2489); // ♯レ
  delay(beat);
  ledcWriteTone(LEDC_CHANNEL, 1760); // ラ
  delay(beat);
  ledcWriteTone(LEDC_CHANNEL, 1661); // ♯ソ
  delay(beat);
  ledcWriteTone(LEDC_CHANNEL, 2637); // ミ
  delay(beat);
  ledcWriteTone(LEDC_CHANNEL, 3322); // ♯ソ
  delay(beat);
  ledcWriteTone(LEDC_CHANNEL, 4186); // ド
  delay(beat);
  ledcWriteTone(LEDC_CHANNEL, 0);
}

void playDQ() {
  int beat = 150;
  ledcWriteTone(LEDC_CHANNEL, F5);
  delay(beat - 10);
  ledcWriteTone(LEDC_CHANNEL, 0);
  delay(10);
  
  ledcWriteTone(LEDC_CHANNEL, F5);
  delay(beat - 10);
  ledcWriteTone(LEDC_CHANNEL, 0);
  delay(10);
  
  ledcWriteTone(LEDC_CHANNEL, F5);
  delay(beat - 10);
  ledcWriteTone(LEDC_CHANNEL, 0);
  delay(10);
  
  ledcWriteTone(LEDC_CHANNEL, F5);
  delay(beat - 10);
  ledcWriteTone(LEDC_CHANNEL, 0);
  delay(10);
  delay(beat);
  
  ledcWriteTone(LEDC_CHANNEL, DS5);
  delay(beat);
  ledcWriteTone(LEDC_CHANNEL, 0);
  delay(beat);
  ledcWriteTone(LEDC_CHANNEL, G5);
  delay(beat);
  ledcWriteTone(LEDC_CHANNEL, 0);
  delay(beat);
  ledcWriteTone(LEDC_CHANNEL, F5);
  delay(beat * 2.5);
  ledcWriteTone(LEDC_CHANNEL, 0);
}

void playMarioCoin() {
  int beat = 100;
  ledcWriteTone(LEDC_CHANNEL, B5);
  delay(beat);
  ledcWriteTone(LEDC_CHANNEL, E6);
  delay(beat * 3);
  ledcWriteTone(LEDC_CHANNEL, 0);
}

// 地上BGM
void playMarioBGM1() {
  int beat = 180;
  ledcWriteTone(LEDC_CHANNEL, E5);
  delay(beat - 10);
  ledcWriteTone(LEDC_CHANNEL, 0);
  delay(10);

  ledcWriteTone(LEDC_CHANNEL, E5);
  delay(beat - 10);
  ledcWriteTone(LEDC_CHANNEL, 0);
  delay(10);

  delay(beat);

  ledcWriteTone(LEDC_CHANNEL, E5);
  delay(beat - 10);
  ledcWriteTone(LEDC_CHANNEL, 0);
  delay(10);

  delay(beat);

  ledcWriteTone(LEDC_CHANNEL, C5);
  delay(beat - 10);
  ledcWriteTone(LEDC_CHANNEL, 0);
  delay(10);

  ledcWriteTone(LEDC_CHANNEL, E5);
  delay(beat * 2 - 10);
  ledcWriteTone(LEDC_CHANNEL, 0);
  delay(10);

  ledcWriteTone(LEDC_CHANNEL, G5);
  delay(beat * 2 - 10);
  ledcWriteTone(LEDC_CHANNEL, 0);
  delay(10);

  delay(beat * 2);
  
  ledcWriteTone(LEDC_CHANNEL, G4);
  delay(beat * 2 - 10);
  ledcWriteTone(LEDC_CHANNEL, 0);
  delay(10);
}

// 地下BGM
void playMarioBGM2() {
  int beat = 180;
  for (int i = 0; i < 2; i++) {
    ledcWriteTone(LEDC_CHANNEL, C4);
    delay(beat);
    ledcWriteTone(LEDC_CHANNEL, C5);
    delay(beat);
    ledcWriteTone(LEDC_CHANNEL, A3);
    delay(beat);
    ledcWriteTone(LEDC_CHANNEL, A4);
    delay(beat);
    ledcWriteTone(LEDC_CHANNEL, AS3);
    delay(beat);
    ledcWriteTone(LEDC_CHANNEL, AS4);
    delay(beat);
    ledcWriteTone(LEDC_CHANNEL, 0);
    if (i == 0) {
      delay(beat*6);
    }
  }
}

// 無敵BGM
void playMarioBGM3() {
  int beat = 90;
  int hz;
  for (int i = 0; i < 2; i++) {
    if (i == 0) {
      hz = C5;
    } else {
      hz = B4;
    }
    
    ledcWriteTone(LEDC_CHANNEL, hz);
    delay(beat * 2 - 10);
    ledcWriteTone(LEDC_CHANNEL, 0);
    delay(10);

    ledcWriteTone(LEDC_CHANNEL, hz);
    delay(beat * 2 - 10);
    ledcWriteTone(LEDC_CHANNEL, 0);
    delay(10);

    ledcWriteTone(LEDC_CHANNEL, hz);
    delay(beat * 2);
    
    ledcWriteTone(LEDC_CHANNEL, 0);
    delay(beat);

    ledcWriteTone(LEDC_CHANNEL, hz);
    delay(beat);
    
    ledcWriteTone(LEDC_CHANNEL, 0);
    delay(beat);

    ledcWriteTone(LEDC_CHANNEL, hz);
    delay(beat * 2);
    
    ledcWriteTone(LEDC_CHANNEL, 0);
    delay(beat);

    ledcWriteTone(LEDC_CHANNEL, hz);
    delay(beat * 2 - 10);
    ledcWriteTone(LEDC_CHANNEL, 0);
    delay(10);

    ledcWriteTone(LEDC_CHANNEL, hz);
    delay(beat * 2 - 10);
    ledcWriteTone(LEDC_CHANNEL, 0);
    delay(10);
  }
}

int getMode() {
  int mode;
  
  int switch1Value = digitalRead(PIN_SWITCH_1);
  int switch2Value = digitalRead(PIN_SWITCH_2);

  if (switch1Value == LOW) {
    mode = MODE_CAR;
  } else if (switch2Value == LOW) {
    mode = MODE_TRAIN;
  } else {
    mode = MODE_CAR;
  }
  return mode;
}

void debug() {
  char buttons[] = {'A', 'B', 'X', 'Y', 'R', 'D', 'L', 'U'};
  int i;
  for (i = 0; i < 8; i++) {
    int value = digitalRead(pins[i]);
    Serial.print(buttons[i]);
    Serial.print(": ");
    Serial.print(value);
    Serial.print("\t");
  }
  Serial.println("");
  //delay(1000);
}

void initialize() {
  Serial.println("initialize");
  
  scannedCnt = 0;
  int mode = getMode();
  if (mode == MODE_CAR) {
    deviceCnt = 1;
    Serial.println("Mode: Car");
  }
  if (mode == MODE_TRAIN) {
    deviceCnt = 2;
    Serial.println("Mode: Train");
  }

  for (int i = 0; i < deviceCnt; i++) {
    pRemoteCharacteristics[i] = nullptr;
    myDevices[i] = nullptr;
  }
}

void setup() {
  Serial.begin(74880);

  pinMode(PIN_SWITCH_1, INPUT);
  pinMode(PIN_SWITCH_2, INPUT);
  
  for (int i = 0; i < sizeof(pins); i++) {
    pinMode(pins[i], INPUT);
  }
  
  // 音を鳴らす準備
  pinMode(PIN_BUZZER, OUTPUT);
  ledcSetup(LEDC_CHANNEL, LEDC_BASE_FREQ, LEDC_TIMER_BIT);
  ledcAttachPin(PIN_BUZZER, LEDC_CHANNEL);

  initialize();
  lastMode = getMode();
  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 5 seconds.
  BLEDevice::init("");
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(5, false);
}

void cmd(BLERemoteCharacteristic* pRemoteCharacteristic, byte cmd[]) {
  if (pRemoteCharacteristic == nullptr) {
    return;
  }
  if (!memcmp(cmd, lastCmd, sizeof(cmd))) {
    return;
  }
  
  pRemoteCharacteristic->writeValue(cmd, 5);
  memcpy(lastCmd, cmd, sizeof(cmd));
  delay(50);
}

void driveCar(int pinValues[]) {
  if (pinValues[INDEX_U] == LOW) {
    cmd(pRemoteCharacteristics[0], CMD_CAR_FORWARD);
  } else if (pinValues[INDEX_D] == LOW) {
    cmd(pRemoteCharacteristics[0], CMD_CAR_BACK);
  } else if (pinValues[INDEX_R] == LOW) {
    cmd(pRemoteCharacteristics[0], CMD_CAR_RIGHT);
  } else if (pinValues[INDEX_L] == LOW) {
    cmd(pRemoteCharacteristics[0], CMD_CAR_LEFT);
  } else {
    cmd(pRemoteCharacteristics[0], CMD_CAR_STOP);
  }

  if (pinValues[INDEX_A] == LOW) {
    playMarioBGM2();
  }
  if (pinValues[INDEX_B] == LOW) {
    playMarioCoin();
  }
  if (pinValues[INDEX_X] == LOW) {
    playMarioBGM1();
  }
  if (pinValues[INDEX_Y] == LOW) {
    playMarioBGM3();
  }
}

void driveTrain(int pinValues[]) {
  int komachiIndex = -1;
  int hayabusaIndex = -1;
  for (int i = 0; i < deviceCnt; i++) {
    if (myDevices[i] != nullptr) {
      if (myDevices[i]->getName() == TRAIN_DEVICE_NAME_KOMACHI) {
        komachiIndex = i;
      }
      if (myDevices[i]->getName() == TRAIN_DEVICE_NAME_HAYABUSA) {
        hayabusaIndex = i;
      }
    }
  }
  
  if (komachiIndex >= 0) {
    if (pinValues[INDEX_A] == LOW) {
      cmd(pRemoteCharacteristics[komachiIndex], CMD_TRAIN_START);
    }
    if (pinValues[INDEX_B] == LOW) {
      cmd(pRemoteCharacteristics[komachiIndex], CMD_TRAIN_STOP);
    } 
  }

  if (hayabusaIndex >= 0) {
    if (pinValues[INDEX_X] == LOW) {
      cmd(pRemoteCharacteristics[hayabusaIndex], CMD_TRAIN_START);
    }
    if (pinValues[INDEX_Y] == LOW) {
      cmd(pRemoteCharacteristics[hayabusaIndex], CMD_TRAIN_STOP);
    }
  }
}

void loop() {
  //debug();
  
  int mode = getMode();
  if (lastMode != mode) {
    initialize();
    BLEDevice::getScan()->start(0);
    lastMode = mode;
  }

  for (int i = 0; i < scannedCnt; i++) {
    if (pRemoteCharacteristics[i] == nullptr) {
      if (connectToServer(i)) {
        Serial.println("connected!");
        playMarioCoin();
      }
    }
  }
  
  int pinValues[8];
  for (int i = 0; i < 8; i++) {
    pinValues[i] = digitalRead(pins[i]);
  }

  if (mode == MODE_CAR) {
    driveCar(pinValues);
  }
  if (mode == MODE_TRAIN) {
    driveTrain(pinValues);
  }
  
  memcpy(lastPinValues, pinValues, sizeof(pinValues));
}
