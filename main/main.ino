#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "PacketIdInfo.h"
#include "esp32_can.h"            // https://github.com/collin80/esp32_can AND https://github.com/collin80/can_common
#include "RunningAverage.h"


uint8_t ratios[] = { 100, 58, 42, 31, 25, 21, 18 };  // Set in the middle INBETWEEN your car's each gears "rpm per kmh" ratios. The default works for many cars, tested on both a BMW M2 DCT and a Peugeot 108 :-D 

#define DEBUG

#define CAN0_RX_GPIO GPIO_NUM_13  // 16 
#define CAN0_TX_GPIO GPIO_NUM_14  // 17

#define ENGINE_RPM                  0x0C
#define VEHICLE_SPEED               0x0D
#define CAN_REQST_ID                0x7DF 
#define CAN_REPLY_ID                0x7E8

 
uint8_t kmh = 0;
uint16_t rpm = 0;
uint8_t gear = 0;
uint16_t ratio;
uint16_t average;
float changes;
#define AVGFILTERSIZE 4

RunningAverage myRA(AVGFILTERSIZE);


#define SERVICE_UUID "00001ff8-0000-1000-8000-00805f9b34fb"
#define CANBUS_MAIN_UUID "0001"
#define CANBUS_FILTER_UUID "0002"

BLEServer* pServer;
BLEAdvertising* pAdvertising;

BLECharacteristic* canBusMainCharacteristic;
BLECharacteristic* canBusFilterCharacteristic;

PacketIdInfo canBusPacketIdInfo;
bool canBusAllowUnknownPackets = false;
uint32_t canBusLastNotifyMs = 0;
boolean isCanBusConnected = false;


uint8_t tempData[20];
unsigned long lastSendTime = 0;
unsigned long lastNotifyTime = 0;
const long sendInterval = 50; // 20 Hz


////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class FilterCallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) {
    String value = pCharacteristic->getValue();
    if (value.length() < 1) return;
    uint8_t command = value[0];
    switch (command) {
      case 0x00: // DENY_ALL
        if (value.length() == 1) {
          canBusPacketIdInfo.reset();
          canBusAllowUnknownPackets = false;
        }
        break;
      case 0x01: // ALLOW_ALL
        if (value.length() == 3) {
          canBusPacketIdInfo.reset();
          canBusPacketIdInfo.setDefaultNotifyInterval(sendInterval); 
          canBusAllowUnknownPackets = true;
        }
        break;
      case 0x02: // ADD_PID
        if (value.length() == 7) {
          uint16_t notifyIntervalMs = value[1] << 8 | value[2];
          uint32_t pid = value[3] << 24 | value[4] << 16 | value[5] << 8 | value[6];
          canBusPacketIdInfo.setNotifyInterval(pid, sendInterval); 
        }
        break;
    }
  }
};



////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void startAdvertising() {
  pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinInterval(0x20); 
  pAdvertising->setMaxInterval(0x40); 
  pAdvertising->start();
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("Booting Rejsa.nu OBD2 Gear translator...");
  myRA.clear();
  startRC();
  startCAN();
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void startRC() {
  BLEDevice::init("RC DIY GEAR");
  pServer = BLEDevice::createServer();
  BLEService* pService = pServer->createService(SERVICE_UUID);
  canBusMainCharacteristic = pService->createCharacteristic(
    CANBUS_MAIN_UUID,
    BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ
  );
  canBusMainCharacteristic->addDescriptor(new BLE2902());
  canBusFilterCharacteristic = pService->createCharacteristic(
    CANBUS_FILTER_UUID,
    BLECharacteristic::PROPERTY_WRITE
  );
  canBusFilterCharacteristic->setCallbacks(new FilterCallback());
  pService->start();
  startAdvertising();
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void sendGearData() {
  unsigned long currentTime = millis();
  if (currentTime - lastNotifyTime < sendInterval) return; 
  lastNotifyTime = currentTime;

  PacketIdInfoItem* infoItem;
  uint32_t packetId;

  packetId = 16;
  tempData[4] = (uint8_t)gear;
  ((uint32_t*)tempData)[0] = packetId;
  infoItem = canBusPacketIdInfo.findItem(packetId, canBusAllowUnknownPackets);
  if (infoItem && infoItem->shouldNotify()) {
    canBusMainCharacteristic->setValue(tempData, 5);
    canBusMainCharacteristic->notify();
    infoItem->markNotified();
  }
  packetId = 17;
  tempData[4] = (uint8_t)kmh/3.6;
  ((uint32_t*)tempData)[0] = packetId;
  infoItem = canBusPacketIdInfo.findItem(packetId, canBusAllowUnknownPackets);
  if (infoItem && infoItem->shouldNotify()) {
    canBusMainCharacteristic->setValue(tempData, 5);
    canBusMainCharacteristic->notify();
    infoItem->markNotified();
  }
  packetId = 18;
  uint16_t rpmInt = (uint16_t)rpm;
  uint8_t rpmHigh = (rpmInt >> 8) & 0xFF;
  uint8_t rpmLow = rpmInt & 0xFF;
  tempData[4] = rpmHigh;
  tempData[5] = rpmLow;
  ((uint32_t*)tempData)[0] = packetId;
  infoItem = canBusPacketIdInfo.findItem(packetId, canBusAllowUnknownPackets);
  if (infoItem && infoItem->shouldNotify()) {
    canBusMainCharacteristic->setValue(tempData, 6);
    canBusMainCharacteristic->notify();
    infoItem->markNotified();
  }
  packetId = 19;
  tempData[4] = (uint8_t)ratio;
  ((uint32_t*)tempData)[0] = packetId;
  infoItem = canBusPacketIdInfo.findItem(packetId, canBusAllowUnknownPackets);
  if (infoItem && infoItem->shouldNotify()) {
    canBusMainCharacteristic->setValue(tempData, 5);
    canBusMainCharacteristic->notify();
    infoItem->markNotified();
  }
  packetId = 20;
  tempData[4] = (uint8_t)abs((int16_t)100*changes);
  ((uint32_t*)tempData)[0] = packetId;
  infoItem = canBusPacketIdInfo.findItem(packetId, canBusAllowUnknownPackets);
  if (infoItem && infoItem->shouldNotify()) {
    canBusMainCharacteristic->setValue(tempData, 5);
    canBusMainCharacteristic->notify();
    infoItem->markNotified();
  }
  
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop() {

  static uint16_t prevratio = 0;
  
  static uint8_t prev = 0;
  static uint8_t next = 0;
  static uint8_t nextcntr = 0;
  
  unsigned long currentTime = millis();
  if (currentTime - lastSendTime >= sendInterval) {

    requestCar();     // REPLIES ARE HANDLED BY CALLBACK. DATA PUT IN GLOBAL VARS kmh AND rpm
    delay(10);        // ALLOW TIME FOR REPLY IN CALLBACK

    prevratio = ratio;
    if ( kmh>0 ) { ratio = (uint16_t) (rpm/kmh); }
    myRA.addValue((ratio-prevratio));
    changes = myRA.getAverage();
  
    prev = next;
    if ( kmh<1 )                            { next = 0; }
    else if ( changes>0.1 || changes<-0.1 ) { int noop; }     // catches when clutch is pressed and ratio value has changes. Smaller changes can equal noise when in gear but also when by *chance* speed and rpm happen to match even when clutch is pressed
    else if ( ratio>ratios[0] )             { next = 1; } 
    else if ( ratio>ratios[1] )             { next = 2; } 
    else if ( rpm<1200 )                    { int noop; }     // when switching gears very slow rpm goes slowly towards idle and often happen by chance to match change ratio between rpm and kmh resulting in wrong (high) gears shown. This disables gear matching when rpm is getting close to idle.
    else if ( ratio>ratios[2] )             { next = 3; } 
    else if ( ratio>ratios[3] )             { next = 4; } 
    else if ( ratio>ratios[4] )             { next = 5; } 
    else if ( ratio>ratios[5] )             { next = 6; } 
    else if ( ratio>ratios[6] )             { next = 7; } 
    
    if (next==prev) { nextcntr++; }    
    else            { nextcntr = 0; }
    if (nextcntr>2) { gear = next; }    // Filter away single "glitches" of ratio (gear) matches that can be erroneous due to clutch being pushed down
  
    printData();      // PRINT DATA ON SERIAL USB
    if (isCanBusConnected) {
      sendGearData(); // SEND DATA TO RACECHRONO
    }
    lastSendTime = currentTime;
  }

  if (!isCanBusConnected && pServer->getConnectedCount() > 0) {
    isCanBusConnected = true;
    Serial.println("BLE connected");
    canBusPacketIdInfo.reset();
  } else if (isCanBusConnected && pServer->getConnectedCount() == 0) {
    isCanBusConnected = false;
    Serial.println("BLE disconnected");
    pAdvertising->stop();
    delay(100);
    startAdvertising();
  }
}




////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void startCAN(void) {
  CAN0.setCANPins(CAN0_RX_GPIO, CAN0_TX_GPIO);
  if(CAN0.begin(500000)) { Serial.println("CAN0 (car): Init OK");  } 
  else                   { Serial.println("CAN0 (car): Init Failed");  }
  CAN0.watchFor(CAN_REPLY_ID);
  CAN0.setCallback(0, fromCar);
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void requestCar(void) {    

  CAN_FRAME outgoing;

  outgoing.id = CAN_REQST_ID;
  outgoing.length = 8;
  outgoing.extended = 0;
  outgoing.rtr = 0;
  outgoing.data.uint8[0] = 0x02;  
  outgoing.data.uint8[1] = 0x01;  
  outgoing.data.uint8[3] = 0x00;
  outgoing.data.uint8[4] = 0x00;  
  outgoing.data.uint8[5] = 0x00;  
  outgoing.data.uint8[6] = 0x00;  
  outgoing.data.uint8[7] = 0x00;  

  outgoing.data.uint8[2] = VEHICLE_SPEED;  
  CAN0.sendFrame(outgoing);

  outgoing.data.uint8[2] = ENGINE_RPM;
  CAN0.sendFrame(outgoing);
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//   * * * * CALLBACK * * * * 

void fromCar(CAN_FRAME *from_car) {
  
  if (from_car->data.uint8[2]==ENGINE_RPM) {
    uint8_t rpmOBDH = from_car->data.uint8[3];
    uint8_t rpmOBDL = from_car->data.uint8[4];
    rpm = (uint16_t) ((256*rpmOBDH) + rpmOBDL)/(float)4.0;
  }
  if (from_car->data.uint8[2]==VEHICLE_SPEED) { 
    kmh = from_car->data.uint8[3]; 
  }
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void printData(void) {

#ifdef DEBUG
  Serial.printf("%3d kmh    ", kmh);
  Serial.printf("%5d rpm    ", rpm);
  Serial.printf("%3d ratio    ", ratio);
  Serial.printf("%6.3f change    ", changes);
  Serial.printf("%3d gear    ", gear);
  Serial.println();
#endif
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////