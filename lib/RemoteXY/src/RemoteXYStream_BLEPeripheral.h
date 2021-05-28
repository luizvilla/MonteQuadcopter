#ifndef RemoteXYStream_BLEPeripheral_h
#define RemoteXYStream_BLEPeripheral_h
                          
#if defined(_BLE_PERIPHERAL_H_) 

#include "RemoteXYStream.h"

#define REMOTEXYCOMM_BLEPERIPHERAL__SEND_BUFFER_SIZE 20
#define REMOTEXYCOMM_BLEPERIPHERAL__RECEIVE_BUFFER_SIZE 1024
#define REMOTEXYCOMM_BLEPERIPHERAL__SERVICE_UUID             "ffe0" // UART service UUID
#define REMOTEXYCOMM_BLEPERIPHERAL__CHARACTERISTIC_UUID_RXTX "ffe1"    

class CRemoteXYStream_BLEPeripheral : public CRemoteXYStream, public BLEPeripheral {

  private:
  BLEService *pService;
  BLECharacteristic *pRxTxCharacteristic;
  
  uint8_t sendBuffer[REMOTEXYCOMM_BLEPERIPHERAL__SEND_BUFFER_SIZE];
  uint16_t sendBufferCount;
  uint16_t sendBytesAvailable;

  uint8_t receiveBuffer[REMOTEXYCOMM_BLEPERIPHERAL__RECEIVE_BUFFER_SIZE];
  uint16_t receiveBufferStart;
  uint16_t receiveBufferPos;
  uint16_t receiveBufferCount;
  
  
  public:
  CRemoteXYStream_BLEPeripheral (const char * _bleDeviceName) : CRemoteXYStream () { 
#if defined(REMOTEXY__DEBUGLOG)
    RemoteXYDebugLog.write("Init NRF BLE on chip");
#endif

    receiveBufferCount = 0;        
    receiveBufferStart = 0;
    receiveBufferPos = 0;
    receiveBufferCount = 0;  
    
    pService = new BLEService (REMOTEXYCOMM_BLEPERIPHERAL__SERVICE_UUID);
    pRxTxCharacteristic = new BLECharacteristic(
      REMOTEXYCOMM_BLEPERIPHERAL__CHARACTERISTIC_UUID_RXTX, 
      BLERead | BLEWriteWithoutResponse | BLENotify, BLE_ATTRIBUTE_MAX_VALUE_LENGTH
    );
    
    setLocalName(_bleDeviceName);
    setDeviceName(_bleDeviceName);
    setAppearance(0x0080);
    
    addAttribute (*pService);
    setAdvertisedServiceUuid (pService->uuid());
    addAttribute (*pRxTxCharacteristic);

    
    BLEPeripheral::begin();
    
#if defined(REMOTEXY__DEBUGLOG)
    RemoteXYDebugLog.write("BLE started");
#endif  

  }              
  
  
  void BLEDeviceConnected(BLEDevice& device, const unsigned char* address) override {
#if defined(REMOTEXY__DEBUGLOG)
    RemoteXYDebugLog.write("BLE client connected");
#endif
    receiveBufferStart = 0;
    receiveBufferPos = 0;
    receiveBufferCount = 0;  
  }
  
  void BLEDeviceDisconnected(BLEDevice& device) {
#if defined(REMOTEXY__DEBUGLOG)
    RemoteXYDebugLog.write("BLE client disconnected");
#endif
    receiveBufferCount = 0;
  }

  void BLEDeviceCharacteristicValueChanged (BLEDevice& device, BLECharacteristic& characteristic, const unsigned char* data, unsigned char size) override {
    for (uint16_t i = 0; i < size; i++) {                
      receiveBuffer[receiveBufferPos++] =  data[i];
      if (receiveBufferPos >= REMOTEXYCOMM_BLEPERIPHERAL__RECEIVE_BUFFER_SIZE) receiveBufferPos=0; 
      if (receiveBufferCount < REMOTEXYCOMM_BLEPERIPHERAL__RECEIVE_BUFFER_SIZE) receiveBufferCount++;
      else {
        receiveBufferStart++;
        if (receiveBufferStart >= REMOTEXYCOMM_BLEPERIPHERAL__RECEIVE_BUFFER_SIZE) receiveBufferStart=0;
      }       
    } 
  }
    
  void startWrite (uint16_t len) override {
    sendBytesAvailable = len;
    sendBufferCount = 0;
  }    
    
  void write (uint8_t b) override {
    sendBuffer[sendBufferCount++] = b;
    sendBytesAvailable--;
    if ((sendBufferCount == REMOTEXYCOMM_BLEPERIPHERAL__SEND_BUFFER_SIZE) || (sendBytesAvailable == 0)) {
      BLEPeripheral::poll();
      if (pRxTxCharacteristic->subscribed() == false) {
#if defined(REMOTEXY__DEBUGLOG)
        RemoteXYDebugLog.write("BLE: tx characteristic not subscribed");
#endif
        return;
      }
      bool ret;
      ret = pRxTxCharacteristic->setValue((uint8_t *)sendBuffer, sendBufferCount);
      BLEPeripheral::poll();
      sendBufferCount = 0;
    }
  }     

  void handler () override {   
    BLEPeripheral::poll();
    if (receiveBufferCount>0) {
      while (receiveBufferCount) {     
        notifyReadByteListener (receiveBuffer[receiveBufferStart++]);
        if (receiveBufferStart >= REMOTEXYCOMM_BLEPERIPHERAL__RECEIVE_BUFFER_SIZE) receiveBufferStart=0;
        receiveBufferCount--;
      }
    }
  } 
};





#endif  // _BLE_PERIPHERAL_H_

#endif //RemoteXYStream_BLEPeripheral_h