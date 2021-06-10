#ifndef RemoteXYStream_HardSerial_h
#define RemoteXYStream_HardSerial_h

#include "RemoteXYStream.h"


class CRemoteXYStream_HardSerial : public CRemoteXYStream {
  
  private:
  HardwareSerial * serial;
  
  public:
  CRemoteXYStream_HardSerial (HardwareSerial * _serial, long _serialSpeed) : CRemoteXYStream () {
    serial = _serial;
    
    #if defined(REMOTEXY_MODE__BLE_NANO)
      Serial.begin (_serialSpeed);
    #else
      serial->begin (_serialSpeed);
    #endif

#if defined(REMOTEXY__DEBUGLOG)
    RemoteXYDebugLog.write("Init hardware serial ");
    RemoteXYDebugLog.writeAdd(_serialSpeed);
    RemoteXYDebugLog.writeAdd(" baud");
#endif
  }              
  
#if defined(REMOTEXY_MODE__BLE_NANO)
  void handler () override {   
    while (Serial.available ()) notifyReadByteListener (Serial.read ());
  }

  void write (uint8_t byte) override {
    Serial.write (byte);
  }

#else  
  void handler () override {   
    while (serial->available ()) notifyReadByteListener (serial->read ());
  }

  void write (uint8_t byte) override {
    serial->write (byte);
  }
#endif
  
};




#endif //RemoteXYStream_HardSerial_h
