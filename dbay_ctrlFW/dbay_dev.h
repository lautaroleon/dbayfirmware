#include <stdlib.h>
#include <stdio.h>
#include <SPI.h>
#include <Wire.h>
#include "errno.h"
#include <stdint.h>
#include <math.h>
#include <Arduino.h>
#include "PCA9557.h"
#include <errno.h>


#ifndef __DBAY_DEV_H__
#define __DBAY_DEV_H__

enum deviceType{
  NODEV,
  DAC4D, //DAC 4 diff channels (triacs)
  DAC16D,  //DAC 16 differential + 500mA 5V + 8V 1mA + 1 diff ADC
  FAFD,   //4ADC + 4DAC
  HIC4   //4DAC high current
};

class dbayDev {

  public:
    dbayDev(int address, TwoWire *bus);
    ~dbayDev();
    char* deviceTypeToString();
    static deviceType deviceTypeFromString(char* devtypestr);
    virtual int reset();
    
    enum deviceType thisDeviceType;
    bool debug;
    
    PCA9557 *BoardSel;
    
    virtual int SetVoltage(int channel, double voltage){return 0;}
    virtual int SetVoltageDiff(int diffchannel, double voltage){return 0;}
    virtual double ReadVoltage(int channel){return 0;}
    //virtual int SetBase(double voltage){return 0;}

  protected:
    int i2cadress;
    TwoWire *_bus;
    char _err[1024];
  
  private:

  
};


#endif
