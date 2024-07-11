#include "dbay_dev.h"

dbayDev::dbayDev(int address, TwoWire *bus):i2cadress(address), _bus (bus){
  
 /* i2cadress = address;
  _bus = bus;*/
  this->BoardSel = new PCA9557(this->i2cadress, _bus);
}


dbayDev::~dbayDev(){
  
}



int dbayDev::reset(){

  return 0;
  
}

static char* dbayDev::deviceTypeToString(){
  switch(this->thisDeviceType){
    case NODEV: return "NODEV";
    case DAC4D: return "DAC4D";   //DAC 4 diff channels (triacs)
    case DAC16D: return "DAC16D"; //DAC 16 differential + 500mA 5V + 8V 1mA + 1 diff ADC
    case FAFD:  return "FAFD"; //4ADC + 4DAC
    case HIC4: return "HIC4";  //4DAC high current
    default: return "UNKNOWN";
  }
}

deviceType dbayDev::deviceTypeFromString(char* devtypestr){
  if(!strcmp(devtypestr , "NODEV")){
    return deviceType::NODEV;
  }else if (!strcmp(devtypestr, "DAC4D")){
     return deviceType::DAC4D;
  }else if (!strcmp(devtypestr, "DAC16D")){
     return deviceType::DAC16D;
  }else return deviceType::NODEV;
}
