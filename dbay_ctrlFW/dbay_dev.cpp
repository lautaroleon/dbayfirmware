#include "dbay_dev.h"

dbayDev::dbayDev(int address, TwoWire *bus):i2cadress(address), _bus (bus){
  
 /* i2cadress = address;
  _bus = bus;*/
  this->BoardSel = new PCA9557(this->i2cadress, _bus);
}


/*dbayDev::~dbayDev() {
  
}*/



int dbayDev::reset(){

  return 0;
  
}

std::string dbayDev::deviceTypeToString(){
  switch(this->thisDeviceType){
    case NODEV: return "NODEV";
    case DAC4D: return "DAC4D";   //DAC 4 diff channels (triacs)
    case DAC16D: return "DAC16D"; //DAC 16 differential + 500mA 5V + 8V 1mA + 1 diff ADC
    case FAFD:  return "FAFD"; //4ADC + 4DAC
    case HIC4: return "HIC4";  //4DAC high current
    case ADC4D: return "ADC4D";
    case DAC4ETH: return "DAC4ETH";
    default: return "UNKNOWN";
  }
}

deviceType dbayDev::deviceTypeFromString(std::string devtypestr){
  if(!strcmp(devtypestr.c_str() , "NODEV")){
    return deviceType::NODEV;
  }else if (!strcmp(devtypestr.c_str(), "DAC4D")){
     return deviceType::DAC4D;
  }else if (!strcmp(devtypestr.c_str(), "DAC16D")){
     return deviceType::DAC16D;
  }else if (!strcmp(devtypestr.c_str(), "ADC4D")){
    return deviceType::ADC4D;
  }else if (!strcmp(devtypestr.c_str(), "DAC16D")) {
      return deviceType::DAC16D;
  }else if (!strcmp(devtypestr.c_str(), "DAC4ETH")) {
      return deviceType::DAC4ETH;
  }else return deviceType::NODEV;
}
