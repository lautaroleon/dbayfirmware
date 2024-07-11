#include "dbay_4triacDAC.h"

dbay4triacDAC::dbay4triacDAC(int address, TwoWire *bus):dbayDev(address, bus){

 thisDeviceType = DAC4D;
 
 if (!BoardSel->pinMode(dbay4triacDAC_CS,OUTPUT)) {
        sprintf(_err, "Error setting pin mode for address 0x%1x ADC. Aborting...\n", BoardSel->boardN);
        Serial.print(_err);
        //rv = -1;
    }
    if (!BoardSel->pinMode(dbay4triacDAC_CLR,OUTPUT)) {
        sprintf(_err, "Error setting pin mode for address 0x%1x ADC. Aborting...\n", BoardSel->boardN);
        Serial.print(_err);
        //rv = -1;
    }
    if (!BoardSel->pinMode(dbay4triacDAC_LDAC,OUTPUT)) {
        sprintf(_err, "Error setting pin mode for address 0x%1x ADC. Aborting...\n", BoardSel->boardN);
        Serial.print(_err);
        //rv = -1;
    }

    if (!BoardSel->digitalWrite(dbay4triacDAC_CLR, HIGH)) {
          sprintf(_err, "Error setting address pin for board %i HIGH\n", BoardSel->boardN);
          Serial.print(_err);
          //rv = -1;
     }

     if (!BoardSel->digitalWrite(dbay4triacDAC_LDAC, HIGH)) {
          sprintf(_err, "Error setting address pin for board %i HIGH\n", BoardSel->boardN);
          Serial.print(_err);
     }

 this->reset();
  
}

int dbay4triacDAC::SetVoltage(int channel, double voltage){

  if (debug){
    Serial.print(channel);Serial.print("\t");
    Serial.println(voltage);
  }
  if(channel<0 || channel >15){
    Serial.print("DAC channel out of range");
    return -1;
  }
  if(voltage<-10 || voltage >10){
    Serial.print("DAC voltage out of range");
    return -1;
  }
  return (DAC->set_voltage(channel, voltage));

}


int dbay4triacDAC::SetVoltageDiff(int diffchannel, double voltage){

  if (debug){
    Serial.print("diffchannel");Serial.println(diffchannel);
    Serial.print("voltage");Serial.println(voltage);
  }
  if(diffchannel<0 || diffchannel >3){
    Serial.print("DAC diff channel out of range");
    return -1;
  }
  if(voltage<-20 || voltage >20){
    Serial.print("DAC voltage out of range");
    return -1;
  }
  int ret1 = this->SetVoltage((diffchannel*2), voltage/2) ;
  int ret2 = this->SetVoltage((diffchannel*2+1), -1*(voltage/2)); 
  
  return (ret1 && ret2);
}

int dbay4triacDAC::reset(){
  
      
      
      for(int j=0; j<16; j++){
         ditherMode[j] = false;
         DACRange[j] = LTC268X_VOLTAGE_RANGE_M10V_10V;
         ditherPhase[j] = LTC268X_DITH_PHASE_0;
         ditherPeriod[j]= LTC268X_DITH_PERIOD_4;
         clkIn[j]= LTC268X_SOFT_TGL;
         ABReg[j]=LTC268X_SELECT_A_REG;
      }
      int CSpin = dbay4triacDAC_CS;
      
      DAC = new ltc268x(dacid, 
                          PWDdacSett,
                          ditherToggleEN,
                          ditherMode,
                          DACRange,
                          ditherPhase,
                          ditherPeriod,
                          clkIn,
                          ABReg,
                          this->BoardSel,
                          CSpin);
  
   return 0;
}
