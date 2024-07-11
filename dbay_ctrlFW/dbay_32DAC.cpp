#include "dbay_32DAC.h"

dbay32DAC::dbay32DAC(int address, TwoWire *bus):dbayDev(address, bus){

  thisDeviceType = DAC16D;

   
   if (!BoardSel->pinMode(dbay32DAC_CS1,OUTPUT)) {
        sprintf(_err, "Error setting pin mode for address 0x%1x ADC. Aborting...\n", BoardSel->boardN);
        Serial.print(_err);
        //rv = -1;
    }
    if (!BoardSel->pinMode(dbay32DAC_CLR1,OUTPUT)) {
        sprintf(_err, "Error setting pin mode for address 0x%1x ADC. Aborting...\n", BoardSel->boardN);
        Serial.print(_err);
        //rv = -1;
    }
    if (!BoardSel->pinMode(dbay32DAC_LDAC1,OUTPUT)) {
        sprintf(_err, "Error setting pin mode for address 0x%1x ADC. Aborting...\n", BoardSel->boardN);
        Serial.print(_err);
        //rv = -1;
    }

    if (!BoardSel->digitalWrite(dbay32DAC_CLR1, HIGH)) {
          sprintf(_err, "Error setting address pin for board %i HIGH\n", BoardSel->boardN);
          Serial.print(_err);
          //rv = -1;
     }

     if (!BoardSel->digitalWrite(dbay32DAC_LDAC1, HIGH)) {
          sprintf(_err, "Error setting address pin for board %i HIGH\n", BoardSel->boardN);
          Serial.print(_err);
     }

   
   if (!BoardSel->pinMode(dbay32DAC_CS2,OUTPUT)) {
        sprintf(_err, "Error setting pin mode for address 0x%1x ADC. Aborting...\n", BoardSel->boardN);
        Serial.print(_err);
        //rv = -1;
    }
    if (!BoardSel->pinMode(dbay32DAC_CLR2,OUTPUT)) {
        sprintf(_err, "Error setting pin mode for address 0x%1x ADC. Aborting...\n", BoardSel->boardN);
        Serial.print(_err);
        //rv = -1;
    }
    if (!BoardSel->pinMode(dbay32DAC_LDAC2,OUTPUT)) {
        sprintf(_err, "Error setting pin mode for address 0x%1x ADC. Aborting...\n", BoardSel->boardN);
        Serial.print(_err);
        //rv = -1;
    }

    if (!BoardSel->digitalWrite(dbay32DAC_CLR2, HIGH)) {
          sprintf(_err, "Error setting address pin for board %i HIGH\n", BoardSel->boardN);
          Serial.print(_err);
          //rv = -1;
     }

     if (!BoardSel->digitalWrite(dbay32DAC_LDAC2, HIGH)) {
          sprintf(_err, "Error setting address pin for board %i HIGH\n", BoardSel->boardN);
          Serial.print(_err);
     }


   if (!BoardSel->pinMode(dbay32DAC_DACSYNC,OUTPUT)) {
        sprintf(_err, "Error setting pin mode for address 0x%1x ADC. Aborting...\n", BoardSel->boardN);
        Serial.print(_err);
        //rv = -1;
    }

    if (!BoardSel->digitalWrite(dbay32DAC_DACSYNC, HIGH)) {
          sprintf(_err, "Error setting address pin for board %i HIGH\n", BoardSel->boardN);
          Serial.print(_err);
          //rv = -1;
     }

    
   if (!BoardSel->pinMode(dbay32DAC_ADC_CS,OUTPUT)) {
        sprintf(_err, "Error setting pin mode for address 0x%1x ADC. Aborting...\n", BoardSel->boardN);
        Serial.print(_err);
        //rv = -1;
    }

    if (!BoardSel->digitalWrite(dbay32DAC_ADC_CS, HIGH)) {
          sprintf(_err, "Error setting address pin for board %i HIGH\n", BoardSel->boardN);
          Serial.print(_err);
          //rv = -1;
     }
     
 this->reset();
  
}

dbay32DAC::~dbay32DAC(){}

int dbay32DAC::reset(){
      int CSPin0 = dbay32DAC_DACSYNC;
      
      DAC4ch = new DAC_AD5684(1, this->BoardSel,  CSPin0);
        
      for(int j=0; j<16; j++){
         ditherMode[j] = false;
         DACRange[j] = LTC268X_VOLTAGE_RANGE_M10V_10V;
         ditherPhase[j] = LTC268X_DITH_PHASE_0;
         ditherPeriod[j]= LTC268X_DITH_PERIOD_4;
         clkIn[j]= LTC268X_SOFT_TGL;
         ABReg[j]=LTC268X_SELECT_A_REG;
      }
      int CSpin1 = dbay32DAC_CS1;
      int CSpin2 = dbay32DAC_CS2;
      
      DAC1 = new ltc268x(dacid, 
                          PWDdacSett,
                          ditherToggleEN,
                          ditherMode,
                          DACRange,
                          ditherPhase,
                          ditherPeriod,
                          clkIn,
                          ABReg,
                          this->BoardSel,
                          CSpin1);

        DAC2 = new ltc268x(dacid, 
                          PWDdacSett,
                          ditherToggleEN,
                          ditherMode,
                          DACRange,
                          ditherPhase,
                          ditherPeriod,
                          clkIn,
                          ABReg,
                          this->BoardSel,
                          CSpin2);
                          
        int adcCS = dbay32DAC_ADC_CS;
        
        adc = new Ad7124Chip();
        adc->begin(this->BoardSel, adcCS);
        // Setting the configuration 0:
        // - use of the internal reference voltage 2.5V
        // - gain of 1 for a bipolar measurement +/- 2.5V
        adc->setConfig (0, Ad7124::RefInternal, Ad7124::Pga1, true);
          // Setting channel 0 using pins AIN1(+)/AIN0(-)
        adc->setChannel (0, 0, Ad7124::AIN1Input, Ad7124::AIN0Input);
          // Configuring ADC in Full Power Mode (Fastest)
        adc->setAdcControl (Ad7124::StandbyMode, Ad7124::FullPower, true);
  
   return 0;
}


int dbay32DAC::SetVoltage (int channel, double voltage){


  if (debug){
    Serial.print(channel);Serial.print("\t");
    Serial.println(voltage);
  }

  if(voltage<-10 || voltage >10){
    Serial.print("DAC voltage out of range");
    return -1;
  }else if (channel == -1){ //channel -1 is assign on this board to the 8V
    DAC4ch->set_V(1,channel,(voltage/dbay32DAC_OPAMPGAIN));
  }else if(channel<0 || channel >31){
    Serial.print("DAC channel out of range");
    return -1;
  }else{
    if(channel >= 0 || channel <= 15)return (DAC1->set_voltage(channel, voltage));
    else if(channel > 15 || channel < 32)return (DAC2->set_voltage(channel-16, voltage));
  }
  
}

int dbay32DAC::SetVoltageDiff(int diffchannel, double voltage) {
  if (debug){
    Serial.print("diffchannel");Serial.println(diffchannel);
    Serial.print("voltage");Serial.println(voltage);
  }
  if(voltage<-20 || voltage >20){
    Serial.print("DAC voltage out of range");
    return -1;
  }
  if(diffchannel<0 || diffchannel >15){
    Serial.print("DAC channel out of range");
    return -1;
  }
  
  int ret1 = this->SetVoltage((diffchannel*2), voltage/2) ;
  int ret2 = this->SetVoltage((diffchannel*2+1), -1*(voltage/2)); 
  
  return (ret1 && ret2);  
}

double dbay32DAC::ReadVoltage(int channel){
  long value;
  double voltage;
  value = adc->read (channel);
  voltage = Ad7124Chip::toVoltage (value, 1, 2.5, true);
  Serial.println(voltage);
 return voltage;
}

/*int dbay32DAC::SetBase(double voltage){

  return 0;
}*/
