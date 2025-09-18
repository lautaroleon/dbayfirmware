#include "dbay_32DAC_4ETH.h"

dbay32DAC_4ETH::dbay32DAC_4ETH(int address, TwoWire* bus) :dbayDev(address, bus) {

    thisDeviceType = DAC4ETH;


    if (!BoardSel->pinMode(dbay32DAC_4ETH_CS1, OUTPUT)) {
        sprintf(_err, "Error setting pin mode for address 0x%1x ADC. Aborting...\n", BoardSel->boardN);
        Serial.print(_err);
        //rv = -1;
    }

    if (!BoardSel->pinMode(dbay32DAC_4ETH_LDAC1, OUTPUT)) {
        sprintf(_err, "Error setting pin mode for address 0x%1x ADC. Aborting...\n", BoardSel->boardN);
        Serial.print(_err);
        //rv = -1;
    }

    if (!BoardSel->digitalWrite(dbay32DAC_4ETH_LDAC1, HIGH)) {
        sprintf(_err, "Error setting address pin for board %i HIGH\n", BoardSel->boardN);
        Serial.print(_err);
    }


    if (!BoardSel->pinMode(dbay32DAC_4ETH_CS2, OUTPUT)) {
        sprintf(_err, "Error setting pin mode for address 0x%1x ADC. Aborting...\n", BoardSel->boardN);
        Serial.print(_err);
        //rv = -1;
    }

    if (!BoardSel->pinMode(dbay32DAC_4ETH_LDAC2, OUTPUT)) {
        sprintf(_err, "Error setting pin mode for address 0x%1x ADC. Aborting...\n", BoardSel->boardN);
        Serial.print(_err);
        //rv = -1;
    }

    if (!BoardSel->digitalWrite(dbay32DAC_4ETH_LDAC2, HIGH)) {
        sprintf(_err, "Error setting address pin for board %i HIGH\n", BoardSel->boardN);
        Serial.print(_err);
    }



    this->reset();

}

dbay32DAC_4ETH::~dbay32DAC_4ETH() {}

int dbay32DAC_4ETH::reset() {


    for (int j = 0; j < 16; j++) {
        ditherMode[j] = false;
        DACRange[j] = LTC268X_VOLTAGE_RANGE_M10V_10V;
        ditherPhase[j] = LTC268X_DITH_PHASE_0;
        ditherPeriod[j] = LTC268X_DITH_PERIOD_4;
        clkIn[j] = LTC268X_SOFT_TGL;
        ABReg[j] = LTC268X_SELECT_A_REG;
    }
    int CSpin1 = dbay32DAC_4ETH_CS1;
    int CSpin2 = dbay32DAC_4ETH_CS2;

    this->DAC1 = new ltc268x(dacid,
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
    //delay(3000);
    this->DAC2 = new ltc268x(dacid,
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



    return 0;
}


int dbay32DAC_4ETH::SetVoltage(int channel, double voltage) {



    Serial.print(channel);Serial.print("\t");
    Serial.println(voltage);


    if (voltage < -10 || voltage >10) {
        Serial.println("DAC voltage out of range");
        return -1;
    }
    else if (channel < 0 || channel >31) {
        Serial.println("DAC channel out of range");
        return -1;
    }
    else if (channel >= 0 && channel <= 15) {
            return (this->DAC1->set_voltage(channel, voltage));
    }else if (channel > 15 && channel < 32) {   
        return (this->DAC2->set_voltage(channel - 16, voltage));
    }else return -1;

}

int dbay32DAC_4ETH::SetVoltageDiff(int diffchannel, double voltage) {
    
       //Serial.print("diffchannel");Serial.println(diffchannel);
       //Serial.print("voltage");Serial.println(voltage);
     
    if (voltage < -20 || voltage >20) {
        Serial.print("DAC diff voltage out of range");
        return -1;
    }
    if (diffchannel < 0 || diffchannel >15) {//to do: group channels with negative channel parameter
        Serial.print("DAC diff channel out of range");
        return -1;
    }
    
    diffchannel = diffChannelParser(diffchannel);
    
    int ret1 = this->SetVoltage((diffchannel * 2), voltage / 2);
    int ret2 = 0;
    if(diffchannel == 1 || diffchannel == 5 || diffchannel == 9 || diffchannel == 13 ){
        ret2 = this->SetVoltage(((diffchannel+1) * 2 + 1), -1 * (voltage / 2));
    }else if(diffchannel == 2 || diffchannel == 6 || diffchannel == 10 || diffchannel == 14 ){
        ret2 = this->SetVoltage(((diffchannel-1) * 2 + 1), -1 * (voltage / 2));
    }else{
        ret2 = this->SetVoltage((diffchannel * 2 + 1), -1 * (voltage / 2));
    }

    return (ret1 && ret2);
}


int dbay32DAC_4ETH::diffChannelParser(int c){
    if(c>=12 && c<=15)return (c-12);
    if(c>=8 && c<=11)return (c-4);
    if(c>=4 && c<=7)return (c+4);
    if(c>=0 && c<=3)return (c+12);
    else return -1;
}