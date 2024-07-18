#include "dbay_dev.h"
#include "ltc268x.h" /* LTC268X definitions. */
#include "AD5684.h"
#include "ad7124.h"

#ifndef __DBAY_32DAC_H__
#define __DBAY_32DAC_H__


#define dbay32DAC_CS1  7
#define dbay32DAC_CLR1  6
#define dbay32DAC_LDAC1  5


#define dbay32DAC_CS2  4
#define dbay32DAC_CLR2  3
#define dbay32DAC_LDAC2  0

#define dbay32DAC_DACSYNC  1
#define dbay32DAC_OPAMPGAIN 3.2
#define dbay32DAC_ADC_CS  2

#define DACbaseRef 2.5

class dbay32DAC : public dbayDev{

  public:

    dbay32DAC(int address, TwoWire *bus);
    ~dbay32DAC();

    int SetVoltage (int channel, double voltage) override;
    int SetVoltageDiff(int diffchannel, double voltage) override;
    int reset() override;
    double ReadVoltage(int channel) override;
    //int SetBase(double voltage) override;
    
    DAC_AD5684 *DAC4ch;
    
    ltc268x *DAC1;
    ltc268x *DAC2;
    
     ltc268x_device_id dacid = LTC2688;
     uint16_t PWDdacSett = 0x0000;
     uint16_t ditherToggleEN = false;
     bool ditherMode[16];
     ltc268x_voltage_range DACRange[16];
     ltc268x_dither_phase ditherPhase[16];
     ltc268x_dither_period ditherPeriod[16];
     ltc268x_clk_input clkIn[16];
     ltc268x_a_b_register ABReg[16];

     Ad7124Chip *adc;

     DAC_AD5684 *dacBase;

};

#endif
