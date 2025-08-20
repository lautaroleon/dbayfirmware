#include "dbay_dev.h"
#include "ltc268x.h" /* LTC268X definitions. */

#ifndef __DBAY_4TRIACDAC_H__
#define __DBAY_4TRIACDAC_H__


#define dbay4triacDAC_CS  7
#define dbay4triacDAC_CLR  6
#define dbay4triacDAC_LDAC  5


class dbay4triacDAC : public dbayDev{

  public:
  
    dbay4triacDAC(int address, TwoWire *bus);
    ~dbay4triacDAC() {};
    int reset() override;
    int SetVoltage (int channel, double voltage) override;
    int SetVoltageDiff(int diffchannel, double voltage) override;

    ltc268x *DAC;

    ltc268x_device_id dacid = LTC2688;
   uint16_t PWDdacSett = 0x0000;
   uint16_t ditherToggleEN = false;
   bool ditherMode[16];
   ltc268x_voltage_range DACRange[16];
   ltc268x_dither_phase ditherPhase[16];
   ltc268x_dither_period ditherPeriod[16];
   ltc268x_clk_input clkIn[16];
   ltc268x_a_b_register ABReg[16];
};

#endif
