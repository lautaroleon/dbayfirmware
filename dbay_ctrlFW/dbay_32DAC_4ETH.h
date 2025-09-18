#include "dbay_dev.h"
#include "ltc268x.h" /* LTC268X definitions. */


#ifndef __DBAY_32DAC_4ETH_H__
#define __DBAY_32DAC_4ETH_H__


#define dbay32DAC_4ETH_CS1  6
#define dbay32DAC_4ETH_LDAC1  7


#define dbay32DAC_4ETH_CS2  4
#define dbay32DAC_4ETH_LDAC2  5



class dbay32DAC_4ETH : public dbayDev {

public:

    dbay32DAC_4ETH(int address, TwoWire* bus);
    ~dbay32DAC_4ETH();

    int SetVoltage(int channel, double voltage) override;
    int SetVoltageDiff(int diffchannel, double voltage) override;
    int reset() override;
    int diffChannelParser(int c);

    ltc268x* DAC1;
    ltc268x* DAC2;

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
