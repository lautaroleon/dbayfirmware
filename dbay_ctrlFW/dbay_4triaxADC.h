#include "dbay_dev.h"
#include "ad7124.h"

#ifndef __DBAY_4TRIAXADC_H__
#define __DBAY_4TRIAXADC_H__

#define dbay4ADC_ADC_CS  7

class dbay4triaxADC : public dbayDev{

    public:
        dbay4triaxADC(int address, TwoWire *bus);
        ~dbay4triaxADC();
        int reset() override;
        double ReadVoltage(int channel) override;
    private:
        Ad7124Chip *adc;
};

#endif