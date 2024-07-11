#ifndef AD5684_H
#define AD5684_H

#include <SPI.h>
#include "PCA9557.h"

//commands//
#define IN_REG_WRITE 0x1
#define IN_REG_TO_DAC_REG 0x2
#define WRITE_UPDATED_DAC 0x3
#define PWR_UPDWN 0x4
#define IN_REF 0x7
#define DCEN 0x8
#define READBACK 0x9
#define NOOP_DC 0xF

#define SPI_BAUD 10000
#define SPI_MODE SPI_MODE0

class DAC_AD5684
{
  public:
  
    DAC_AD5684(uint8_t ex_num_dac,
              PCA9557 *BoardSel, 
              int CSPin);
                                         
    ~DAC_AD5684();

    int set_V(int dac, int out, double value);
    int set_ref(int dac, float ref);

    
  private:
    uint8_t num_dacs;
    float* dacs_volts;
    float* dacsref; 
   // bool refs_loaded;
    char data_buff[3];
    char no_op[3] = {0xFF,0x00, 0x00};
    PCA9557 *_BoardSel;
    int _CSPin;
    char _err[1024];

  protected:
    int i2cadress;
    TwoWire *_bus;
};
#endif
